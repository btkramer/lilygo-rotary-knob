// screen_now_playing.cpp — Now Playing screen and its HID transport controls.

#include <Arduino.h>
#include <lvgl.h>
#include <WebSerial.h>
#include "USBHIDConsumerControl.h"
#include "pin_config.h"
#include "hid_media.h"
#include "pc_state.h"
#include "screen_nav.h"
#include "screen_home.h"
#include "screen_now_playing.h"

// Now Playing: the last of the original 5-screen scope (2026-08-17). Unlike
// Volume/Speaker Mute, this is a normal swipeable screen, not a global-input
// overlay — reached via its Quick-jump spot or swipe left/right from Home,
// and returns to Home on the standard 30s idle timeout (see
// idleToHomeMs below) rather than an overlay-style "restore whatever
// it interrupted" fuse.
//
// Play/pause and prev/next are real USB HID (CONSUMER_CONTROL_PLAY_PAUSE /
// SCAN_NEXT / SCAN_PREVIOUS) — work today with zero companion-app dependency,
// same HID-first guarantee as Volume/Mute. Track title/artist/elapsed/
// progress are placeholders (no companion app yet to report real now-playing
// state — see PROGRESS.md's "Next" list), matching Home's CPU/GPU/RAM and
// Volume's "SPEAKERS" device-name placeholders. Values match the original
// mockup (https://claude.ai/code/artifact/659dd01a-d6ab-42d9-8998-
// 5f99360b04ad) exactly, same as every other screen's placeholder data.
constexpr uint32_t NOW_PLAYING_ACCENT_COLOR = 0x4FB477; // same green as the mockup's Now Playing card

// Defaults shown until the first pcConnected poll reconciles them (see
// loop()'s sync-apply step) — String, not const char*, so a real synced
// value (arbitrary length, e.g. actual SMTC track titles) can replace them.
// lv_label_set_text() copies internally, so re-pointing these on every poll
// is safe even while a label is currently on screen.
String nowPlayingArtist = "Boards of Canada";
String nowPlayingTrack = "Roygbiv";
char nowPlayingElapsed[24] = "1:42/3:45";
// Placeholder for the source app (Brian, 2026-08-19, e.g. "YouTube") —
// SMTC exposes which app owns the active media session once pc-dash-app
// exists (see ECOSYSTEM_PLAN.md's "YouTube-aware pause vs. mute" section);
// per-app, not per-tab/site, so a browser tab may only ever show as the
// browser itself unless the site's own Media Session metadata disambiguates
// it further. Same "no companion app yet" placeholder pattern as artist/
// track/elapsed above.
String nowPlayingSource = "YouTube";
int nowPlayingProgressPercent = 38;
// Raw ms. T29 (2026-08-21): this is now this device's own locally-ticking
// clock while nowPlayingIsPlaying (advanced 1s at a time in loop(), frozen
// while paused) — mirrors pc-dash-app's own T25 position-extrapolation fix,
// one hop further down the pipeline — rather than only ever moving when a
// poll happens to carry a fresher value (sync poll's granularity, ~1s, was
// visibly a few seconds behind real playback).
//
// Reconciliation (2026-08-21, Brian's design): rather than blindly snapping
// to whatever a single poll says on a flat timer (original T29 approach,
// `POSITION_RECONCILE_INTERVAL_MS`, since replaced), a poll's position is
// only trusted as a real correction once it disagrees with this local clock
// by more than `POSITION_DRIFT_TOLERANCE_MS` on `POSITION_DRIFT_CONFIRM_COUNT`
// *consecutive* polls — one noisy/glitchy poll can't yank the display, but a
// genuine, sustained drift still corrects within a couple of poll cycles.
// Unlike the equivalent idea for nowPlayingIsPlaying (deliberately NOT
// applied there — see loop()'s own comment on why a boolean field can't
// distinguish "real but slow to confirm" from "never happening" this way),
// position doesn't have that ambiguity: it's continuously reported based on
// real elapsed playback time, not a discrete pending transition, so there's
// no equivalent "legitimately unconfirmed" state to misfire on.
uint32_t nowPlayingPositionMs = 0;
uint32_t nowPlayingDurationMs = 225000;
uint32_t lastPositionTickMs = 0; // millis() the local clock last advanced by one tick
uint32_t positionDriftStreak = 0; // consecutive polls in a row disagreeing with the local clock by more than tolerance

// "current/total" (e.g. "1:23/3:45", T22) from raw ms values.
void formatNowPlayingElapsed(uint32_t positionMs, uint32_t durationMs) {
  uint32_t posSec = positionMs / 1000;
  uint32_t durSec = durationMs / 1000;
  snprintf(nowPlayingElapsed, sizeof(nowPlayingElapsed), "%u:%02u/%u:%02u",
           posSec / 60, posSec % 60, durSec / 60, durSec % 60);
}

// T21 (Brian, 2026-08-21): an apostrophe rendered as a solid missing-glyph
// square on real track titles. SMTC/Windows track metadata commonly uses
// Unicode curly quotes/dashes (e.g. U+2019 right single quote) rather than
// plain ASCII, and the built-in LVGL fonts here (lv_font_montserrat_*) only
// cover a basic Latin glyph range — no extended-punctuation build option was
// enabled, and rebuilding a custom font atlas is a lot of overhead for this.
// Normalizing on receipt (firmware side, not pc-dash-app) means any future
// data source with the same habit is covered too, not just SMTC.
String normalizeSmartPunctuation(const String &in) {
  String out;
  out.reserve(in.length());
  for (size_t i = 0; i < in.length(); ) {
    uint8_t c = in[i];
    if (c == 0xE2 && i + 2 < in.length() && (uint8_t)in[i + 1] == 0x80) {
      uint8_t c2 = in[i + 2];
      switch (c2) {
        case 0x98: case 0x99: out += '\''; i += 3; continue; // ‘ ’
        case 0x9C: case 0x9D: out += '"';  i += 3; continue; // “ ”
        case 0x93: case 0x94: out += '-';  i += 3; continue; // – —
        case 0xA6: out += "...";           i += 3; continue; // …
        default: break;
      }
    }
    out += (char)c;
    i++;
  }
  return out;
}
// No HID readback exists for actual playback state — this is a local guess,
// toggled optimistically on each play/pause tap, same "local estimate only"
// pattern as volumePercent/speakerMuted. Icon shows the action a tap will
// perform next (pause glyph while assumed playing), the standard media-
// player convention — deliberately different from Speaker Mute's
// icon-shows-current-state convention, since transport controls are a
// different, well-established UI vocabulary.
bool nowPlayingIsPlaying = true;
uint32_t lastPlayPauseLocalWriteMs = 0; // T28 — see lastVolumeLocalWriteMs's comment
// nowPlayingScreen and lastNowPlayingActivityMs now declared up with
// homeScreen/volumeScreen (see that comment) — restoreScreen() needs them.

lv_obj_t *nowPlayingArc;
lv_obj_t *nowPlayingIcon;
lv_obj_t *nowPlayingPrevIcon;
lv_obj_t *nowPlayingNextIcon;
lv_obj_t *nowPlayingSourceLabel;
lv_obj_t *nowPlayingArtistLabel;
lv_obj_t *nowPlayingTrackLabel;
lv_obj_t *nowPlayingElapsedLabel;
lv_obj_t *nowPlayingOfflineLabel;
// Behind each transport icon — see nowPlayingPulseGlow's own comment.
lv_obj_t *nowPlayingPrevGlow;
lv_obj_t *nowPlayingPlayPauseGlow;
lv_obj_t *nowPlayingNextGlow;

void updateNowPlayingScreen() {
  lv_arc_set_value(nowPlayingArc, nowPlayingProgressPercent);
  lv_label_set_text(nowPlayingIcon, nowPlayingIsPlaying ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY);
  lv_label_set_text(nowPlayingSourceLabel, nowPlayingSource.c_str());
  lv_label_set_text(nowPlayingArtistLabel, nowPlayingArtist.c_str());
  lv_label_set_text(nowPlayingTrackLabel, nowPlayingTrack.c_str());
  lv_label_set_text(nowPlayingElapsedLabel, nowPlayingElapsed);

  // Same PC-offline dim treatment as Home (see pcConnected's own comment and
  // PC_OFFLINE_DIM_ENABLED, currently suppressing it for testing) —
  // track/artist/progress are all PC-sourced, transport controls (HID) keep
  // working regardless since they don't depend on this channel.
  bool showConnected = pcConnected || !PC_OFFLINE_DIM_ENABLED;
  lv_opa_t opa = showConnected ? LV_OPA_COVER : LV_OPA_30;
  lv_obj_set_style_arc_opa(nowPlayingArc, opa, LV_PART_INDICATOR);
  lv_obj_set_style_text_opa(nowPlayingSourceLabel, opa, 0);
  lv_obj_set_style_text_opa(nowPlayingArtistLabel, opa, 0);
  lv_obj_set_style_text_opa(nowPlayingTrackLabel, opa, 0);
  lv_obj_set_style_text_opa(nowPlayingElapsedLabel, opa, 0);
  if (showConnected) {
    lv_obj_add_flag(nowPlayingOfflineLabel, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_remove_flag(nowPlayingOfflineLabel, LV_OBJ_FLAG_HIDDEN);
  }
}

void nowPlayingGlowAnimCb(void *obj, int32_t value) {
  lv_obj_set_style_bg_opa((lv_obj_t *)obj, (lv_opa_t)value, 0);
}

// A plain circle behind a transport icon, opacity-animated — same safe
// "plain object, never an arc" pattern as speakerMuteRingPulse (see its own
// comment: small decorative arcs produced tearing here even under
// LV_DISPLAY_RENDER_MODE_FULL). Starts fully transparent; nowPlayingPulseGlow
// below is what actually flashes it.
lv_obj_t *makeNowPlayingGlow(lv_obj_t *parent, int16_t xOffset, int16_t yOffset, int16_t diameter) {
  lv_obj_t *glow = lv_obj_create(parent);
  lv_obj_remove_flag(glow, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(glow, LV_OBJ_FLAG_CLICKABLE); // purely decorative — the tap zones on top own all input here
  lv_obj_set_size(glow, diameter, diameter);
  lv_obj_set_style_radius(glow, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(glow, 0, 0);
  lv_obj_set_style_bg_color(glow, lv_color_hex(NOW_PLAYING_ACCENT_COLOR), 0);
  lv_obj_set_style_bg_opa(glow, LV_OPA_TRANSP, 0);
  lv_obj_align(glow, LV_ALIGN_CENTER, xOffset, yOffset);
  return glow;
}

// Brief glow flash behind a transport icon on tap — visual confirmation the
// tap registered (Brian, 2026-08-19: wanted "some indication that one was
// tapped, like a soft pulse glow"). Prev/Next especially have no persistent
// on-screen state to change on tap the way Play/Pause's own icon swaps —
// HID has no readback for "did skip actually happen" — so without this a
// prev/next tap gave zero visible feedback at all. One-shot pulse, not a
// loop — a momentary acknowledgment, unlike speakerMuteRingPulse's infinite
// pulse for an ongoing persistent state.
//
// Fades *in* then out (not an instant jump to peak opacity) — the first cut
// of this jumped straight to LV_OPA_60 and only animated the fade-down,
// which read as choppy (Brian, 2026-08-19). A real pulse ramps both ways;
// fade-in is quicker than fade-out so the glow still reads as an immediate
// response to the tap, not a delayed one, while the exit stays slow and
// soft.
void nowPlayingPulseGlow(lv_obj_t *glow) {
  lv_anim_delete(glow, nowPlayingGlowAnimCb); // retapping mid-pulse restarts cleanly instead of stacking anims
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, glow);
  lv_anim_set_exec_cb(&a, nowPlayingGlowAnimCb);
  lv_anim_set_values(&a, LV_OPA_TRANSP, LV_OPA_60);
  lv_anim_set_time(&a, 220);            // fade in
  lv_anim_set_playback_time(&a, 700);   // fade back out — slower than the fade-in, a soft glow rather than a flash
  lv_anim_set_repeat_count(&a, 1);      // one-shot, not a loop (LV_ANIM_REPEAT_INFINITE is speakerMuteRingPulse's job)
  lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
  lv_anim_start(&a);
}

// LVGL still fires LV_EVENT_CLICKED on release even when the touch was
// actually a swipe, since the tap zones are deliberately non-scrollable
// (see makeNowPlayingTapZone's own comment) — LVGL only withholds CLICKED
// when something actually scrolled in response to the drag (lv_indev.c:
// `if (scroll_obj == NULL) { ... send CLICKED }`), and nothing here ever
// scrolls. Swiping up to Quick-jump from this screen (added 2026-08-19) was
// firing a spurious play/pause/prev/next underneath it as a result. Gesture
// direction is set mid-drag, before release (lv_indev.c's indev_gesture(),
// called every read while pressed) and stays set through the release that
// fires CLICKED, so checking it here reliably tells a real tap apart from a
// swipe that happened to start on one of these zones.
bool nowPlayingTapWasGesture() {
  lv_indev_t *indev = lv_indev_active();
  return indev && lv_indev_get_gesture_dir(indev) != LV_DIR_NONE;
}

void nowPlayingPrevCb(lv_event_t *e) {
  if (nowPlayingTapWasGesture()) return;
  ConsumerControl.press(CONSUMER_CONTROL_SCAN_PREVIOUS);
  ConsumerControl.release();
  nowPlayingPulseGlow(nowPlayingPrevGlow);
  lastNowPlayingActivityMs = millis();
  Serial.println("MEDIA_PREV");
  WebSerial.println("MEDIA_PREV");
}

void nowPlayingNextCb(lv_event_t *e) {
  if (nowPlayingTapWasGesture()) return;
  ConsumerControl.press(CONSUMER_CONTROL_SCAN_NEXT);
  ConsumerControl.release();
  nowPlayingPulseGlow(nowPlayingNextGlow);
  lastNowPlayingActivityMs = millis();
  Serial.println("MEDIA_NEXT");
  WebSerial.println("MEDIA_NEXT");
}

void nowPlayingPlayPauseCb(lv_event_t *e) {
  if (nowPlayingTapWasGesture()) return;
  ConsumerControl.press(CONSUMER_CONTROL_PLAY_PAUSE);
  ConsumerControl.release();
  nowPlayingIsPlaying = !nowPlayingIsPlaying;
  lastPlayPauseLocalWriteMs = millis(); // T28: don't let a lagging poll response stomp this
  lastPositionTickMs = millis(); // T29: resume/pause here, not mid-tick — avoids a catch-up burst on resume
  updateNowPlayingScreen();
  nowPlayingPulseGlow(nowPlayingPlayPauseGlow);
  lastNowPlayingActivityMs = millis();
  Serial.println("MEDIA_PLAY_PAUSE");
  WebSerial.println("MEDIA_PLAY_PAUSE");
}

// Three invisible full-height tap zones (left/center/right thirds) — matches
// the mockup's "Click: play/pause, Tap L/R: previous/next" caption. Added
// after the rest of the screen's content so they sit on top in z-order;
// doesn't actually matter here since nothing else on this screen is
// clickable, but keeps click delivery unambiguous. No explicit
// LV_OBJ_FLAG_GESTURE_BUBBLE needed — swipe left/right is caught at the
// indev level (see screenCycleGestureCb below), same proven-working approach
// as Quick-jump's swipe-up, so per-child bubbling isn't load-bearing here.
lv_obj_t *makeNowPlayingTapZone(lv_obj_t *parent, int16_t x, int16_t w) {
  lv_obj_t *zone = lv_obj_create(parent);
  lv_obj_remove_flag(zone, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_opa(zone, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(zone, 0, 0);
  lv_obj_set_style_pad_all(zone, 0, 0);
  lv_obj_set_size(zone, w, SCREEN_HEIGHT);
  lv_obj_set_pos(zone, x, 0);
  return zone;
}

void addNowPlayingTapZones(lv_obj_t *scr) {
  constexpr int16_t ZONE_W = SCREEN_WIDTH / 3;
  lv_obj_t *prevZone = makeNowPlayingTapZone(scr, 0, ZONE_W);
  lv_obj_add_event_cb(prevZone, nowPlayingPrevCb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *playPauseZone = makeNowPlayingTapZone(scr, ZONE_W, ZONE_W);
  lv_obj_add_event_cb(playPauseZone, nowPlayingPlayPauseCb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *nextZone = makeNowPlayingTapZone(scr, ZONE_W * 2, SCREEN_WIDTH - ZONE_W * 2);
  lv_obj_add_event_cb(nextZone, nowPlayingNextCb, LV_EVENT_CLICKED, NULL);
}

void buildNowPlayingScreen(lv_obj_t *scr) {
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  // Same ring size as Volume/Speaker Mute for visual consistency across
  // every screen that centers one accent-colored ring.
  nowPlayingArc = makeRingGauge(scr, 350, 32, lv_color_hex(NOW_PLAYING_ACCENT_COLOR));

  // Top caption shows the source (e.g. "YouTube") instead of a generic
  // "MEDIA" label (Brian, 2026-08-19: "the source should replace the word
  // 'media' on top" — the separate small source line below the buttons,
  // added earlier this session, read as too tiny regardless of font size at
  // that position; this reuses the caption's existing prominent top spot and
  // font size instead of trying to enlarge a line squeezed under the
  // buttons). Font bumped 22px→24px same session, second pass ("slightly
  // larger").
  nowPlayingSourceLabel = lv_label_create(scr);
  lv_obj_set_style_text_font(nowPlayingSourceLabel, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(nowPlayingSourceLabel, lv_color_hex(NOW_PLAYING_ACCENT_COLOR), 0);
  lv_label_set_text(nowPlayingSourceLabel, nowPlayingSource.c_str());
  lv_obj_align(nowPlayingSourceLabel, LV_ALIGN_CENTER, 0, -85);

  // Layout, revised again (Brian, 2026-08-19): the prior pass put the
  // transport row at dead-center (y=0); Brian clarified he didn't want that
  // literally — picture a horizontal line through screen center, buttons
  // just *above* it, media details (artist/track) below it. Row moved to
  // y=-35, prev/next pulled in from ±130 to ±100 (closer to Play/Pause, per
  // Brian: "bring the prev/next buttons in a little"). Glow diameters
  // unchanged (70/90) — still clear of each other at ±100 (closest edge gap
  // ±100 - 45 - 35 = 20px).
  nowPlayingPrevGlow = makeNowPlayingGlow(scr, -100, -35, 70);
  nowPlayingPlayPauseGlow = makeNowPlayingGlow(scr, 0, -35, 90);
  nowPlayingNextGlow = makeNowPlayingGlow(scr, 100, -35, 70);

  // Prev/Next/Play-Pause as a standard transport-control row (Brian,
  // 2026-08-19: wanted on-screen indication of where the prev/next tap zones
  // are — previously just invisible thirds of the screen with no visual
  // marker at all, matching only the mockup's caption text, not an icon).
  // Smaller than Play/Pause to keep it the visual primary action, standard
  // media-player hierarchy.
  nowPlayingPrevIcon = lv_label_create(scr);
  lv_obj_set_style_text_font(nowPlayingPrevIcon, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(nowPlayingPrevIcon, lv_color_hex(NOW_PLAYING_ACCENT_COLOR), 0);
  lv_label_set_text(nowPlayingPrevIcon, LV_SYMBOL_PREV);
  lv_obj_align(nowPlayingPrevIcon, LV_ALIGN_CENTER, -100, -35);

  nowPlayingIcon = lv_label_create(scr);
  lv_obj_set_style_text_font(nowPlayingIcon, &lv_font_montserrat_42, 0);
  lv_obj_set_style_text_color(nowPlayingIcon, lv_color_hex(NOW_PLAYING_ACCENT_COLOR), 0);
  lv_obj_align(nowPlayingIcon, LV_ALIGN_CENTER, 0, -35);

  nowPlayingNextIcon = lv_label_create(scr);
  lv_obj_set_style_text_font(nowPlayingNextIcon, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(nowPlayingNextIcon, lv_color_hex(NOW_PLAYING_ACCENT_COLOR), 0);
  lv_label_set_text(nowPlayingNextIcon, LV_SYMBOL_NEXT);
  lv_obj_align(nowPlayingNextIcon, LV_ALIGN_CENTER, 100, -35);

  // Artist (brighter, biggest of the three — Brian confirmed this size is
  // fine) above Track above Elapsed, each their own line now (Brian,
  // 2026-08-19, second pass: Track+Elapsed used to share one line — "since
  // time is short, it can move to a line below... and the time and the line
  // above it can get bigger" — splitting freed enough horizontal room to
  // bump both). Ordering still matches the mockup's original artist-above-
  // track layout, re-checked directly against the fetched artifact rather
  // than guessed (this project's standing lesson) — Elapsed is new, appended
  // after rather than disturbing that. All sit below the center line.
  nowPlayingArtistLabel = lv_label_create(scr);
  lv_obj_set_style_text_font(nowPlayingArtistLabel, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(nowPlayingArtistLabel, lv_color_hex(0xE7ECE9), 0);
  lv_label_set_text(nowPlayingArtistLabel, nowPlayingArtist.c_str());
  lv_obj_align(nowPlayingArtistLabel, LV_ALIGN_CENTER, 0, 25);

  // Fonts bumped one step each (Brian, 2026-08-19, third pass). Gaps pulled
  // back in a bit on the 4th pass — round 3's 40px/40px read as too even;
  // Brian preferred round 2's staggered look (first gap bigger than the
  // second: 33px/28px) but wanted the overall spacing a bit more open than
  // that. Landed in between, staggered: Artist-to-Track 36px, Track-to-
  // Elapsed 30px. Still comfortably inside the round display's visible
  // chord at these heights (y=91's half-width is ~172px, offline label
  // stays clear below).
  nowPlayingTrackLabel = lv_label_create(scr);
  lv_obj_set_style_text_font(nowPlayingTrackLabel, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(nowPlayingTrackLabel, lv_color_hex(0x9AA5A3), 0);
  // T16 (2026-08-21): real track titles (SMTC-sourced, not this project's own
  // placeholder data) run well past the panel — LVGL's circular-scroll long
  // mode marquees automatically only when the text actually overflows this
  // fixed width (short titles stay static/centered, no animation), so this
  // needs a real width instead of the default content-auto-sized one. 320px
  // stays safely inside the round display's visible chord at this label's
  // y=61 offset (~370px full chord — see the Elapsed label's own comment
  // below for the same chord-math pattern at y=91).
  lv_obj_set_width(nowPlayingTrackLabel, 320);
  lv_obj_set_style_text_align(nowPlayingTrackLabel, LV_TEXT_ALIGN_CENTER, 0);
  // T20 (Brian, 2026-08-21): default circular-scroll speed (LVGL's built-in
  // LV_LABEL_DEF_SCROLL_SPEED, speed=40) read as too fast on-device. Halving
  // the speed value doubles scroll duration for any given title's overflow
  // distance (this is speed, not a fixed duration, so it still scales with
  // title length rather than making long titles zip by proportionally faster).
  lv_obj_set_style_anim_duration(nowPlayingTrackLabel, lv_anim_speed_clamped(20, 300, 10000), 0);
  lv_label_set_long_mode(nowPlayingTrackLabel, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);
  lv_label_set_text(nowPlayingTrackLabel, nowPlayingTrack.c_str());
  lv_obj_align(nowPlayingTrackLabel, LV_ALIGN_CENTER, 0, 61);

  nowPlayingElapsedLabel = lv_label_create(scr);
  lv_obj_set_style_text_font(nowPlayingElapsedLabel, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(nowPlayingElapsedLabel, lv_color_hex(0x9AA5A3), 0);
  lv_label_set_text(nowPlayingElapsedLabel, nowPlayingElapsed);
  lv_obj_align(nowPlayingElapsedLabel, LV_ALIGN_CENTER, 0, 91);

  nowPlayingOfflineLabel = lv_label_create(scr);
  lv_obj_set_style_text_font(nowPlayingOfflineLabel, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(nowPlayingOfflineLabel, lv_color_hex(0x9AA5A3), 0);
  lv_label_set_text(nowPlayingOfflineLabel, "PC OFFLINE");
  lv_obj_align(nowPlayingOfflineLabel, LV_ALIGN_BOTTOM_MID, 0, -20);
  lv_obj_add_flag(nowPlayingOfflineLabel, LV_OBJ_FLAG_HIDDEN); // shown by updateNowPlayingScreen() once pcConnected is known

  addNowPlayingTapZones(scr);
  updateNowPlayingScreen();
}

// Entry point for Now Playing (Quick-jump spot, swipe left/right) — mirrors
// showQuickJumpMenu()'s shape but with no return-screen memory needed, since
// this screen always falls back to Home on idle, never to "whatever it
// interrupted" (it's not an overlay).
void showNowPlayingScreen() {
  lv_screen_load(nowPlayingScreen);
  lastNowPlayingActivityMs = millis();
}
