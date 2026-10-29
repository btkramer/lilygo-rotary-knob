// screen_nav.cpp — Shared screen objects, return-screen state, restoreScreen(), screen timeouts and swipe gestures.

#include <Arduino.h>
#include <lvgl.h>
#include <WebSerial.h>
#include "input.h"
#include "screen_nav.h"
#include "screen_volume.h"
#include "screen_now_playing.h"
#include "screen_quick_jump.h"

lv_obj_t *homeScreen;
lv_obj_t *volumeScreen;
// Declared together with volumeScreen (rather than where each is actually
// built, further down) so the return-screen variables below can reference
// all three transient screens even though Speaker Mute's and Quick-jump's
// own build code comes later in the file. nowPlayingScreen and
// lastNowPlayingActivityMs (moved up here 2026-08-22, T35 fix) join them
// for the same reason: restoreScreen() below needs both.
lv_obj_t *speakerMuteScreen;
lv_obj_t *quickJumpScreen;
lv_obj_t *nowPlayingScreen;
uint32_t lastNowPlayingActivityMs = 0;

// Real bug, found and fixed three times before landing here. First attempt
// (2026-08-07): a single shared `previousScreen` variable, set by whichever
// of Volume/Speaker Mute/Quick-jump was entered most recently. Broke two
// ways: (1) entering one transient screen from another (e.g. pressing mute
// while looking at Volume) could set previousScreen to a transient screen
// itself, and once that screen's own fuse later fired using a stale
// previousScreen still pointing at itself, it reloaded onto itself — a no-op
// that looked exactly like "the timeout doesn't work". Fixed *that* by
// refusing to ever store a transient screen as previousScreen — but that
// broke the case where Speaker Mute genuinely needs to stay the eventual
// landing spot while muted (see below). The second fix: each transient
// screen gets its OWN dedicated "what am I covering" variable instead of
// sharing one, allowing Volume/Mute to interrupt each other and unwind back
// correctly — quickJumpReturnScreen follows the same pattern and is
// unaffected by anything below (Quick-jump is a real, intentionally-browsed
// menu, not a transient auto-triggered overlay like these two).
//
// Third fix (Brian, 2026-08-19): unconditional chaining between Volume and
// Mute was itself wrong — turning the volume knob then unmuting (or vice
// versa) visibly bounced through the *other* overlay for its own fuse
// duration before finally reaching the real screen underneath, since each
// treated "whatever's currently on screen" as a valid thing to return to
// even when that was the other transient overlay. Volume is always purely
// transient (never worth returning to for its own sake — see
// showVolumeOverlay()), so Mute unconditionally skips straight past it.
// Mute is different: while actively muted, it's Brian's own confirmed
// deliberate-persistent-state design (`speakerMuted` — "should not return to
// the main screen... only after unmuting"), so a Volume interruption started
// *while still muted* still needs to land back on Mute afterward or the
// mute indicator would vanish behind Home despite audio staying muted; only
// skipped when not muted (an about-to-auto-dismiss moment, not a
// deliberate state) — see showVolumeOverlay()'s own comment for exactly how.
lv_obj_t *volumeReturnScreen;
lv_obj_t *speakerMuteReturnScreen;
// Declared here (not down by Speaker Mute's own screen code) since
// showVolumeOverlay() below needs it in scope — local estimate only, HID
// mute is a toggle with no readback.
bool speakerMuted = false;
uint32_t lastMuteLocalWriteMs = 0; // T28 — see lastVolumeLocalWriteMs's comment
lv_obj_t *quickJumpReturnScreen;

// Loads `target` and, if target is itself one of the three transient
// screens, refreshes its own idle-fuse timer — otherwise re-landing on it
// via another overlay's fuse would find a stale timer and immediately
// bounce onward again instead of giving it a fresh window (e.g. you were
// browsing Quick-jump, got interrupted by a Volume adjustment, and want a
// full fresh look at the grid again once Volume settles, not an instant
// second bounce).
void restoreScreen(lv_obj_t *target) {
  lv_screen_load(target);
  uint32_t now = millis();
  if (target == volumeScreen) lastVolumeActivityMs = now;
  else if (target == speakerMuteScreen) lastSpeakerMuteActivityMs = now;
  else if (target == quickJumpScreen) lastQuickJumpActivityMs = now;
  // T35 fix (2026-08-22): nowPlayingScreen was missing from this list.
  // Confirmed via live capture the real bug this caused: Quick-jump's own
  // 8s auto-return fuse (QUICK_JUMP_RETURN_MS) calls restoreScreen() to
  // land back on Now Playing if that's where Quick-jump was opened from —
  // without this branch, lastNowPlayingActivityMs stayed at whatever it
  // was *before* the Quick-jump detour, so the very next check of Now
  // Playing's own idle fuse (see loop()) could already be past even the
  // shorter NOW_PLAYING_STOPPED_IDLE_MS threshold, firing near-instantly
  // instead of giving a real fresh window — this is what made T34's
  // playing/stopped switch look like it wasn't taking effect.
  else if (target == nowPlayingScreen) lastNowPlayingActivityMs = now;
}

uint32_t idleToHomeMs = IDLE_TO_HOME_MS_DEFAULT;

uint32_t dimDelayMs = DIM_DELAY_MS_DEFAULT;
// Tracks "is the panel currently dimmed" independently of which screen ends
// up active — see loop()'s idle-state machine for why this can't just be
// `lv_screen_active() == idleScreen`: a rotation/button press while dimmed
// jumps straight to Volume/Speaker Mute (showVolumeOverlay()/
// showSpeakerMuteOverlay(), earlier in the same loop() pass) *before* the
// idle-state check ever runs, so by the time it does, the active screen is
// already something other than idleScreen even though brightness still
// needs restoring.
bool displayDimmed = false;

// A gesture stops bubbling up at the first child whose own
// LV_OBJ_FLAG_GESTURE_BUBBLE isn't set (see lv_indev.c's indev_gesture()) —
// so a swipe starting ON a widget (not empty background) would otherwise be
// swallowed by that widget instead of ever reaching the screen-level
// LV_EVENT_GESTURE handler below. Arcs mostly dodge this on their own (their
// hit test only counts the ring stroke, not their full bounding box — see
// lv_arc.c's LV_EVENT_HIT_TEST), but labels and images hit-test their whole
// bounding box, and the ones here (CPU/GPU/RAM numbers, the mute icon) sit
// centered — exactly where a swipe naturally starts. Every screen's content
// is non-interactive display-only, none of it needs to claim gestures for
// itself, so mark it all bubble-through recursively.
void makeGestureBubbleThrough(lv_obj_t *obj) {
  uint32_t n = lv_obj_get_child_count(obj);
  for (uint32_t i = 0; i < n; i++) {
    lv_obj_t *child = lv_obj_get_child(obj, i);
    lv_obj_add_flag(child, LV_OBJ_FLAG_GESTURE_BUBBLE);
    makeGestureBubbleThrough(child);
  }
}

// Fires for gestures on Home, Volume, and Speaker Mute (not the grid's own
// screen — nothing to do there). LV_DIR_TOP is LVGL's gesture direction for
// a finger dragging upward, i.e. a swipe up. Subscribed on the indev itself
// (see setupLVGL), not any one screen object — LVGL sends LV_EVENT_GESTURE
// to both the "gesture_obj" (whichever object stopped the bubble-up walk,
// see makeGestureBubbleThrough's comment) AND the indev directly
// (lv_indev.c's indev_gesture(): `lv_indev_send_event(indev_act,
// LV_EVENT_GESTURE, gesture_obj)`), so subscribing here catches every
// gesture regardless of which widget it lands on, sidestepping the
// bubbling question entirely.
void quickJumpGestureCb(lv_event_t *e) {
  lv_indev_t *indev = lv_indev_active();
  if (!indev) return;
  lv_dir_t dir = lv_indev_get_gesture_dir(indev);
  lv_obj_t *active = lv_screen_active();
  if (dir == LV_DIR_TOP && (active == homeScreen || active == volumeScreen ||
                             active == speakerMuteScreen || active == nowPlayingScreen)) {
    showQuickJumpMenu();
  }
}

// Swipe left/right toggles between Home and Now Playing — the only two
// "normal" (non-overlay) screens today, so a straight toggle is complete and
// correct (Brian confirmed 2026-08-17: Home participates in the cycle,
// matching the original mockup's legend — "Swipe left/right: cycle screens,
// Volume excluded" — nothing in it excludes Home). Volume and Speaker Mute
// aren't stops on that swipe ring, same reasoning as Quick-jump: they're
// transient global-input overlays. Extend the Home/Now Playing toggle to
// genuine ordered N-way cycling once a third such screen exists — not worth
// the abstraction for two. Subscribed on the indev directly, same proven
// approach as quickJumpGestureCb above.
//
// On Volume/Speaker Mute specifically, though, left/right swipe *is*
// meaningful: an explicit dismiss, straight to the resolved return screen
// (Brian, 2026-08-19) — skips waiting out the fuse rather than cycling
// screens (there's nothing to cycle to from an overlay). Goes through
// restoreScreen() so a dismiss onto another transient screen still on its
// own countdown (Mute, if muted) gets a fresh timer, same as the fuse path.
// Unconditional regardless of `speakerMuted` — a deliberate user gesture
// should always work, unlike the passive idle-timeout fuse which
// deliberately withholds auto-return while muted.
void screenCycleGestureCb(lv_event_t *e) {
  lv_indev_t *indev = lv_indev_active();
  if (!indev) return;
  lv_dir_t dir = lv_indev_get_gesture_dir(indev);
  // T30 root cause CONFIRMED (2026-08-22) via two live WebSerial captures:
  // a single physical swipe dispatched this callback TWICE (same dir,
  // ~60-75ms apart, activeBefore differing between the two calls — the
  // second one already reflecting the screen change the first one made),
  // despite this file's prior belief that LVGL's gesture_sent guard makes
  // dispatch one-shot per gesture. Since this function is a strict
  // Home<->NowPlaying toggle, a spurious second dispatch for the same
  // swipe immediately undoes whatever the first one just did — exactly
  // Brian's "flashes to the new screen, then reverts" symptom. Fixed below
  // by ignoring a second dispatch for the same physical touch-down
  // (touchPressCounter, bumped once per PRESSED edge in lvTouchpadRead()),
  // rather than chasing why LVGL sends it twice. Debug logging (every
  // dispatch, including bailed ones) kept for now to confirm the fix
  // actually suppresses the duplicate on the next live retest — remove
  // once confirmed.
  static uint32_t lastHandledPressCounter = 0;
  bool duplicateForThisTouch = touchPressCounter == lastHandledPressCounter;
  lv_obj_t *activeBefore = lv_screen_active();
  Serial.printf("gestureCb: t=%ums dir=%d activeBefore=%p dup=%d\n", millis(), (int)dir, (void *)activeBefore, duplicateForThisTouch);
  WebSerial.printf("gestureCb: t=%ums dir=%d activeBefore=%p dup=%d\n", millis(), (int)dir, (void *)activeBefore, duplicateForThisTouch);
  if (dir != LV_DIR_LEFT && dir != LV_DIR_RIGHT) return;
  if (duplicateForThisTouch) return;
  lastHandledPressCounter = touchPressCounter;
  lv_obj_t *active = lv_screen_active();
  if (active == homeScreen) {
    showNowPlayingScreen();
  } else if (active == nowPlayingScreen) {
    lv_screen_load(homeScreen);
  } else if (active == volumeScreen) {
    restoreScreen(volumeReturnScreen);
  } else if (active == speakerMuteScreen) {
    restoreScreen(speakerMuteReturnScreen);
  }
}
