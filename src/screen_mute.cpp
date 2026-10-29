// screen_mute.cpp — Speaker-mute overlay screen.

#include <Arduino.h>
#include <lvgl.h>
#include "USBHIDConsumerControl.h"
#include "hid_media.h"
#include "screen_nav.h"
#include "screen_home.h"
#include "screen_idle.h"
#include "screen_volume.h"
#include "screen_mute.h"

// Speaker Mute screen: originally scoped from the mockup's "Mic Mute" card,
// but retargeted (2026-08-06, Brian caught mid-build that the mockup's mic
// icon meant *microphone* mute, not speaker) — there's no broadly-recognized
// HID usage for muting a mic the way CONSUMER_CONTROL_MUTE (0x00E2) already
// reliably mutes the default *output* device (confirmed working on a real
// desktop earlier this project), so retargeting this screen to speaker mute
// makes it immediately, reliably functional over pure HID, same as Volume.
// Real mic mute is left as an open question — see PROGRESS.md.
// Triggered by the knob's physical push-button, globally, mirroring how
// rotate globally triggers Volume — not by a touchscreen tap on this screen
// (the mockup's original "Click: Toggle mute" assumed touch-based per-screen
// navigation reaching this screen first, which no longer applies now that
// it's reached the same way Volume is, via a global hardware input).
constexpr uint32_t SPEAKER_MUTE_ACCENT_COLOR = 0xF0555F; // same red as the mockup's Mic card
// speakerMuted itself is declared up near volumeReturnScreen (showVolumeOverlay() needs it in scope) — see comment there.
// lastSpeakerMuteActivityMs itself is declared up near lastVolumeActivityMs — see comment there.

// Speaker-muted icon (src/img_speaker_muted.c) — the same FA5 "volume-mute"
// cone as LV_SYMBOL_MUTE, but with a diagonal slash baked directly into the
// bitmap (cut into the alpha mask as transparent pixels, not drawn as extra
// opaque ones — same-alpha-as-cone is invisible in a single-color A8 image).
// Replaces an earlier label+lv_line combo: LV_SYMBOL_MUTE alone rendered as
// a bare speaker with no visible X/slash (confirmed on hardware — "shows no
// lines... but should have a slash"), and positioning a separate line
// widget to track the label's own runtime-computed layout box took several
// rounds of guessing and still didn't land right. Baking both into one
// bitmap sidesteps the whole alignment problem — rendered and previewed
// locally (Python/Pillow) before ever touching the device, same as the
// wave-tier icons.
extern const lv_image_dsc_t img_speaker_muted;

// speakerMuteScreen itself is declared up near volumeScreen — see comment there.
lv_obj_t *speakerMuteArc;
lv_obj_t *speakerMuteIcon;     // img_speaker_muted, shown only when muted
lv_obj_t *speakerMuteWaveIcon; // same img_volume_wave1/2/3 as Volume, shown when not muted
lv_obj_t *speakerMuteLabel;
// Pulsing ring shown only while muted (2026-08-07, Brian: "is it possible to
// make the red ring pulse when muted?") — deliberately a plain bordered
// circle (lv_obj), NOT an animated lv_arc. This project has an established,
// hard-won finding that lv_arc redrawn repeatedly for decorative purposes is
// unsafe on this display (see PROGRESS.md's icon-as-bitmap session — two
// separate small-decorative-arc attempts both produced tearing, even with
// LV_DISPLAY_RENDER_MODE_FULL already in place). A pulse is a repeated
// redraw at animation frame rate, so this avoids that risk entirely by only
// ever animating a plain object's border opacity, never touching lv_arc.
// Since a pulsing ring is always a *full* circle (never a partial sweep),
// a plain bordered circle looks identical to speakerMuteArc at value=100
// anyway — no capability is lost by not using lv_arc here.
lv_obj_t *speakerMuteRingPulse;

void speakerMutePulseAnimCb(void *obj, int32_t value) {
  lv_obj_set_style_border_opa((lv_obj_t *)obj, (lv_opa_t)value, 0);
}

void updateSpeakerMuteScreen() {
  // speakerMuteArc itself stays at the dim, near-invisible track look
  // (value 0) always now — speakerMuteRingPulse owns the "muted" visual.
  lv_label_set_text(speakerMuteLabel, speakerMuted ? "MUTED" : "LIVE");
  if (speakerMuted) {
    lv_obj_remove_flag(speakerMuteIcon, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(speakerMuteWaveIcon, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(speakerMuteRingPulse, LV_OBJ_FLAG_HIDDEN);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, speakerMuteRingPulse);
    lv_anim_set_exec_cb(&a, speakerMutePulseAnimCb);
    lv_anim_set_values(&a, LV_OPA_30, LV_OPA_COVER);
    lv_anim_set_time(&a, 900);
    lv_anim_set_playback_time(&a, 900); // fade back down, not a hard cut
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_start(&a);
  } else {
    lv_obj_add_flag(speakerMuteIcon, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(speakerMuteWaveIcon, LV_OBJ_FLAG_HIDDEN);
    lv_anim_delete(speakerMuteRingPulse, speakerMutePulseAnimCb);
    lv_obj_add_flag(speakerMuteRingPulse, LV_OBJ_FLAG_HIDDEN);
    // Same tier count as the Volume screen (Brian: "let the number of
    // speaker arcs correspond with the number of speaker arcs on the volume
    // page") — both read from the same shared `volumePercent`.
    const lv_image_dsc_t *src = volumePercent > 66  ? &img_volume_wave3
                                 : volumePercent > 33 ? &img_volume_wave2
                                                       : &img_volume_wave1;
    lv_image_set_src(speakerMuteWaveIcon, src);
  }
}

void buildSpeakerMuteScreen(lv_obj_t *scr) {
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  // Same ring/caption/icon sizing as Volume for visual consistency between
  // the two globally-triggered overlay screens.
  speakerMuteArc = makeRingGauge(scr, 350, 32, lv_color_hex(SPEAKER_MUTE_ACCENT_COLOR));

  // Plain bordered circle, same outer diameter/thickness as speakerMuteArc
  // above — see its declaration's comment for why this isn't an lv_arc.
  // Hidden until muted; updateSpeakerMuteScreen() drives the pulse.
  speakerMuteRingPulse = lv_obj_create(scr);
  lv_obj_remove_flag(speakerMuteRingPulse, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(speakerMuteRingPulse, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(speakerMuteRingPulse, 350, 350);
  lv_obj_center(speakerMuteRingPulse);
  lv_obj_set_style_radius(speakerMuteRingPulse, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_opa(speakerMuteRingPulse, LV_OPA_TRANSP, 0); // ring only, no fill
  lv_obj_set_style_border_width(speakerMuteRingPulse, 32, 0);
  lv_obj_set_style_border_color(speakerMuteRingPulse, lv_color_hex(SPEAKER_MUTE_ACCENT_COLOR), 0);
  lv_obj_set_style_pad_all(speakerMuteRingPulse, 0, 0);
  lv_obj_add_flag(speakerMuteRingPulse, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *caption = lv_label_create(scr);
  lv_obj_set_style_text_font(caption, &lv_font_montserrat_22, 0);
  lv_obj_set_style_text_color(caption, lv_color_hex(SPEAKER_MUTE_ACCENT_COLOR), 0);
  lv_label_set_text(caption, "SPEAKER");
  lv_obj_align(caption, LV_ALIGN_CENTER, 0, -95);

  speakerMuteIcon = lv_image_create(scr);
  lv_image_set_src(speakerMuteIcon, &img_speaker_muted);
  lv_obj_set_style_image_recolor(speakerMuteIcon, lv_color_hex(SPEAKER_MUTE_ACCENT_COLOR), 0);
  lv_obj_set_style_image_recolor_opa(speakerMuteIcon, LV_OPA_COVER, 0);
  lv_obj_align(speakerMuteIcon, LV_ALIGN_CENTER, 0, -50);

  speakerMuteWaveIcon = lv_image_create(scr);
  lv_image_set_src(speakerMuteWaveIcon, &img_volume_wave3);
  lv_obj_set_style_image_recolor(speakerMuteWaveIcon, lv_color_hex(SPEAKER_MUTE_ACCENT_COLOR), 0);
  lv_obj_set_style_image_recolor_opa(speakerMuteWaveIcon, LV_OPA_COVER, 0);
  lv_obj_align(speakerMuteWaveIcon, LV_ALIGN_CENTER, 0, -50);

  // "MUTED"/"LIVE" is a word, not a number, so it uses the same built-in
  // font as Home's labels rather than the custom digit-only font made for
  // Volume's number.
  speakerMuteLabel = lv_label_create(scr);
  lv_obj_set_style_text_font(speakerMuteLabel, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(speakerMuteLabel, lv_color_white(), 0);
  lv_obj_align(speakerMuteLabel, LV_ALIGN_CENTER, 0, 15);

  updateSpeakerMuteScreen();
}

// Called on every knob button press, from any screen — mirrors
// showVolumeOverlay()'s pattern (remember previous screen, 5s fuse to
// restore it) but for a toggle instead of a continuous value.
void showSpeakerMuteOverlay() {
  lv_obj_t *active = lv_screen_active();
  if (active != speakerMuteScreen) {
    // Volume is always purely transient (see volumeReturnScreen's own
    // comment) — never a real destination, so skip straight past it
    // unconditionally rather than chaining through it on the way back.
    // T40: idleScreen isn't a real destination either — see
    // showVolumeOverlay()'s matching comment.
    speakerMuteReturnScreen = (active == idleScreen) ? homeScreen
        : (active == volumeScreen) ? volumeReturnScreen : active;
    lv_screen_load(speakerMuteScreen);
  }
  speakerMuted = !speakerMuted;
  lastMuteLocalWriteMs = millis(); // T28: don't let a lagging poll response stomp this
  ConsumerControl.press(CONSUMER_CONTROL_MUTE);
  ConsumerControl.release();
  updateSpeakerMuteScreen();
  lastSpeakerMuteActivityMs = millis();
}
