// screen_volume.cpp — Volume overlay screen.

#include <Arduino.h>
#include <lvgl.h>
#include "screen_nav.h"
#include "screen_home.h"
#include "screen_idle.h"
#include "screen_volume.h"

// Custom-generated font (src/lv_font_montserrat_64_custom.c) — see that
// file's header comment for why. Not one of LVGL's own bundled sizes, so
// no LV_FONT_* macro/lv_conf.h toggle for it; just declare and link.
extern const lv_font_t lv_font_montserrat_64_custom;

uint32_t volumeFuseMs = VOLUME_FUSE_MS_DEFAULT;
int volumePercent = 50;
// T28 (2026-08-21): when this local optimistic guess was last changed by knob
// rotation — distinct from lastVolumeActivityMs below (which also bumps on
// just re-entering the screen, an unrelated UI-fuse concern). See this
// section's use in loop()'s sync-apply step for why it exists.
uint32_t lastVolumeLocalWriteMs = 0;
uint32_t lastVolumeActivityMs = 0;
// Declared together with lastVolumeActivityMs (rather than down where each
// is actually used) since restoreScreen() below needs all three up front.
uint32_t lastSpeakerMuteActivityMs = 0;
uint32_t lastQuickJumpActivityMs = 0;

lv_obj_t *volumeArc;
lv_obj_t *volumeMuteIcon; // LV_SYMBOL_MUTE label, shown only at 0%
lv_obj_t *volumeWaveIcon; // single image widget, src swapped per tier
lv_obj_t *volumeLabel;

// Same blue used for the ring, the "VOLUME" caption, and the icon — matches
// the original UI mockup (https://claude.ai/code/artifact/659dd01a-d6ab-42d9-
// 8998-5f99360b04ad), where the mode's accent color carries across all three,
// not just the ring. Re-checked directly against that mockup (2026-08-06)
// after the first LVGL pass drifted from it — see PROGRESS.md.
constexpr uint32_t VOLUME_ACCENT_COLOR = 0x0080FF;

void updateVolumeScreen() {
  lv_arc_set_value(volumeArc, volumePercent);
  lv_label_set_text_fmt(volumeLabel, "%d", volumePercent); // no "%" — matches mockup's bare number

  // Show/hide + src swap only — no lv_obj_align() calls here, position is
  // fixed at build time (see PROGRESS.md: realigning on every update was the
  // suspected cause of the very first icon attempt's tearing).
  if (volumePercent == 0) {
    lv_obj_remove_flag(volumeMuteIcon, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(volumeWaveIcon, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(volumeMuteIcon, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(volumeWaveIcon, LV_OBJ_FLAG_HIDDEN);
    const lv_image_dsc_t *src = volumePercent > 66  ? &img_volume_wave3
                                 : volumePercent > 33 ? &img_volume_wave2
                                                       : &img_volume_wave1;
    lv_image_set_src(volumeWaveIcon, src);
  }
}

void buildVolumeScreen(lv_obj_t *scr) {
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  // Ring: 350 diameter, 32 thickness (inner radius 143). Text sizes went
  // through several rounds of "a little bigger" (2026-08-06): captions and
  // device label ended at 22pt, icon at 42pt. The number moved off LVGL's
  // built-in fonts entirely (48pt was the largest bundled size) onto a
  // custom-generated 64pt one — see the extern decl near the top of this
  // file and src/lv_font_montserrat_64_custom.c.
  volumeArc = makeRingGauge(scr, 350, 32, lv_color_hex(VOLUME_ACCENT_COLOR));

  // "VOLUME" sits alone near the top of the ring, not part of the centered
  // icon/number/device cluster below — matches the mockup's separate
  // `.mode-name-onscreen` element. Placed well inside the ring's inner edge
  // (radius 143) — an earlier attempt further out put it right on top of
  // the ring stroke itself, burying the text under it. Nudged up slightly
  // (-90→-95) to keep clearance from the now-bigger icon below it.
  lv_obj_t *caption = lv_label_create(scr);
  lv_obj_set_style_text_font(caption, &lv_font_montserrat_22, 0);
  lv_obj_set_style_text_color(caption, lv_color_hex(VOLUME_ACCENT_COLOR), 0);
  lv_label_set_text(caption, "VOLUME");
  lv_obj_align(caption, LV_ALIGN_CENTER, 0, -95);

  // Icon, number, and device name form one tight centered cluster (matches
  // the mockup's `.dial-content` flex column) — icon/waves take the accent
  // color like the caption, number is the only white element, device name is
  // small/muted right below it. Both icon states are built once at a fixed
  // position; updateVolumeScreen() only ever toggles which is hidden.
  volumeMuteIcon = lv_label_create(scr);
  lv_obj_set_style_text_font(volumeMuteIcon, &lv_font_montserrat_42, 0);
  lv_obj_set_style_text_color(volumeMuteIcon, lv_color_hex(VOLUME_ACCENT_COLOR), 0);
  lv_label_set_text(volumeMuteIcon, LV_SYMBOL_MUTE);
  lv_obj_align(volumeMuteIcon, LV_ALIGN_CENTER, 0, -50);

  // Single image widget, src swapped per tier in updateVolumeScreen() —
  // matches the size/style of Speaker Mute's LV_SYMBOL_VOLUME_MAX icon
  // (Brian: "make it match the size and shape of the one on the mute
  // screen"), since these are literally built from the same source font.
  volumeWaveIcon = lv_image_create(scr);
  lv_image_set_src(volumeWaveIcon, &img_volume_wave3);
  lv_obj_set_style_image_recolor(volumeWaveIcon, lv_color_hex(VOLUME_ACCENT_COLOR), 0);
  lv_obj_set_style_image_recolor_opa(volumeWaveIcon, LV_OPA_COVER, 0);
  lv_obj_align(volumeWaveIcon, LV_ALIGN_CENTER, 0, -50);

  volumeLabel = lv_label_create(scr);
  lv_obj_set_style_text_font(volumeLabel, &lv_font_montserrat_64_custom, 0);
  lv_obj_set_style_text_color(volumeLabel, lv_color_white(), 0);
  lv_obj_align(volumeLabel, LV_ALIGN_CENTER, 0, 5);

  // Output device name: no companion app yet (see PROGRESS.md's "Next"
  // list), so there's no real way to know the PC's actual default audio
  // device from firmware alone — placeholder until that protocol exists,
  // same pattern as Home's placeholder CPU/GPU/RAM values. Muted grey,
  // uppercase, matches the mockup's sub-value styling exactly. Nudged down
  // slightly (+48→+55) for clearance from the number now that its caption
  // font is bigger too.
  lv_obj_t *deviceLabel = lv_label_create(scr);
  lv_obj_set_style_text_font(deviceLabel, &lv_font_montserrat_22, 0);
  lv_obj_set_style_text_color(deviceLabel, lv_color_hex(0x9AA5A3), 0);
  lv_label_set_text(deviceLabel, "SPEAKERS");
  lv_obj_align(deviceLabel, LV_ALIGN_CENTER, 0, 55);

  updateVolumeScreen();
}

// Called on every encoder detent, from any screen. Switches to the Volume
// screen if not already there (remembering what was showing so the 5s fuse
// in loop() can restore it), nudges the local volume estimate, and resets
// the fuse's timer.
void showVolumeOverlay(int delta) {
  lv_obj_t *active = lv_screen_active();
  if (active != volumeScreen) {
    // Skip straight past Mute if it's not actively muted right now — an
    // unmuted Mute screen is just mid-flight back to whatever it covers
    // anyway, not a real destination (see volumeReturnScreen's own comment).
    // While actively muted, Mute stays the legitimate target so its
    // indicator survives a volume bump instead of getting bounced behind it.
    // T40: idleScreen is never a real destination either (same reasoning as
    // Volume itself never being one) — rotating the knob is itself a real
    // interaction, so lastAnyInteractionMs already resets and the idle
    // screen would just get swapped back to Home again next loop() pass
    // regardless; redirecting the return target here avoids a one-frame
    // bounce through it first.
    volumeReturnScreen = (active == idleScreen) ? homeScreen
        : (active == speakerMuteScreen && !speakerMuted) ? speakerMuteReturnScreen : active;
    lv_screen_load(volumeScreen);
  }
  volumePercent = constrain(volumePercent + delta, 0, 100);
  lastVolumeLocalWriteMs = millis(); // T28: don't let a lagging poll response stomp this
  updateVolumeScreen();
  lastVolumeActivityMs = millis();
}
