// Volume/mic-mute peripheral: rotating the encoder sends USB HID consumer-control
// volume up/down, pressing the knob sends mute. Requires ARDUINO_USB_MODE=0 (TinyUSB
// composite CDC+HID) — see PROGRESS.md for why mode=1 (used during initial bring-up)
// can't do HID at all.

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Arduino_GFX_Library.h>
#include <TouchDrv.hpp>
#include <lvgl.h>
#include "USB.h"
#include "USBHIDConsumerControl.h"
#include "secrets.h"
#include "pin_config.h"

// Custom-generated font (src/lv_font_montserrat_64_custom.c) — see that
// file's header comment for why. Not one of LVGL's own bundled sizes, so
// no LV_FONT_* macro/lv_conf.h toggle for it; just declare and link.
extern const lv_font_t lv_font_montserrat_64_custom;

constexpr uint32_t DEBOUNCE_MS = 30;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;

USBHIDConsumerControl ConsumerControl;

// Display bring-up only at this stage — proving the panel actually lights up
// and responds correctly before building any real UI on top of it. Pin
// mapping from LilyGo's own GFX.ino example (see pin_config.h for where
// these pins came from, since their repo references a pin_config.h that
// doesn't actually exist upstream). LilyGo's example used Arduino_SH8601,
// but this board is actually the CO5300 variant — confirmed on real
// hardware (2026-08-06), see PROGRESS.md. LilyGo ships both depending on
// batch; don't assume SH8601 if working from their example code elsewhere.
Arduino_DataBus *displayBus = new Arduino_ESP32QSPI(
    SCREEN_CS, SCREEN_SCLK, SCREEN_SDIO0, SCREEN_SDIO1, SCREEN_SDIO2, SCREEN_SDIO3);
// panel = the real hardware driver. setBrightness() is a CO5300 extension,
// not in the base Arduino_GFX interface, so this stays its own concrete
// pointer rather than the base class type.
Arduino_CO5300 *panel = new Arduino_CO5300(
    displayBus, SCREEN_RST, 0 /* rotation */, false /* IPS */, SCREEN_WIDTH, SCREEN_HEIGHT);

bool displayOk = false;

// Touch bring-up only at this stage — same "prove it works before building
// on it" approach as the encoder and display. LilyGo's README says this
// board ships with either CHSC5816 or CST816 touch IC depending on batch;
// CHSC5816 failed init on this actual board (confirmed via the periodic
// status log, 2026-08-06) — consistent with the display turning out to be
// CO5300 rather than SH8601, so trying CST816 next. Declared before the LVGL
// section below since its input-device read callback needs `touch` in scope.
TouchDrvCST816 touch;
bool touchOk = false;

void setupTouch() {
  touch.setPins(TOUCH_RST, TOUCH_INT);
  touchOk = touch.begin(Wire, CST816_SLAVE_ADDRESS, TOUCH_SDA, TOUCH_SCL);
  if (!touchOk) {
    Serial.println("Touch init FAILED (CST816)");
    return;
  }
  Serial.println("Touch init OK");
  Serial.printf("Touch model: %s, chip ID: 0x%02X\n", touch.getModelName(), touch.getChipID());
}

// Placeholder values until the companion app protocol exists (see
// PROGRESS.md's "Next" list) — proving the LVGL screen rendering works on
// real hardware first, same as every other piece of this firmware so far.
// CPU/GPU are temperatures (°C), not percentages — matches the original
// mockup, which shows "CPU 42°"/"GPU 54°" but "RAM 61%". Named accordingly
// (fixed 2026-08-06 after the Home screen mislabeled all three as %).
float cpuTempC = 42;
float gpuTempC = 54;
float ramPercent = 61;

// LVGL owns its own draw buffer and rendering entirely — this replaces the
// earlier direct-to-Arduino_GFX drawHomeScreen()/drawRingGauge() approach.
// Two full-frame PSRAM buffers (LV_DISPLAY_RENDER_MODE_FULL), not a small
// partial buffer. Tried a 40-row partial buffer first (matching Arduino_GFX's
// own LVGL example) — rapid encoder turns produced visible tearing on both
// the arc and the label, since a large redraw (the whole arc sweep changing)
// got split into several separate stripe writes sent to the panel one at a
// time, each individually slow enough over QSPI to catch mid-write.
// LV_DISPLAY_RENDER_MODE_FULL always redraws + flushes the entire display as
// one contiguous blit per frame — same "one big write instead of many small
// ones" shape that fixed the original fillArc/fillCircle corruption (see
// PROGRESS.md "Display rendering bug"), and eliminates the tearing since
// there's no longer a multi-write window to catch mid-update. Two buffers
// (not one) so LVGL can render the next frame while the previous one's
// flush is still in flight. 390×390×2 bytes ≈ 297KB each — fits internal
// RAM alone in a pinch, but PSRAM (confirmed working, 8MB available) is used
// so this doesn't compete with everything else's RAM budget.
lv_display_t *lvDisplay = nullptr;
lv_color_t *lvDrawBuf = nullptr;
lv_color_t *lvDrawBuf2 = nullptr;

lv_obj_t *cpuArc, *gpuArc, *ramArc;
lv_obj_t *cpuLabel, *gpuLabel, *ramLabel;

uint32_t lvMillisCb() { return millis(); }

void lvLogPrint(lv_log_level_t level, const char *buf) {
  LV_UNUSED(level);
  Serial.print(buf);
}

void lvDispFlush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  uint32_t w = lv_area_get_width(area);
  uint32_t h = lv_area_get_height(area);
  panel->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
  lv_disp_flush_ready(disp);
}

void lvTouchpadRead(lv_indev_t *indev, lv_indev_data_t *data) {
  // Deliberately does NOT gate on touch.isPressed(): with an IRQ pin
  // configured (ours is, see setupTouch()), that call does a raw
  // digitalRead() of the interrupt line rather than real touch state (see
  // TouchDrvInterface.hpp's isPressed() doc comment: "suitable for screens
  // with an interrupt pin... For devices without an interrupt pin, simply
  // call getTouchPoints"). The CST816's INT line pulses briefly per touch
  // *report*, it isn't held low for the whole drag, so most ~30ms polls
  // caught it HIGH mid-swipe (looking "released") even with a finger still
  // down — this was the root cause of swipe-up gestures being unreliable
  // (2026-08-06, diagnosed via temporary on-screen instrumentation, since
  // removed — see PROGRESS.md). getTouchPoints() does a genuine fresh I2C
  // register read every call, independent of the interrupt pin, so it alone
  // is enough to detect press state reliably.
  if (touchOk) {
    TouchPoints points = touch.getTouchPoints();
    if (points.hasPoints()) {
      const TouchPoint &p = points.getPoint(0);
      data->state = LV_INDEV_STATE_PRESSED;
      data->point.x = p.x;
      data->point.y = p.y;
      return;
    }
  }
  data->state = LV_INDEV_STATE_RELEASED;
}

// One ring = a dim full-circle "track" plus a colored value arc, swept
// clockwise starting from 12 o'clock — matches the watch-face mockup from
// the original UI brainstorm. LVGL's arc angle convention is 0°=3 o'clock
// like Arduino_GFX's was, so rotation=270 (i.e. -90) shifts the whole arc's
// coordinate frame so 0 visually lands at 12 o'clock; LV_ARC_MODE_NORMAL
// (the default) then grows the value indicator clockwise from there.
lv_obj_t *makeRingGauge(lv_obj_t *parent, int16_t diameter, int16_t thickness, lv_color_t color) {
  lv_obj_t *arc = lv_arc_create(parent);
  lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
  lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(arc, diameter, diameter);
  lv_obj_center(arc);
  lv_arc_set_bg_angles(arc, 0, 360);
  lv_arc_set_rotation(arc, 270);
  lv_arc_set_range(arc, 0, 100);
  // lv_arc_set_range() above is a no-op when the range already matches
  // lv_arc_create()'s own default (0-100, which it does) — it early-returns
  // before ever clamping the arc's internal value out of its "unset"
  // sentinel state. A freshly-created arc's indicator angles default to a
  // fixed 135°-270° sweep (LVGL's own generic-gauge look) and stay stuck
  // there — showing a visible partial arc — until something actually calls
  // lv_arc_set_value(). Every screen except Quick-jump happens to call that
  // via its own update*Screen() right after building, which is what masked
  // this; Quick-jump's ring never got a value at all and visibly showed the
  // leftover default arc (Brian: "partially filled light blue arc... what
  // is that for?"). Setting it explicitly here means every ring this helper
  // builds starts in a real "empty" state regardless of what its caller
  // does afterward.
  lv_arc_set_value(arc, 0);
  // True near-black, not a plain grey — the track should blend into the
  // true-black background per the original watch-face design (AMOLED
  // burn-in mitigation), just barely visible as a guide ring. Confirmed on
  // hardware with the equivalent color in the pre-LVGL version.
  lv_obj_set_style_arc_color(arc, lv_color_hex(0x141414), LV_PART_MAIN);
  lv_obj_set_style_arc_width(arc, thickness, LV_PART_MAIN);
  lv_obj_set_style_arc_rounded(arc, false, LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc, color, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc, thickness, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(arc, false, LV_PART_INDICATOR);
  return arc;
}

lv_obj_t *makeRingLabel(lv_obj_t *parent, int16_t yOffset, lv_color_t color) {
  lv_obj_t *label = lv_label_create(parent);
  lv_obj_set_style_text_font(label, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(label, color, 0);
  lv_obj_align(label, LV_ALIGN_CENTER, 0, yOffset);
  return label;
}

void updateHomeScreen() {
  // The arcs still take a 0-100 value regardless of unit — temperature and
  // percentage both happen to live in a sane 0-100 range for a ring gauge,
  // same as the mockup implicitly assumes (pct=42 drawn for "42°").
  lv_arc_set_value(cpuArc, (int)cpuTempC);
  lv_arc_set_value(gpuArc, (int)gpuTempC);
  lv_arc_set_value(ramArc, (int)ramPercent);
  lv_label_set_text_fmt(cpuLabel, "CPU %d°", (int)cpuTempC);
  lv_label_set_text_fmt(gpuLabel, "GPU %d°", (int)gpuTempC);
  lv_label_set_text_fmt(ramLabel, "RAM %d%%", (int)ramPercent);
}

void buildHomeScreen(lv_obj_t *scr) {
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE); // no scrollable content; keeps swipe-up gesture detection reliable

  // Diameters/thickness match the outer/inner radius pairs from the
  // pre-LVGL version (185/165, 160/140, 135/115) — same visual layout.
  cpuArc = makeRingGauge(scr, 370, 20, lv_color_hex(0xFF8C00));
  gpuArc = makeRingGauge(scr, 320, 20, lv_color_hex(0x0080FF));
  ramArc = makeRingGauge(scr, 270, 20, lv_color_hex(0x00C853));

  cpuLabel = makeRingLabel(scr, -44, lv_color_hex(0xFF8C00));
  gpuLabel = makeRingLabel(scr, 0, lv_color_hex(0x0080FF));
  ramLabel = makeRingLabel(scr, 44, lv_color_hex(0x00C853));

  updateHomeScreen();
}

// Volume screen: the only screen reachable by rotating the knob, from any
// other screen, TV-OSD style (see PROGRESS.md's design decisions — rotate is
// claimed globally, not per-screen). No companion app yet (see PROGRESS.md's
// "Next" list), so there's no real OS volume level to read back — this is a
// **local estimate only**, seeded at 50 and nudged by VOLUME_STEP per detent.
// It will drift from the PC's actual volume over time since the HID
// increment/decrement pulses we send are fire-and-forget with no readback.
// Good enough until the companion app can report the real value over serial.
constexpr int VOLUME_STEP = 2;
// Shared by Volume and Speaker Mute — both are transient, globally-triggered
// overlays (rotate / knob-press) that restore whatever they interrupted
// after this long. Renamed from VOLUME_RETURN_MS (2026-08-06) once Speaker
// Mute started using the same fuse.
constexpr uint32_t OVERLAY_RETURN_MS = 5000;
int volumePercent = 50;
uint32_t lastVolumeActivityMs = 0;

// Volume level icons (src/img_volume_waves.c) — real FontAwesome 5 pixels,
// same style as LV_SYMBOL_VOLUME_MID/MAX (confirmed perfect on the Speaker
// Mute screen, since those render as pre-baked bitmap glyphs rather than a
// live lv_arc widget — the actual root cause of the tearing on every earlier
// attempt here was lv_arc specifically, not curved shapes in general). Only
// wave2 (the 2-wave middle tier) needed synthesizing — FA5 doesn't ship one —
// done by pixel-diffing the 1-wave and 3-wave glyphs to isolate just the
// middle wave arc and compositing it onto the 1-wave glyph, so it's real
// font pixels throughout, not a hand-drawn approximation. Two earlier
// attempts at this (small lv_arc wave marks, then a rectangle-built
// speaker+bars composite) are superseded by this — see PROGRESS.md for the
// full story of how this converged.
extern const lv_image_dsc_t img_volume_wave1; // 1 wave (low)
extern const lv_image_dsc_t img_volume_wave2; // 2 waves (medium)
extern const lv_image_dsc_t img_volume_wave3; // 3 waves (high)

lv_obj_t *homeScreen;
lv_obj_t *volumeScreen;
lv_obj_t *volumeArc;
lv_obj_t *volumeMuteIcon; // LV_SYMBOL_MUTE label, shown only at 0%
lv_obj_t *volumeWaveIcon; // single image widget, src swapped per tier
lv_obj_t *volumeLabel;
lv_obj_t *previousScreen; // whatever was showing before Volume/Speaker Mute interrupted it

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
    previousScreen = active;
    lv_screen_load(volumeScreen);
  }
  volumePercent = constrain(volumePercent + delta, 0, 100);
  updateVolumeScreen();
  lastVolumeActivityMs = millis();
}

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
bool speakerMuted = false; // local estimate only — HID mute is a toggle with no readback
uint32_t lastSpeakerMuteActivityMs = 0;

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

lv_obj_t *speakerMuteScreen;
lv_obj_t *speakerMuteArc;
lv_obj_t *speakerMuteIcon;     // img_speaker_muted, shown only when muted
lv_obj_t *speakerMuteWaveIcon; // same img_volume_wave1/2/3 as Volume, shown when not muted
lv_obj_t *speakerMuteLabel;

void updateSpeakerMuteScreen() {
  lv_arc_set_value(speakerMuteArc, speakerMuted ? 100 : 0);
  lv_label_set_text(speakerMuteLabel, speakerMuted ? "MUTED" : "LIVE");
  if (speakerMuted) {
    lv_obj_remove_flag(speakerMuteIcon, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(speakerMuteWaveIcon, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(speakerMuteIcon, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(speakerMuteWaveIcon, LV_OBJ_FLAG_HIDDEN);
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
    previousScreen = active;
    lv_screen_load(speakerMuteScreen);
  }
  speakerMuted = !speakerMuted;
  ConsumerControl.press(CONSUMER_CONTROL_MUTE);
  ConsumerControl.release();
  updateSpeakerMuteScreen();
  lastSpeakerMuteActivityMs = millis();
}

// Quick-jump grid: the only way to jump straight to a screen out of the
// normal swipe/rotate order (see PROGRESS.md's design decisions — "Swipe up
// = quick-jump grid, the only way to open it"). Layout matches the original
// mockup's compass positions: Home at top, Speaker Mute at bottom-left, Now
// Playing at bottom-right — Now Playing's spot is dimmed/inert since that
// screen doesn't exist yet (see PROGRESS.md's "Next" list), kept in place so
// the layout doesn't need to shift once it's built. No Volume spot, by
// design — Volume stays rotate-only and excluded from both swipe nav and
// this grid, same reasoning as the mockup's "No Volume icon, rotate-only."
constexpr uint32_t QUICK_JUMP_RING_COLOR = 0x2EC4B6;        // teal, matches the mockup's Quick-jump card
constexpr uint32_t QUICK_JUMP_HOME_COLOR = 0xE0A940;        // same gold as Home's CPU ring
constexpr uint32_t QUICK_JUMP_NOW_PLAYING_COLOR = 0x4FB477; // same green as the mockup's Now Playing card
// Longer than OVERLAY_RETURN_MS (5s) — this is a browsed menu you look at
// and choose from, not a transient nudge like Volume/Speaker Mute.
constexpr uint32_t QUICK_JUMP_RETURN_MS = 8000;

lv_obj_t *quickJumpScreen;
uint32_t lastQuickJumpActivityMs = 0;

// One circular "spot": dark filled circle, colored border, colored icon
// centered inside — mirrors the mockup's `.menu-spot` styling. Pass either
// an LV_SYMBOL_* glyph (bitmap-based font icon, e.g. LV_SYMBOL_HOME) or a
// pre-rendered lv_image_dsc_t (for the Speaker Mute spot, which reuses the
// existing slashed-speaker bitmap) — not both.
lv_obj_t *makeQuickJumpSpot(lv_obj_t *parent, int16_t xOffset, int16_t yOffset,
                             lv_color_t color, const char *iconSymbol,
                             const lv_image_dsc_t *imgSrc) {
  lv_obj_t *spot = lv_obj_create(parent);
  lv_obj_remove_flag(spot, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(spot, 76, 76);
  lv_obj_set_style_radius(spot, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(spot, lv_color_hex(0x141414), 0);
  lv_obj_set_style_bg_opa(spot, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(spot, 2, 0);
  lv_obj_set_style_border_color(spot, color, 0);
  lv_obj_set_style_pad_all(spot, 0, 0);
  lv_obj_align(spot, LV_ALIGN_CENTER, xOffset, yOffset);

  if (iconSymbol) {
    lv_obj_t *icon = lv_label_create(spot);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(icon, color, 0);
    lv_label_set_text(icon, iconSymbol);
    lv_obj_center(icon);
  } else if (imgSrc) {
    lv_obj_t *icon = lv_image_create(spot);
    lv_image_set_src(icon, imgSrc);
    lv_obj_set_style_image_recolor(icon, color, 0);
    lv_obj_set_style_image_recolor_opa(icon, LV_OPA_COVER, 0);
    lv_obj_center(icon);
  }
  return spot;
}

void quickJumpSpotClickedCb(lv_event_t *e) {
  lv_screen_load((lv_obj_t *)lv_event_get_user_data(e));
}

void buildQuickJumpScreen(lv_obj_t *scr) {
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  // Plain dim ring, no accent segment — matches the mockup's `--pct:0` dial;
  // this screen has no single value to show, it's a menu.
  makeRingGauge(scr, 350, 32, lv_color_hex(QUICK_JUMP_RING_COLOR));

  lv_obj_t *centerLabel = lv_label_create(scr);
  lv_obj_set_style_text_font(centerLabel, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(centerLabel, lv_color_hex(0x9AA5A3), 0);
  lv_obj_set_style_text_letter_space(centerLabel, 2, 0);
  lv_label_set_text(centerLabel, "TAP A SCREEN");
  lv_obj_center(centerLabel);

  lv_obj_t *homeSpot =
      makeQuickJumpSpot(scr, 0, -120, lv_color_hex(QUICK_JUMP_HOME_COLOR), LV_SYMBOL_HOME, nullptr);
  lv_obj_add_event_cb(homeSpot, quickJumpSpotClickedCb, LV_EVENT_CLICKED, homeScreen);

  // Speaker Mute's spot (mockup's bottom-left slot) was removed (2026-08-07)
  // — Brian found it wasn't useful in practice: the physical knob button
  // already jumps there from any screen, and while actually muted that
  // screen doesn't time out back to Home anyway (see loop()'s fuse check),
  // so there was nothing left for this spot to be a *shortcut* to. It also
  // turned out to be visibly broken — its icon was hardcoded to the slashed
  // bitmap regardless of live speakerMuted state, always showing "muted"
  // even when live. Not worth fixing for a spot that wasn't earning its
  // place anyway.
  // Now Playing doesn't exist yet — dimmed and non-interactive, but kept
  // in place so it's ready to wire up once that screen is built (see
  // PROGRESS.md's "Next" list). Moved to bottom-center (was bottom-right,
  // mirroring the mockup's 3-spot layout) now that Speaker Mute's bottom-
  // left spot is gone — centering it under Home reads better with only two
  // spots than leaving it off to one side with an empty gap opposite it.
  lv_obj_t *nowPlayingSpot =
      makeQuickJumpSpot(scr, 0, 100, lv_color_hex(QUICK_JUMP_NOW_PLAYING_COLOR), LV_SYMBOL_PLAY, nullptr);
  lv_obj_remove_flag(nowPlayingSpot, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_opa(nowPlayingSpot, LV_OPA_40, 0);
}

// Called on a detected swipe-up gesture, from Home, Volume, or Speaker Mute
// (see quickJumpGestureCb below) — mirrors showVolumeOverlay()'s
// remember-previous-screen pattern. Swiping up from Volume was originally
// excluded by design (rotate-only), but Brian asked for it directly
// (2026-08-06: "It doesn't work from the volume page, though. It should.").
// One accepted edge case from allowing this: if quick-jump's own idle fuse
// later restores previousScreen == volumeScreen, Volume's own fuse-check
// (loop()) will immediately re-fire on its own stale timer — but by then
// previousScreen has already been overwritten to volumeScreen too, so it
// just reloads the same screen rather than looping. Same shared-global
// tradeoff already accepted between Volume and Speaker Mute elsewhere in
// this file.
void showQuickJumpMenu() {
  lv_obj_t *active = lv_screen_active();
  if (active == quickJumpScreen) return;
  previousScreen = active;
  lv_screen_load(quickJumpScreen);
  lastQuickJumpActivityMs = millis();
}

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
  if (dir == LV_DIR_TOP && (active == homeScreen || active == volumeScreen || active == speakerMuteScreen)) {
    showQuickJumpMenu();
  }
}

void setupDisplay() {
  pinMode(SCREEN_EN, OUTPUT);
  digitalWrite(SCREEN_EN, HIGH);

  displayOk = panel->begin();
  if (!displayOk) {
    Serial.println("Display init FAILED");
    return;
  }
  Serial.println("Display init OK");

  panel->setBrightness(200);
  panel->fillScreen(BLACK); // clear before LVGL takes over, avoid boot-time garbage
}

void setupLVGL() {
  if (!displayOk) return;

  lv_init();
  lv_tick_set_cb(lvMillisCb);
#if LV_USE_LOG
  lv_log_register_print_cb(lvLogPrint);
#endif

  uint32_t bufPixels = (uint32_t)SCREEN_WIDTH * SCREEN_HEIGHT;
  size_t bufBytes = bufPixels * sizeof(lv_color_t);
  lvDrawBuf = (lv_color_t *)heap_caps_malloc(bufBytes, MALLOC_CAP_SPIRAM);
  lvDrawBuf2 = (lv_color_t *)heap_caps_malloc(bufBytes, MALLOC_CAP_SPIRAM);
  if (!lvDrawBuf || !lvDrawBuf2) {
    // PSRAM alloc failed for some reason — fall back to a single internal-RAM
    // buffer rather than fail outright, though tearing risk returns without
    // the second buffer.
    if (!lvDrawBuf) lvDrawBuf = (lv_color_t *)heap_caps_malloc(bufBytes, MALLOC_CAP_8BIT);
    lvDrawBuf2 = nullptr;
  }
  if (!lvDrawBuf) {
    Serial.println("LVGL draw buffer allocation FAILED");
    return;
  }

  lvDisplay = lv_display_create(SCREEN_WIDTH, SCREEN_HEIGHT);
  lv_display_set_flush_cb(lvDisplay, lvDispFlush);
  lv_display_set_buffers(lvDisplay, lvDrawBuf, lvDrawBuf2, bufBytes, LV_DISPLAY_RENDER_MODE_FULL);

  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, lvTouchpadRead);
  // Default gesture distance is 50px (LV_INDEV_DEF_GESTURE_LIMIT). On a
  // round display the touchable chord narrows fast near the top/bottom
  // edge — exactly where a swipe-up naturally starts/ends — so it's easy
  // for a real swipe to drift off the vertical axis before reaching 50px,
  // which resolves the gesture to LEFT/RIGHT instead of TOP (confirmed:
  // Brian saw both "TOP" and "LEFT" from swipe-up attempts via the debug
  // readout, very inconsistently). Shorter threshold gives less distance
  // for a swipe to wander off-axis.
  lv_indev_set_gesture_min_distance(indev, 30);

  homeScreen = lv_obj_create(NULL);
  buildHomeScreen(homeScreen);
  volumeScreen = lv_obj_create(NULL);
  buildVolumeScreen(volumeScreen);
  speakerMuteScreen = lv_obj_create(NULL);
  buildSpeakerMuteScreen(speakerMuteScreen);
  quickJumpScreen = lv_obj_create(NULL);
  buildQuickJumpScreen(quickJumpScreen);

  // Swipe-up entry point for the quick-jump grid. Subscribed on the indev
  // itself, not a screen object — see quickJumpGestureCb's comment for why
  // (LVGL sends LV_EVENT_GESTURE to the indev too, not just whichever
  // widget the bubble-up walk lands on). Screen filtering (Home/Speaker
  // Mute only, not Volume) happens inside the callback instead.
  lv_indev_add_event_cb(indev, quickJumpGestureCb, LV_EVENT_GESTURE, NULL);
  makeGestureBubbleThrough(homeScreen); // still correct practice even though no longer load-bearing for this
  makeGestureBubbleThrough(volumeScreen);
  makeGestureBubbleThrough(speakerMuteScreen);

  previousScreen = homeScreen;
  lv_screen_load(homeScreen);

  Serial.println("LVGL init OK");
}

void setupOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("t-encoder-pro"); // DHCP/router-visible name — separate from
                                      // ArduinoOTA's mDNS hostname below, both needed
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t start = millis();
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    // No WiFi, no OTA — HID over USB still works fine without it.
    Serial.println("\nWiFi connect failed, continuing without OTA");
    return;
  }

  Serial.printf("\nWiFi connected: %s\n", WiFi.localIP().toString().c_str());

  ArduinoOTA.setHostname("t-encoder-pro");
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() { Serial.println("OTA: start"); });
  ArduinoOTA.onEnd([]() { Serial.println("OTA: done"); });
  ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA error: %u\n", error); });
  ArduinoOTA.begin();
  Serial.println("OTA ready");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_KEY, INPUT_PULLUP);

  ConsumerControl.begin();
  USB.begin();

  setupDisplay();
  setupTouch();
  setupLVGL();
  setupOTA();

  Serial.println("T-Encoder Pro: HID volume/mute ready");
}

void loop() {
  static int lastA = HIGH;
  static int lastKey = HIGH;
  static uint32_t lastStepMs = 0;
  static uint32_t lastKeyMs = 0;
  static uint32_t lastStatusMs = 0;

  ArduinoOTA.handle();

  // LVGL now owns touch reads (via lvTouchpadRead(), called through this) —
  // no separate manual touch-print block needed anymore.
  lv_timer_handler();

  // Periodic, not just at-boot: one-time boot messages are easy to miss
  // when reading serial after the fact (the display/touch init messages
  // above fire within milliseconds of USB enumerating, faster than a
  // freshly-attached listener can reliably catch — confirmed by testing,
  // not assumption). This makes status visible no matter when you happen
  // to be listening.
  if (millis() - lastStatusMs > 10000) {
    lastStatusMs = millis();
    Serial.printf("status: wifi=%s display=%s touch=%s ip=%s\n",
                   WiFi.status() == WL_CONNECTED ? "up" : "down",
                   displayOk ? "ok" : "FAIL",
                   touchOk ? "ok" : "FAIL",
                   WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "-");
  }

  int a = digitalRead(PIN_ENCODER_A);
  if (a != lastA) {
    lastA = a;
    uint32_t now = millis();
    // Direction from B's state on A's falling edge. If this reads backwards
    // on the real hardware, just swap the two branches below.
    if (a == LOW && (now - lastStepMs) > DEBOUNCE_MS) {
      lastStepMs = now;
      if (digitalRead(PIN_ENCODER_B) == HIGH) {
        ConsumerControl.press(CONSUMER_CONTROL_VOLUME_INCREMENT);
        ConsumerControl.release();
        showVolumeOverlay(VOLUME_STEP);
        Serial.println("VOL_UP");
      } else {
        ConsumerControl.press(CONSUMER_CONTROL_VOLUME_DECREMENT);
        ConsumerControl.release();
        showVolumeOverlay(-VOLUME_STEP);
        Serial.println("VOL_DOWN");
      }
    }
  }

  // 5s-after-last-turn fuse: restore whatever screen Volume interrupted.
  if (lv_screen_active() == volumeScreen && (millis() - lastVolumeActivityMs) > OVERLAY_RETURN_MS) {
    lv_obj_t *restoring = previousScreen;
    lv_screen_load(restoring);
    if (restoring == quickJumpScreen) lastQuickJumpActivityMs = millis();
  }
  // Same fuse for Speaker Mute, but only while *not* muted (Brian: "when
  // muted, it should not [return] to the main screen... only return after
  // unmuting") — staying muted is a deliberate, persistent state worth
  // keeping visible indefinitely, unlike Volume's transient adjustments.
  // The fuse only starts counting once you unmute (lastSpeakerMuteActivityMs
  // resets on every button press, so unmuting gets its own fresh 5s window).
  if (lv_screen_active() == speakerMuteScreen && !speakerMuted &&
      (millis() - lastSpeakerMuteActivityMs) > OVERLAY_RETURN_MS) {
    lv_obj_t *restoring = previousScreen;
    lv_screen_load(restoring);
    if (restoring == quickJumpScreen) lastQuickJumpActivityMs = millis();
  }
  // Quick-jump's own, longer idle fuse: walked away without tapping
  // anything, restore whatever screen it was opened from.
  if (lv_screen_active() == quickJumpScreen && (millis() - lastQuickJumpActivityMs) > QUICK_JUMP_RETURN_MS) {
    lv_screen_load(previousScreen);
  }

  int key = digitalRead(PIN_ENCODER_KEY);
  if (key != lastKey) {
    lastKey = key;
    uint32_t now = millis();
    if (key == LOW && (now - lastKeyMs) > DEBOUNCE_MS) {
      lastKeyMs = now;
      showSpeakerMuteOverlay(); // sends the HID mute pulse itself
      Serial.println("MUTE");
    }
  }

  delay(5); // matches Arduino_GFX's own LVGL example's loop cadence
}
