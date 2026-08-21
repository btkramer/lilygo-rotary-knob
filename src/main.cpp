// Volume/mic-mute peripheral: rotating the encoder sends USB HID consumer-control
// volume up/down, pressing the knob sends mute. Requires ARDUINO_USB_MODE=0 (TinyUSB
// composite CDC+HID) — see PROGRESS.md for why mode=1 (used during initial bring-up)
// can't do HID at all.

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <Arduino_GFX_Library.h>
#include <TouchDrv.hpp>
#include <lvgl.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
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

// Encoder reading moved from loop()-polling to a hardware interrupt
// (2026-08-07) — polling missed/misread transitions on fast spins (Brian:
// "sometimes skip a number or two or go in the wrong direction... usually
// only when I turn it quickly"). loop() does real, variably-slow work every
// iteration (LVGL's full-390x390-panel redraw+flush, ArduinoOTA.handle(),
// WebSerial.loop()), so digitalRead(PIN_ENCODER_A) wasn't being sampled
// often/regularly enough to catch every real quadrature transition during a
// fast rotation — a transition landing between two loop() iterations is
// either missed entirely (skipped step) or, worse, read alongside a B value
// that's already moved on to the *next* transition (wrong direction), since
// this decode assumes A and B are sampled close enough together to reflect
// one single real transition. An interrupt captures each transition the
// instant it happens, independent of whatever loop() is currently busy
// doing. Direction logic (`a != b`) is unchanged from the polling version —
// see loop()'s old comment history / PROGRESS.md for how that was derived.
constexpr uint32_t ENCODER_ISR_DEBOUNCE_MS = 2; // contact-bounce filter only — NOT a rotation-speed throttle like the old 30ms was
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t encoderStepAccum = 0; // net detents since loop() last drained it; ISR writes, loop() reads+clears
volatile uint32_t lastEncoderIsrMs = 0;

void IRAM_ATTR encoderIsr() {
  uint32_t now = millis();
  if (now - lastEncoderIsrMs < ENCODER_ISR_DEBOUNCE_MS) return;
  lastEncoderIsrMs = now;
  bool a = digitalRead(PIN_ENCODER_A) == HIGH;
  bool b = digitalRead(PIN_ENCODER_B) == HIGH;
  portENTER_CRITICAL_ISR(&encoderMux);
  encoderStepAccum += (a != b) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

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

// Whether pc-dash-app (the Windows companion, not built yet — see
// PROGRESS.md's "Next" list) has reported in recently, per the Docker app
// sync protocol's pcState.pcConnected. PC-dependent display data (these
// rings, Now Playing's track info) shows a dimmed/offline treatment when
// false instead of quietly showing stale numbers; PC-independent data
// (clock, weather, lights, settings) is unaffected either way.
bool pcConnected = false;
uint32_t lastSeenSecondsAgo = 0;
// pc-dash-app verified live on real Windows hardware 2026-08-20 (reported via
// the pc-dash coordinator) — offline dim treatment re-enabled now that there's
// a real PC to go offline from. pcConnected itself, the sync poll, and NVS
// persistence were never touched by this flag; only the *display* of the dim
// treatment was suppressed while testing (Brian, 2026-08-19).
constexpr bool PC_OFFLINE_DIM_ENABLED = true;

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
lv_obj_t *homeOfflineLabel;

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

  // PC-offline treatment (see pcConnected's own comment) — dim the rings
  // rather than hide them, so the layout doesn't jump, plus a label making
  // the "these numbers are stale/unavailable" state explicit. Gated on
  // PC_OFFLINE_DIM_ENABLED (see its own comment) — suppressed for now so
  // pcConnected's permanent false doesn't dim every screen during testing.
  bool showConnected = pcConnected || !PC_OFFLINE_DIM_ENABLED;
  lv_opa_t ringOpa = showConnected ? LV_OPA_COVER : LV_OPA_30;
  lv_obj_set_style_arc_opa(cpuArc, ringOpa, LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(gpuArc, ringOpa, LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(ramArc, ringOpa, LV_PART_INDICATOR);
  lv_obj_set_style_text_opa(cpuLabel, ringOpa, 0);
  lv_obj_set_style_text_opa(gpuLabel, ringOpa, 0);
  lv_obj_set_style_text_opa(ramLabel, ringOpa, 0);
  if (showConnected) {
    lv_obj_add_flag(homeOfflineLabel, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_remove_flag(homeOfflineLabel, LV_OBJ_FLAG_HIDDEN);
  }
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

  homeOfflineLabel = lv_label_create(scr);
  lv_obj_set_style_text_font(homeOfflineLabel, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(homeOfflineLabel, lv_color_hex(0x9AA5A3), 0);
  lv_label_set_text(homeOfflineLabel, "PC OFFLINE");
  lv_obj_align(homeOfflineLabel, LV_ALIGN_BOTTOM_MID, 0, -20);
  lv_obj_add_flag(homeOfflineLabel, LV_OBJ_FLAG_HIDDEN); // shown by updateHomeScreen() once pcConnected is known

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
// Mute started using the same fuse. Operator-adjustable from the Docker app
// (see "Docker app sync protocol" in ECOSYSTEM_PLAN.md) — mutable, not
// constexpr, so a synced value can override this hardcoded default; the
// default is what's used until NVS/a sync poll has ever supplied one.
constexpr uint32_t VOLUME_FUSE_MS_DEFAULT = 5000;
uint32_t volumeFuseMs = VOLUME_FUSE_MS_DEFAULT;
int volumePercent = 50;
uint32_t lastVolumeActivityMs = 0;
// Declared together with lastVolumeActivityMs (rather than down where each
// is actually used) since restoreScreen() below needs all three up front.
uint32_t lastSpeakerMuteActivityMs = 0;
uint32_t lastQuickJumpActivityMs = 0;

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
// Declared together with volumeScreen (rather than where each is actually
// built, further down) so the return-screen variables below can reference
// all three transient screens even though Speaker Mute's and Quick-jump's
// own build code comes later in the file.
lv_obj_t *speakerMuteScreen;
lv_obj_t *quickJumpScreen;
lv_obj_t *volumeArc;
lv_obj_t *volumeMuteIcon; // LV_SYMBOL_MUTE label, shown only at 0%
lv_obj_t *volumeWaveIcon; // single image widget, src swapped per tier
lv_obj_t *volumeLabel;

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
}

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
    volumeReturnScreen =
        (active == speakerMuteScreen && !speakerMuted) ? speakerMuteReturnScreen : active;
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
    speakerMuteReturnScreen = (active == volumeScreen) ? volumeReturnScreen : active;
    lv_screen_load(speakerMuteScreen);
  }
  speakerMuted = !speakerMuted;
  ConsumerControl.press(CONSUMER_CONTROL_MUTE);
  ConsumerControl.release();
  updateSpeakerMuteScreen();
  lastSpeakerMuteActivityMs = millis();
}

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
// Operator-adjustable from the Docker app, same as volumeFuseMs above.
constexpr uint32_t IDLE_TO_HOME_MS_DEFAULT = 30000; // standard non-Home idle timeout, see PROGRESS.md
uint32_t idleToHomeMs = IDLE_TO_HOME_MS_DEFAULT;
// Defaults shown until the first pcConnected poll reconciles them (see
// loop()'s sync-apply step) — String, not const char*, so a real synced
// value (arbitrary length, e.g. actual SMTC track titles) can replace them.
// lv_label_set_text() copies internally, so re-pointing these on every poll
// is safe even while a label is currently on screen.
String nowPlayingArtist = "Boards of Canada";
String nowPlayingTrack = "Roygbiv";
char nowPlayingElapsed[16] = "1:42";
// Placeholder for the source app (Brian, 2026-08-19, e.g. "YouTube") —
// SMTC exposes which app owns the active media session once pc-dash-app
// exists (see ECOSYSTEM_PLAN.md's "YouTube-aware pause vs. mute" section);
// per-app, not per-tab/site, so a browser tab may only ever show as the
// browser itself unless the site's own Media Session metadata disambiguates
// it further. Same "no companion app yet" placeholder pattern as artist/
// track/elapsed above.
String nowPlayingSource = "YouTube";
int nowPlayingProgressPercent = 38;

// mm:ss from a raw ms position — sync poll's granularity (~20s), not a
// locally-ticking clock; matches the same periodic-reconciliation pattern
// as volumePercent rather than adding a new per-second timer.
void formatNowPlayingElapsed(uint32_t ms) {
  uint32_t totalSeconds = ms / 1000;
  snprintf(nowPlayingElapsed, sizeof(nowPlayingElapsed), "%u:%02u", totalSeconds / 60, totalSeconds % 60);
}
// No HID readback exists for actual playback state — this is a local guess,
// toggled optimistically on each play/pause tap, same "local estimate only"
// pattern as volumePercent/speakerMuted. Icon shows the action a tap will
// perform next (pause glyph while assumed playing), the standard media-
// player convention — deliberately different from Speaker Mute's
// icon-shows-current-state convention, since transport controls are a
// different, well-established UI vocabulary.
bool nowPlayingIsPlaying = true;
uint32_t lastNowPlayingActivityMs = 0;

lv_obj_t *nowPlayingScreen; // declared here (not up with the other screens) since it's built later in file order
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
// Longer than volumeFuseMs (5s default) — this is a browsed menu you look at
// and choose from, not a transient nudge like Volume/Speaker Mute.
constexpr uint32_t QUICK_JUMP_RETURN_MS = 8000;

// quickJumpScreen and lastQuickJumpActivityMs are declared up near volumeScreen — see comment there.

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
  // Now Playing spot, wired up (2026-08-17) now that screen exists. Moved to
  // bottom-center (was bottom-right, mirroring the mockup's 3-spot layout)
  // now that Speaker Mute's bottom-left spot is gone — centering it under
  // Home reads better with only two spots than leaving it off to one side
  // with an empty gap opposite it. Uses its own click cb (not the generic
  // quickJumpSpotClickedCb every other spot uses) since entering Now Playing
  // also needs to start its own idle-timeout fuse — see showNowPlayingScreen().
  lv_obj_t *nowPlayingSpot =
      makeQuickJumpSpot(scr, 0, 100, lv_color_hex(QUICK_JUMP_NOW_PLAYING_COLOR), LV_SYMBOL_PLAY, nullptr);
  lv_obj_add_event_cb(nowPlayingSpot, [](lv_event_t *e) { showNowPlayingScreen(); }, LV_EVENT_CLICKED, NULL);
}

// Called on a detected swipe-up gesture, from Home, Volume, Speaker Mute, or
// Now Playing (see quickJumpGestureCb's own screen whitelist below) — mirrors
// showVolumeOverlay()'s remember-previous-screen pattern. Swiping up from
// Volume was originally excluded by design (rotate-only), but Brian asked
// for it directly (2026-08-06: "It doesn't work from the volume page,
// though. It should."). Now Playing was missing from quickJumpGestureCb's
// whitelist entirely until Brian caught it (2026-08-19) — added after Now
// Playing itself (2026-08-17) without updating this list.
void showQuickJumpMenu() {
  lv_obj_t *active = lv_screen_active();
  if (active == quickJumpScreen) return;
  quickJumpReturnScreen = active;
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
  if (dir != LV_DIR_LEFT && dir != LV_DIR_RIGHT) return;
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
  nowPlayingScreen = lv_obj_create(NULL);
  buildNowPlayingScreen(nowPlayingScreen);
  quickJumpScreen = lv_obj_create(NULL);
  buildQuickJumpScreen(quickJumpScreen);

  // Swipe-up entry point for the quick-jump grid. Subscribed on the indev
  // itself, not a screen object — see quickJumpGestureCb's comment for why
  // (LVGL sends LV_EVENT_GESTURE to the indev too, not just whichever
  // widget the bubble-up walk lands on). Screen filtering (Home/Speaker
  // Mute only, not Volume) happens inside the callback instead.
  lv_indev_add_event_cb(indev, quickJumpGestureCb, LV_EVENT_GESTURE, NULL);
  // Swipe left/right entry point for the Home <-> Now Playing toggle — same
  // indev-level subscription pattern, coexists fine with quickJumpGestureCb
  // above since each only acts on its own gesture direction.
  lv_indev_add_event_cb(indev, screenCycleGestureCb, LV_EVENT_GESTURE, NULL);
  makeGestureBubbleThrough(homeScreen); // still correct practice even though no longer load-bearing for this
  makeGestureBubbleThrough(volumeScreen);
  makeGestureBubbleThrough(speakerMuteScreen);

  volumeReturnScreen = homeScreen;
  speakerMuteReturnScreen = homeScreen;
  quickJumpReturnScreen = homeScreen;
  lv_screen_load(homeScreen);

  Serial.println("LVGL init OK");
}

// Docker app sync protocol (pc-dash-hub) — see ECOSYSTEM_PLAN.md's "Docker
// app sync protocol" section for the full design. Firmware polls one
// combined settings+PC-state endpoint over plain HTTP; no push/WebSocket
// (matches this project's own hard-won polling-beats-push lesson, see
// HARDWARE.md). NVS storage is one namespace, one key per settings field —
// a firmware update adding a new field just falls back to its hardcoded
// default instead of one corrupt blob invalidating everything.
//
// Runs on its own FreeRTOS task (syncPollTask, added 2026-08-19), not
// inline in loop() as it originally was. HTTPClient.GET() blocks for up to
// SYNC_HTTP_TIMEOUT_MS per attempt, and one poll can make two attempts
// back-to-back (hostname, then the IP fallback) — so a slow/unresponsive
// pc-dash-hub could stall loop() for ~8s at a time. ArduinoOTA.handle()
// (called from loop()) has no thread of its own — it's only ever serviced
// when loop() reaches it — so an OTA upload's UDP invitation/auth handshake
// landing inside one of those stalls could time out or get corrupted,
// causing an AUTH failure on the uploading side with the board itself never
// actually going down (confirmed via WebSerial staying reachable through
// several such failures this session — see PROGRESS.md; this was the root
// cause, not a watchdog reset). WebSerial/OTA's own web server survived
// those stalls fine because AsyncTCP genuinely runs its own task; this poll
// didn't, until now. The task below does the actual blocking network I/O
// and JSON parse entirely off the main/LVGL task, then hands the result to
// loop() via a mutex — loop() remains the only thing that ever touches LVGL
// objects (not thread-safe), so applying pcConnected's on-screen effect
// stays there.
constexpr const char *SYNC_HOST = "pcdashhub.lan"; // .lan hostname primary, SYNC_FALLBACK_IP (secrets.h) if this ever fails to resolve
constexpr uint16_t SYNC_PORT = 8015;
constexpr const char *SYNC_DEVICE_ID = "t-encoder-pro"; // matches WiFi.setHostname() below — pc-dash-hub has no allowlist, just needs to match
constexpr uint32_t SYNC_POLL_INTERVAL_CONNECTED_MS = 20000; // ~15-30s once a poll has succeeded
constexpr uint32_t SYNC_POLL_INTERVAL_RETRY_MS = 5000;      // fast retry while the hub's unreachable — also what gives near-instant resync once it comes back
constexpr uint32_t SYNC_HTTP_TIMEOUT_MS = 4000;

Preferences syncPrefs; // only ever touched from syncPollTask post-refactor — see setupSyncSettings()'s comment
// Whether the *poll itself* reached pc-dash-hub and got a parseable 200 —
// distinct from pcConnected (the server's own "has pc-dash-app reported in"
// answer, currently a permanent false stub — see pcConnected's comment).
// Kept separate so "hub unreachable" and "hub reachable, no PC yet" don't
// get conflated in debug output. Only ever written by loop() (the result-
// apply step below), so it needs no synchronization of its own.
bool lastSyncPollOk = false;

// Handoff from syncPollTask (background) to loop() (main/LVGL task).
// syncResultMutex guards pendingSyncResult; syncResultPending is checked
// without the mutex first as a cheap early-out (only a bool read) — the
// mutex is only actually taken once there's something to apply.
SemaphoreHandle_t syncResultMutex;
volatile bool syncResultPending = false;
struct SyncResult {
  bool ok;
  bool pcConnected;
  uint32_t lastSeenSecondsAgo;
  uint32_t volumeFuseMs;
  uint32_t idleToHomeMs;
  // pcState's volume/mute/now-playing fields (T14, 2026-08-21) — parsed
  // unconditionally below, but loop() only ever applies them when
  // result.pcConnected is true (see loop()'s sync-apply step and
  // ECOSYSTEM_PLAN.md's "Volume/mute/media reconciliation" section).
  int volumePercent;
  bool speakerMuted;
  bool nowPlayingIsPlaying;
  String nowPlayingArtist;
  String nowPlayingTrack;
  String nowPlayingSource;
  uint32_t nowPlayingPositionMs;
  uint32_t nowPlayingDurationMs;
};
SyncResult pendingSyncResult;

// GET the state blob from one host, true + fills `body` on success. Only
// ever called from syncPollTask — safe to block here.
bool syncGet(const char *host, String &body) {
  HTTPClient http;
  String url = String("http://") + host + ":" + SYNC_PORT + "/api/devices/" + SYNC_DEVICE_ID + "/state";
  http.begin(url);
  http.setConnectTimeout(SYNC_HTTP_TIMEOUT_MS);
  http.setTimeout(SYNC_HTTP_TIMEOUT_MS);

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("sync: GET %s failed, code=%d\n", host, httpCode);
    http.end();
    return false;
  }
  body = http.getString();
  http.end();
  return true;
}

// One poll: GET the combined settings+PC-state blob, parse it, stage the
// result for loop() to apply (never touches LVGL or the real globals
// itself — see this section's top comment). Tries the .lan hostname first,
// falls back to the static IP (secrets.h) if that fails — e.g. the
// router's local DNS is briefly down but the box itself is fine. Stages a
// result on every call, success or failure, so loop()'s syncOk reading
// always reflects the most recent attempt rather than going stale. Returns
// true on a successful, parseable response from either host — only ever
// called from syncPollTask.
bool pollSyncState() {
  SyncResult result;
  result.volumeFuseMs = volumeFuseMs; // preserved unless the payload has a real value
  result.idleToHomeMs = idleToHomeMs;
  result.pcConnected = false;
  result.lastSeenSecondsAgo = 0;
  // Left at these neutral defaults (not read from the local globals — those
  // are also written by loop()/ISR code on this same board, so avoiding a
  // cross-task read here is simpler than reasoning about the race) on a
  // failed/unparseable poll; harmless since loop() only ever applies these
  // fields when result.pcConnected is true anyway.
  result.volumePercent = 0;
  result.speakerMuted = false;
  result.nowPlayingIsPlaying = false;
  result.nowPlayingArtist = "";
  result.nowPlayingTrack = "";
  result.nowPlayingSource = "";
  result.nowPlayingPositionMs = 0;
  result.nowPlayingDurationMs = 0;
  result.ok = false;

  if (WiFi.status() == WL_CONNECTED) {
    String body;
    if (syncGet(SYNC_HOST, body) || syncGet(SYNC_FALLBACK_IP, body)) {
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, body);
      if (err) {
        Serial.printf("sync: JSON parse failed: %s\n", err.c_str());
      } else {
        JsonObject timeouts = doc["settings"]["timeouts"];
        if (!timeouts.isNull()) {
          // Only the two timeouts are consumed today — buzzer/vibration/
          // clockFaceId/weather/activeProfile/macros exist server-side but
          // nothing in this firmware reads them yet (see PROGRESS.md's
          // "Next" list); wire each up here once the feature reading it
          // actually exists, rather than storing fields nothing uses.
          result.volumeFuseMs = timeouts["volumeFuseMs"] | volumeFuseMs;
          result.idleToHomeMs = timeouts["idleToHomeMs"] | idleToHomeMs;
          if (result.volumeFuseMs != volumeFuseMs) syncPrefs.putUInt("volFuseMs", result.volumeFuseMs);
          if (result.idleToHomeMs != idleToHomeMs) syncPrefs.putUInt("idleMs", result.idleToHomeMs);
        }
        // pc-dash-app (step 4) is live now (2026-08-20) — pcState carries
        // real volume/mute/now-playing values, reconciled against the local
        // knob-driven estimate in loop() once pcConnected is true (T14,
        // 2026-08-21; see each var's own comment for the prior placeholder
        // state, and ECOSYSTEM_PLAN.md's "Volume/mute/media reconciliation"
        // section for the design this implements).
        JsonObject pcState = doc["pcState"];
        result.pcConnected = pcState["pcConnected"] | false;
        result.lastSeenSecondsAgo = pcState["lastSeenSecondsAgo"] | 0;
        result.volumePercent = pcState["volumePercent"] | 0;
        result.speakerMuted = pcState["speakerMuted"] | false;
        result.nowPlayingIsPlaying = pcState["nowPlayingIsPlaying"] | false;
        JsonObject nowPlaying = pcState["nowPlaying"];
        result.nowPlayingTrack = nowPlaying["title"] | "";
        result.nowPlayingArtist = nowPlaying["artist"] | "";
        result.nowPlayingSource = nowPlaying["source"] | "";
        result.nowPlayingPositionMs = nowPlaying["positionMs"] | 0;
        result.nowPlayingDurationMs = nowPlaying["durationMs"] | 0;
        result.ok = true;
      }
    }
  }

  xSemaphoreTake(syncResultMutex, portMAX_DELAY);
  pendingSyncResult = result;
  syncResultPending = true;
  xSemaphoreGive(syncResultMutex);

  return result.ok;
}

// Background task body — see this section's top comment for why this
// exists. vTaskDelay() actually yields the CPU (unlike a busy-wait on
// millis()), so this costs nothing while idle between polls. Same "retry
// fast after failure, relax once healthy" adaptive shape as before, just
// driven by a real task delay instead of a millis() comparison in loop().
void syncPollTask(void *pvParameters) {
  for (;;) {
    bool ok = pollSyncState();
    vTaskDelay(pdMS_TO_TICKS(ok ? SYNC_POLL_INTERVAL_CONNECTED_MS : SYNC_POLL_INTERVAL_RETRY_MS));
  }
}

// Loads NVS-persisted settings (if any) over the hardcoded defaults already
// assigned at global scope, then starts the background poll task. Doesn't
// need WiFi first — called early in setup(), before setupOTA() connects —
// operator-adjusted values (from a previous sync) take effect even if this
// boot never reaches the Docker app at all, and the task itself just finds
// WiFi not-yet-connected and retries at the fast interval until it is (same
// as any other disconnected state, no special-casing needed). Pinned to
// core 0 to stay clear of core 1's loop()/LVGL rendering entirely — not
// just avoiding blocking it, but not competing with it for CPU time either.
// syncPrefs (NVS) is opened here, on the main task, then only ever written
// from syncPollTask afterward — sequential-but-cross-task, never
// concurrent, since this function returns (and its effects are published to
// the new task via xTaskCreatePinnedToCore) before the task's first poll
// could possibly reach a putUInt() call.
void setupSyncSettings() {
  syncPrefs.begin("sync", false);
  volumeFuseMs = syncPrefs.getUInt("volFuseMs", VOLUME_FUSE_MS_DEFAULT);
  idleToHomeMs = syncPrefs.getUInt("idleMs", IDLE_TO_HOME_MS_DEFAULT);
  Serial.printf("sync: loaded from NVS: volumeFuseMs=%u idleToHomeMs=%u\n", volumeFuseMs, idleToHomeMs);

  syncResultMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(syncPollTask, "syncPoll", 8192, NULL, 1, NULL, 0);
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

// Debug console over WiFi (http://<device-ip>/webserial) — added 2026-08-07
// so runtime logging doesn't need a USB cable, same reasoning as OTA: once
// this board lives permanently on Brian's desktop, this session has no
// physical/serial access to it at all (see PROGRESS.md's quick-jump/swipe
// session for how much that hurt debugging touch behavior earlier).
// Needs WiFi already up, so only called from setup() after setupOTA() — same
// "no WiFi, no OTA" graceful-degradation shape as that function.
AsyncWebServer webServer(80);

void setupWebSerial() {
  if (WiFi.status() != WL_CONNECTED) return; // no WiFi, no WebSerial — same as OTA
  WebSerial.begin(&webServer);
  WebSerial.onMessage([](const String &msg) { Serial.printf("WebSerial recv: %s\n", msg.c_str()); });
  webServer.begin();
  Serial.println("WebSerial ready at /webserial");
}

// HTTP-based OTA (added 2026-08-21) — a single-connection POST upload,
// alongside ArduinoOTA rather than replacing it. Exists because ArduinoOTA's
// espota protocol is stop-and-wait (one ~1460-byte TCP chunk, wait for a
// text ACK, repeat — ~1,150 round trips for this firmware's ~1.7MB image),
// which measured at ~266s for one flash. This POST handler streams the body
// as AsyncTCP delivers it — no per-chunk ACK round trip — and, critically,
// runs from AsyncTCP's own task, not loop(), so it isn't gated by loop()'s
// per-iteration LVGL/encoder work either (see the "OTA transfer speed"
// investigation in PROGRESS.md/conversation history for why that doesn't
// actually matter for ArduinoOTA's *transfer* phase specifically, but does
// for its invitation/auth handshake — this path sidesteps both).
// Auth reuses OTA_PASSWORD (secrets.h) as HTTP Basic Auth's password, fixed
// username "ota" — same secret, different transport, rather than minting a
// second one nobody will remember to keep in sync.
bool httpOtaRebootPending = false;
uint32_t httpOtaRebootAtMs = 0;

void setupHttpOta() {
  if (WiFi.status() != WL_CONNECTED) return; // no WiFi, no HTTP OTA — same as ArduinoOTA/WebSerial

  webServer.on(
      "/firmware-update", HTTP_POST,
      [](AsyncWebServerRequest *request) {
        // Fires once after the upload handler below has already processed
        // the whole body — too late to reject an unauthenticated *upload*
        // (the upload handler's own index==0 check below is what actually
        // gates that), but this is still where the HTTP response for the
        // request goes, auth failure included.
        if (!request->authenticate("ota", OTA_PASSWORD)) {
          return request->requestAuthentication();
        }
        bool ok = !Update.hasError();
        request->send(ok ? 200 : 500, "text/plain", ok ? "OK" : "FAIL");
        if (ok) {
          // Reboot from loop() after a short grace period, not inline here
          // — give the AsyncTCP task time to actually flush this response
          // to the socket before ESP.restart() tears everything down.
          httpOtaRebootPending = true;
          httpOtaRebootAtMs = millis() + 500;
        }
      },
      [](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final) {
        if (!index) {
          if (!request->authenticate("ota", OTA_PASSWORD)) {
            Serial.println("HTTP OTA: rejected, bad auth");
            return; // main handler above still runs and sends 401 once the (now-ignored) body finishes arriving
          }
          Serial.printf("HTTP OTA: start, filename=%s\n", filename.c_str());
          if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Update.printError(Serial);
          }
        }
        if (!request->authenticate("ota", OTA_PASSWORD)) return; // keep rejecting every chunk of a bad-auth upload
        if (len) Update.write(data, len);
        // Yield after every chunk — AsyncTCP can deliver large chunks
        // back-to-back with no gap between them (unlike ArduinoOTA's own
        // _runUpdate(), which naturally paces itself via delay(1) while
        // polling client.available()). Without this, a full-size image's
        // worth of back-to-back flash writes on this task starved the idle
        // task long enough to trip the watchdog and silently reboot the
        // board mid-transfer (confirmed: a first attempt without this
        // failed partway through and the board came back up on its own).
        delay(1);
        if (final) {
          if (Update.end(true)) {
            Serial.printf("HTTP OTA: done, %u bytes\n", (unsigned)(index + len));
          } else {
            Update.printError(Serial);
          }
        }
      });

  Serial.println("HTTP OTA ready at /firmware-update");
}

void setup() {
  Serial.begin(115200);
  setupSyncSettings(); // NVS load, no WiFi needed — operator-adjusted timeouts apply even if this boot never reaches pc-dash-hub
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_KEY, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderIsr, CHANGE);

  ConsumerControl.begin();
  USB.begin();

  setupDisplay();
  setupTouch();
  setupLVGL();
  setupOTA();
  setupWebSerial();
  setupHttpOta();

  Serial.println("T-Encoder Pro: HID volume/mute ready");
}

void loop() {
  static int lastKey = HIGH;
  static uint32_t lastKeyMs = 0;
  static uint32_t lastStatusMs = 0;

  ArduinoOTA.handle();
  WebSerial.loop();

  // HTTP OTA finished successfully (set from the AsyncTCP task — see
  // setupHttpOta()'s comment) — reboot from here, not inline in that
  // handler, so the AsyncTCP task gets a chance to actually flush the "OK"
  // response to the socket first.
  if (httpOtaRebootPending && (int32_t)(millis() - httpOtaRebootAtMs) >= 0) {
    ESP.restart();
  }

  // Apply the latest result from syncPollTask, if any — cheap early-out
  // first (a plain bool read), mutex only taken once there's actually
  // something to apply. The network I/O itself doesn't happen here anymore
  // (see the "Docker app sync protocol" section's top comment for why:
  // loop() must never block, since ArduinoOTA.handle() above only ever gets
  // serviced when loop() reaches it).
  if (syncResultPending) {
    xSemaphoreTake(syncResultMutex, portMAX_DELAY);
    SyncResult result = pendingSyncResult;
    syncResultPending = false;
    xSemaphoreGive(syncResultMutex);

    lastSyncPollOk = result.ok;
    if (result.ok) {
      bool wasPcConnected = pcConnected;
      pcConnected = result.pcConnected;
      lastSeenSecondsAgo = result.lastSeenSecondsAgo;
      volumeFuseMs = result.volumeFuseMs;
      idleToHomeMs = result.idleToHomeMs;

      // Volume/mute/now-playing: only trust the payload once the companion
      // app has actually reported in on *this* poll (result.pcConnected) —
      // see ECOSYSTEM_PLAN.md's "Volume/mute/media reconciliation" section.
      // While disconnected, the local optimistic estimate (set by knob
      // rotation / button press / tap) is left uncorrected rather than
      // being overwritten by the hub's never-set defaults.
      bool pcDataChanged = false;
      if (pcConnected) {
        pcDataChanged = volumePercent != result.volumePercent ||
                         speakerMuted != result.speakerMuted ||
                         nowPlayingIsPlaying != result.nowPlayingIsPlaying ||
                         nowPlayingArtist != result.nowPlayingArtist ||
                         nowPlayingTrack != result.nowPlayingTrack ||
                         nowPlayingSource != result.nowPlayingSource;
        volumePercent = result.volumePercent;
        speakerMuted = result.speakerMuted;
        nowPlayingIsPlaying = result.nowPlayingIsPlaying;
        nowPlayingArtist = result.nowPlayingArtist;
        nowPlayingTrack = result.nowPlayingTrack;
        nowPlayingSource = result.nowPlayingSource;
        nowPlayingProgressPercent = result.nowPlayingDurationMs > 0
            ? constrain((int)(((uint64_t)result.nowPlayingPositionMs * 100) / result.nowPlayingDurationMs), 0, 100)
            : 0;
        formatNowPlayingElapsed(result.nowPlayingPositionMs);
        updateVolumeScreen();
        updateSpeakerMuteScreen();
      }

      if (pcConnected != wasPcConnected || pcDataChanged) {
        updateHomeScreen();
        updateNowPlayingScreen();
      }
    }
  }

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
    // syncOk = did the last poll actually reach pc-dash-hub and parse.
    // pcConnected = the hub's own answer to "has pc-dash-app reported in"
    // (currently a permanent false stub server-side — see pcConnected's
    // comment). Logged separately on purpose: syncOk=no means "can't reach
    // the hub," syncOk=yes+pcConnected=no means "reached it fine, it just
    // doesn't have a PC yet" — collapsing these into one field would make
    // the two indistinguishable from this log alone.
    Serial.printf("status: wifi=%s display=%s touch=%s ip=%s syncOk=%s pcConnected=%s lastSeen=%us\n",
                   WiFi.status() == WL_CONNECTED ? "up" : "down",
                   displayOk ? "ok" : "FAIL",
                   touchOk ? "ok" : "FAIL",
                   WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "-",
                   lastSyncPollOk ? "yes" : "no",
                   pcConnected ? "yes" : "no",
                   lastSeenSecondsAgo);
    WebSerial.printf("status: wifi=%s display=%s touch=%s ip=%s syncOk=%s pcConnected=%s lastSeen=%us\n",
                      WiFi.status() == WL_CONNECTED ? "up" : "down",
                      displayOk ? "ok" : "FAIL",
                      touchOk ? "ok" : "FAIL",
                      WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "-",
                      lastSyncPollOk ? "yes" : "no",
                      pcConnected ? "yes" : "no",
                      lastSeenSecondsAgo);
  }

  // Drain whatever encoderIsr() has accumulated since the last iteration —
  // see its comment (near the top of this file) for why reading happens via
  // interrupt now instead of polling here. One HID pulse + overlay update
  // per real detent, same as before, just sourced from the ISR's count
  // instead of a fresh digitalRead() each time.
  int32_t steps;
  portENTER_CRITICAL(&encoderMux);
  steps = encoderStepAccum;
  encoderStepAccum = 0;
  portEXIT_CRITICAL(&encoderMux);
  while (steps > 0) {
    ConsumerControl.press(CONSUMER_CONTROL_VOLUME_INCREMENT);
    ConsumerControl.release();
    showVolumeOverlay(VOLUME_STEP);
    Serial.println("VOL_UP");
    WebSerial.println("VOL_UP");
    steps--;
  }
  while (steps < 0) {
    ConsumerControl.press(CONSUMER_CONTROL_VOLUME_DECREMENT);
    ConsumerControl.release();
    showVolumeOverlay(-VOLUME_STEP);
    Serial.println("VOL_DOWN");
    WebSerial.println("VOL_DOWN");
    steps++;
  }

  // 5s-after-last-turn fuse: restore whatever screen Volume interrupted.
  if (lv_screen_active() == volumeScreen && (millis() - lastVolumeActivityMs) > volumeFuseMs) {
    restoreScreen(volumeReturnScreen);
  }
  // Same fuse for Speaker Mute, but only while *not* muted (Brian: "when
  // muted, it should not [return] to the main screen... only return after
  // unmuting") — staying muted is a deliberate, persistent state worth
  // keeping visible indefinitely, unlike Volume's transient adjustments.
  // The fuse only starts counting once you unmute (lastSpeakerMuteActivityMs
  // resets on every button press, so unmuting gets its own fresh 5s window).
  if (lv_screen_active() == speakerMuteScreen && !speakerMuted &&
      (millis() - lastSpeakerMuteActivityMs) > volumeFuseMs) {
    restoreScreen(speakerMuteReturnScreen);
  }
  // Quick-jump's own, longer idle fuse: walked away without tapping
  // anything, restore whatever screen it was opened from.
  if (lv_screen_active() == quickJumpScreen && (millis() - lastQuickJumpActivityMs) > QUICK_JUMP_RETURN_MS) {
    restoreScreen(quickJumpReturnScreen);
  }
  // Now Playing's idle fuse always falls back to Home directly — unlike the
  // overlay screens above, it's not "restore whatever this interrupted"
  // (see PROGRESS.md's design rules: the standard 30s-idle-to-Home rule for
  // any non-Home screen, first real implementation of it since Now Playing
  // is the first non-overlay non-Home screen to exist).
  if (lv_screen_active() == nowPlayingScreen && (millis() - lastNowPlayingActivityMs) > idleToHomeMs) {
    lv_screen_load(homeScreen);
  }

  int key = digitalRead(PIN_ENCODER_KEY);
  if (key != lastKey) {
    lastKey = key;
    uint32_t now = millis();
    if (key == LOW && (now - lastKeyMs) > DEBOUNCE_MS) {
      lastKeyMs = now;
      showSpeakerMuteOverlay(); // sends the HID mute pulse itself
      Serial.println("MUTE");
      WebSerial.println("MUTE");
    }
  }

  delay(5); // matches Arduino_GFX's own LVGL example's loop cadence
}
