// display.cpp — CO5300 AMOLED panel, LVGL buffers/flush/tick, brightness levels, setupDisplay()/setupLVGL().

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include "pin_config.h"
#include "main_decls.h"
#include "input.h"
#include "display.h"
#include "agent_status.h"
#include "screen_nav.h"

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

void setupDisplay() {
  pinMode(SCREEN_EN, OUTPUT);
  digitalWrite(SCREEN_EN, HIGH);

  displayOk = panel->begin();
  if (!displayOk) {
    Serial.println("Display init FAILED");
    return;
  }
  Serial.println("Display init OK");

  panel->setBrightness(DISPLAY_BRIGHTNESS_NORMAL);
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
  idleScreen = lv_obj_create(NULL);
  buildIdleScreen(idleScreen);

  // T77: parented to lv_layer_top(), the one LVGL layer that stays rendered
  // above every screen regardless of which one is currently loaded — see
  // updateAgentAttentionRing()'s own comment for why this lives here instead
  // of as a per-screen child like agentStatusBadge/agentFlashOverlay.
  agentAttentionRing = lv_obj_create(lv_layer_top());
  lv_obj_remove_flag(agentAttentionRing, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(agentAttentionRing, LV_OBJ_FLAG_CLICKABLE); // must not block touch/gestures on the screen underneath
  lv_obj_set_size(agentAttentionRing, SCREEN_WIDTH - 4, SCREEN_HEIGHT - 4); // just inside the true bezel edge
  lv_obj_center(agentAttentionRing);
  lv_obj_set_style_radius(agentAttentionRing, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_opa(agentAttentionRing, LV_OPA_TRANSP, 0); // ring only, no fill
  lv_obj_set_style_border_width(agentAttentionRing, 6, 0); // thin, per spec — much lighter than agentStatusBadge's 7px on a 34px badge
  lv_obj_set_style_pad_all(agentAttentionRing, 0, 0);
  lv_obj_add_flag(agentAttentionRing, LV_OBJ_FLAG_HIDDEN); // shown by updateAgentAttentionRing() once there's a real attention state
  updateAgentAttentionRing();
  // No gesture subscriptions/makeGestureBubbleThrough on idleScreen, deliberately
  // — a touch here should just wake the device (any touch already updates
  // lastAnyInteractionMs via lvTouchpadRead()), not trigger a specific
  // nested action like Quick-jump or the Home<->Now-Playing swipe would.

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
