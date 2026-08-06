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
float cpuPercent = 42;
float gpuPercent = 54;
float ramPercent = 61;

// LVGL owns its own draw buffer and rendering entirely — this replaces the
// earlier direct-to-Arduino_GFX drawHomeScreen()/drawRingGauge() approach.
// Partial buffer (40 rows, matching Arduino_GFX's own LVGL example), not a
// full-frame buffer: LVGL renders in chunks and calls lvDispFlush() per
// chunk, which pushes each chunk to the panel with draw16bitRGBBitmap() — a
// single contiguous blit per chunk, structurally the same "few large writes,
// not many small scanline writes" shape that fixed the earlier fillArc/
// fillCircle corruption (see PROGRESS.md "Display rendering bug"), so this
// should avoid that failure mode too even though it's a different code path.
constexpr uint32_t LV_BUF_ROWS = 40;
lv_display_t *lvDisplay = nullptr;
lv_color_t *lvDrawBuf = nullptr;

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
  if (touchOk && touch.isPressed()) {
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
  lv_arc_set_value(cpuArc, (int)cpuPercent);
  lv_arc_set_value(gpuArc, (int)gpuPercent);
  lv_arc_set_value(ramArc, (int)ramPercent);
  lv_label_set_text_fmt(cpuLabel, "CPU %d%%", (int)cpuPercent);
  lv_label_set_text_fmt(gpuLabel, "GPU %d%%", (int)gpuPercent);
  lv_label_set_text_fmt(ramLabel, "RAM %d%%", (int)ramPercent);
}

void buildHomeScreen() {
  lv_obj_t *scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

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

  uint32_t bufPixels = SCREEN_WIDTH * LV_BUF_ROWS;
  lvDrawBuf = (lv_color_t *)heap_caps_malloc(bufPixels * sizeof(lv_color_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!lvDrawBuf) {
    lvDrawBuf = (lv_color_t *)heap_caps_malloc(bufPixels * sizeof(lv_color_t), MALLOC_CAP_8BIT);
  }
  if (!lvDrawBuf) {
    Serial.println("LVGL draw buffer allocation FAILED");
    return;
  }

  lvDisplay = lv_display_create(SCREEN_WIDTH, SCREEN_HEIGHT);
  lv_display_set_flush_cb(lvDisplay, lvDispFlush);
  lv_display_set_buffers(lvDisplay, lvDrawBuf, NULL, bufPixels * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, lvTouchpadRead);

  buildHomeScreen();
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
        Serial.println("VOL_UP");
      } else {
        ConsumerControl.press(CONSUMER_CONTROL_VOLUME_DECREMENT);
        ConsumerControl.release();
        Serial.println("VOL_DOWN");
      }
    }
  }

  int key = digitalRead(PIN_ENCODER_KEY);
  if (key != lastKey) {
    lastKey = key;
    uint32_t now = millis();
    if (key == LOW && (now - lastKeyMs) > DEBOUNCE_MS) {
      lastKeyMs = now;
      ConsumerControl.press(CONSUMER_CONTROL_MUTE);
      ConsumerControl.release();
      Serial.println("MUTE");
    }
  }

  delay(5); // matches Arduino_GFX's own LVGL example's loop cadence
}
