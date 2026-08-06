// Volume/mic-mute peripheral: rotating the encoder sends USB HID consumer-control
// volume up/down, pressing the knob sends mute. Requires ARDUINO_USB_MODE=0 (TinyUSB
// composite CDC+HID) — see PROGRESS.md for why mode=1 (used during initial bring-up)
// can't do HID at all.

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Arduino_GFX_Library.h>
#include <TouchDrv.hpp>
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
// Typed as Arduino_CO5300*, not the base Arduino_GFX*, specifically so
// setBrightness() (a CO5300 extension, not in the base drawing-primitive
// interface) is callable without a cast.
Arduino_CO5300 *gfx = new Arduino_CO5300(
    displayBus, SCREEN_RST, 0 /* rotation */, false /* IPS */, SCREEN_WIDTH, SCREEN_HEIGHT);

bool displayOk = false;

// Placeholder values until the companion app protocol exists (see
// PROGRESS.md's "Next" list) — proving the ring-gauge rendering works on
// real hardware first, same as every other piece of this firmware so far.
float cpuPercent = 42;
float gpuPercent = 54;
float ramPercent = 61;

// One ring = a dim full-circle "track" plus a colored arc for the value,
// swept clockwise starting from 12 o'clock (-90°) — matches the watch-face
// mockup from the original UI brainstorm. Arduino_GFX's fillArc uses
// standard math angle convention (0°=3 o'clock), which appears clockwise on
// screen since screen Y increases downward — confirmed by visual check on
// hardware, not just assumed from reading the source.
void drawRingGauge(int16_t cx, int16_t cy, int16_t rOuter, int16_t rInner, float pct, uint16_t color) {
  gfx->fillArc(cx, cy, rOuter, rInner, 0, 360, DARKGREY);
  if (pct > 0) {
    float endAngle = -90 + (pct / 100.0f) * 360.0f;
    gfx->fillArc(cx, cy, rOuter, rInner, -90, endAngle, color);
  }
}

void drawHomeScreen() {
  gfx->fillScreen(BLACK);

  constexpr int16_t cx = SCREEN_WIDTH / 2;
  constexpr int16_t cy = SCREEN_HEIGHT / 2;

  // DIAGNOSTIC (2026-08-06): rings weren't showing up at all — not even the
  // colored value-arc segments, which use the same colors as text that DID
  // render fine. Isolating whether this is arc-specific or something
  // broader about non-full-screen fills on this display. Remove once
  // resolved.
  gfx->fillRect(20, 20, 60, 60, RED);
  gfx->fillCircle(100, 100, 30, MAGENTA);

  drawRingGauge(cx, cy, 185, 165, cpuPercent, ORANGE);
  drawRingGauge(cx, cy, 160, 140, gpuPercent, BLUE);
  drawRingGauge(cx, cy, 135, 115, ramPercent, GREEN);

  gfx->setTextSize(2);
  gfx->setTextColor(ORANGE);
  gfx->setCursor(cx - 70, cy - 40);
  gfx->printf("CPU %d%%", (int)cpuPercent);
  gfx->setTextColor(BLUE);
  gfx->setCursor(cx - 70, cy - 10);
  gfx->printf("GPU %d%%", (int)gpuPercent);
  gfx->setTextColor(GREEN);
  gfx->setCursor(cx - 70, cy + 20);
  gfx->printf("RAM %d%%", (int)ramPercent);
}

void setupDisplay() {
  pinMode(SCREEN_EN, OUTPUT);
  digitalWrite(SCREEN_EN, HIGH);

  displayOk = gfx->begin();
  if (!displayOk) {
    Serial.println("Display init FAILED");
    return;
  }
  Serial.println("Display init OK");

  gfx->setBrightness(200);
  drawHomeScreen();
}

// Touch bring-up only at this stage — same "prove it works before building
// on it" approach as the encoder and display. LilyGo's README says this
// board ships with either CHSC5816 or CST816 touch IC depending on batch;
// CHSC5816 failed init on this actual board (confirmed via the periodic
// status log, 2026-08-06) — consistent with the display turning out to be
// CO5300 rather than SH8601, so trying CST816 next.
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

  if (touchOk && touch.isPressed()) {
    TouchPoints points = touch.getTouchPoints();
    if (points.hasPoints()) {
      const TouchPoint &p = points.getPoint(0);
      Serial.printf("TOUCH x=%d y=%d\n", p.x, p.y);
    }
  }

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
}
