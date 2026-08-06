// Volume/mic-mute peripheral: rotating the encoder sends USB HID consumer-control
// volume up/down, pressing the knob sends mute. Requires ARDUINO_USB_MODE=0 (TinyUSB
// composite CDC+HID) — see PROGRESS.md for why mode=1 (used during initial bring-up)
// can't do HID at all.

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Arduino_GFX_Library.h>
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
// but that class doesn't exist in the pinned 1.5.0 library version this
// project uses — only Arduino_CO5300 is present. LilyGo's own README says
// this board ships with *either* SH8601 or CO5300 depending on batch, so
// trying CO5300 first since it's what's actually available; if the panel
// stays dark/garbled, that's the signal this board has SH8601 instead and
// needs either a newer library version or a ported driver.
Arduino_DataBus *displayBus = new Arduino_ESP32QSPI(
    SCREEN_CS, SCREEN_SCLK, SCREEN_SDIO0, SCREEN_SDIO1, SCREEN_SDIO2, SCREEN_SDIO3);
// Typed as Arduino_CO5300*, not the base Arduino_GFX*, specifically so
// setBrightness() (a CO5300 extension, not in the base drawing-primitive
// interface) is callable without a cast.
Arduino_CO5300 *gfx = new Arduino_CO5300(
    displayBus, SCREEN_RST, 0 /* rotation */, false /* IPS */, SCREEN_WIDTH, SCREEN_HEIGHT);

void setupDisplay() {
  pinMode(SCREEN_EN, OUTPUT);
  digitalWrite(SCREEN_EN, HIGH);

  if (!gfx->begin()) {
    Serial.println("Display init FAILED");
    return;
  }
  Serial.println("Display init OK");

  gfx->fillScreen(BLACK);
  gfx->setBrightness(200);
  gfx->setCursor(60, 180);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  gfx->println("T-Encoder Pro");
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

  // Periodic, not just at-boot: a one-time boot message is easy to miss
  // when reading serial after the fact. This makes WiFi/IP status visible
  // no matter when you happen to be listening.
  if (millis() - lastStatusMs > 10000) {
    lastStatusMs = millis();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("status: wifi up, ip=%s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("status: wifi down");
    }
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
