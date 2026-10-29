// wifi_ota.cpp — WiFi connect, ArduinoOTA and SNTP time sync.

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "secrets.h"
#include "display.h"
#include "screen_nav.h"
#include "wifi_ota.h"

constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;

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
  // Dim for the duration of the write, same reasoning/mitigation as
  // setupHttpOta()'s upload handler (see bootResetReason's own comment) —
  // this is the other of the two OTA paths, equally exposed to the same
  // flash-write current spike.
  ArduinoOTA.onStart([]() {
    Serial.println("OTA: start");
    panel->setBrightness(DISPLAY_BRIGHTNESS_OTA_DIM);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("OTA: done");
    panel->setBrightness(displayDimmed ? DISPLAY_BRIGHTNESS_IDLE : DISPLAY_BRIGHTNESS_NORMAL);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA error: %u\n", error);
    panel->setBrightness(displayDimmed ? DISPLAY_BRIGHTNESS_IDLE : DISPLAY_BRIGHTNESS_NORMAL);
  });
  ArduinoOTA.begin();
  Serial.println("OTA ready");
}

// T40 (2026-08-25): NTP time sync for the idle screen's clock — nothing on
// this board read/needed wall-clock time before this. configTzTime() starts
// SNTP and sets the TZ env var in one call; it doesn't block — getLocalTime()
// (see updateIdleElement()) just returns false until the first sync actually
// lands. Only called once WiFi is up (same "no WiFi, no X" degradation as
// OTA/WebSerial above) — the idle screen shows a placeholder until then.
//
// T44 (2026-08-26): TZ fixed to plain "UTC0", not secrets.h's old
// TIMEZONE_POSIX constant (now unused/removed there) — local time is sourced
// from the PC's own clock instead (utcOffsetMinutes, applied manually in
// updateIdleElement()), so this device's own clock only ever needs to be
// correct UTC, with zero POSIX-TZ/DST logic on this end. Brian no longer
// needs to set/maintain a real TZ string here; the PC-reported offset
// self-corrects if the PC's own timezone ever changes.
void setupTimeSync() {
  if (WiFi.status() != WL_CONNECTED) return;
  configTzTime("UTC0", "pool.ntp.org", "time.nist.gov");
  Serial.println("Time sync started (SNTP, UTC)");
}
