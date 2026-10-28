// debug_console.cpp — Boot reset-reason diagnostic and the WebSerial console (with its test commands).

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include "esp_system.h" // reset-reason diagnostic, see bootResetReason's own comment
#include "main_decls.h"
#include "debug_console.h"

// Reset-reason diagnostic (2026-08-25) — added while investigating the
// recurring "OTA/espota wedged, needs a physical power cycle" pattern
// (T18/T25/T26, recurred again on T40's own flash attempt). Leading theory:
// a marginal/shared power supply browning out under OTA's own current spikes
// (flash writes draw noticeably more current, worsened by this board's
// PSRAM-in-OPI-mode + always-on display + WiFi all sharing one supply; a
// reboot's WiFi-reconnect immediately afterward draws its own separate
// spike) — corroborated by Brian's own observation that another device
// sharing the same USB hub was also visibly power-starved (charging very
// slowly). Captured once at boot, read from setup() before anything else
// runs and printed both immediately (Serial, for a USB-serial-connected
// session) and on every periodic status line (Serial+WebSerial, since this
// board normally has no serial access once deployed — see HARDWARE.md's
// "Debugging without serial access" note). If this ever reads
// ESP_RST_BROWNOUT right after a future wedge, that's a direct confirmation
// instead of a theory; consistently reading something else (e.g. ESP_RST_SW,
// meaning a normal ESP.restart()) would argue against this theory instead.
esp_reset_reason_t bootResetReason;

const char *resetReasonStr(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT_PIN";
    case ESP_RST_SW:        return "SW (ESP.restart())";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "OTHER_WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP_WAKE";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
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
  WebSerial.onMessage([](const String &msg) {
    Serial.printf("WebSerial recv: %s\n", msg.c_str());
    String m = msg;
    m.trim();
    if (m == "buzzsweep") {
      startBuzzSweep();
    } else if (m == "buzz") {
      startBuzzPattern(BUZZ_PATTERN_NEEDS_INPUT,
                       sizeof(BUZZ_PATTERN_NEEDS_INPUT) / sizeof(BUZZ_PATTERN_NEEDS_INPUT[0]));
    } else if (m.startsWith("buzz ")) {
      long f = m.substring(5).toInt();
      if (f >= 100 && f <= 10000) {
        buzzSpotTest((uint16_t)f);
      } else {
        WebSerial.println("buzz: freq must be 100-10000 Hz");
      }
    } else if (m.startsWith("snd ")) {
      // T98 (2026-09-02) — TEMP beep-vocab audition bench. Strip with the rest
      // of the buzzer test instrumentation once the vocabulary is locked.
      String a = m.substring(4);
      a.trim();
      if (a == "complete") {
        BuzzStep s[] = {{BUZZ_FREQ_HZ, 70, 60}, {BUZZ_FREQ_HZ, 70, 0}};
        playSndTest(s, 2);
        WebSerial.println("snd: complete  70/60/70");
      } else if (a == "needsinput") {
        BuzzStep s[] = {{BUZZ_FREQ_HZ, 160, 110}, {BUZZ_FREQ_HZ, 160, 0}};
        playSndTest(s, 2);
        WebSerial.println("snd: needsinput  160/110/160");
      } else if (a == "error") {
        BuzzStep s[] = {{BUZZ_FREQ_HZ, 90, 70}, {BUZZ_FREQ_HZ, 90, 70},
                        {BUZZ_FREQ_HZ, 90, 70}, {BUZZ_FREQ_HZ, 90, 0}};
        playSndTest(s, 4);
        WebSerial.println("snd: error  90/70 x4");
      } else if (a == "renag") {
        BuzzStep s[] = {{BUZZ_FREQ_HZ, 160, 0}};
        playSndTest(s, 1);
        WebSerial.println("snd: renag  160");
      } else if (a.startsWith("raw ")) {
        String list = a.substring(4);
        long vals[SND_TEST_MAX_STEPS * 2];
        int n = 0;
        int start = 0;
        while (start <= (int)list.length() && n < (int)(SND_TEST_MAX_STEPS * 2)) {
          int comma = list.indexOf(',', start);
          String tok = (comma < 0) ? list.substring(start) : list.substring(start, comma);
          tok.trim();
          if (tok.length() > 0) {
            long v = tok.toInt();
            if (v < 0) v = 0;
            if (v > 5000) v = 5000;
            vals[n++] = v;
          }
          if (comma < 0) break;
          start = comma + 1;
        }
        if (n == 0) {
          WebSerial.println("snd raw: need a comma list of ms, e.g. snd raw 70,60,70");
        } else {
          size_t steps = (n + 1) / 2;
          if (steps > SND_TEST_MAX_STEPS) steps = SND_TEST_MAX_STEPS;
          BuzzStep s[SND_TEST_MAX_STEPS];
          for (size_t i = 0; i < steps; i++) {
            s[i].freqHz = BUZZ_FREQ_HZ;
            s[i].onMs = (uint16_t)vals[2 * i];
            s[i].offMs = (2 * i + 1 < n) ? (uint16_t)vals[2 * i + 1] : 0;
          }
          playSndTest(s, steps);
          WebSerial.printf("snd raw: %d step(s)\n", (int)steps);
        }
      } else {
        WebSerial.println("snd: complete | needsinput | error | renag | raw <ms,ms,...>");
      }
    }
  });
  webServer.begin();
  Serial.println("WebSerial ready at /webserial");
}
