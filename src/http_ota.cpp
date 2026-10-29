// http_ota.cpp — HTTP firmware upload, /restart and temporary /debug endpoints.

#include <Arduino.h>
#include <WiFi.h>
#include <Update.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <time.h> // T40: struct tm/strftime for the idle screen's clock
#include "secrets.h"
#include "display.h"
#include "debug_console.h"
#include "pc_state.h"
#include "agent_status.h"
#include "buzzer.h"
#include "screen_nav.h"
#include "screen_home.h"
#include "screen_idle.h"
#include "http_ota.h"

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
// Shared by both a successful HTTP-OTA upload and the /restart endpoint
// (T26) below — same "reboot from loop() after a grace period" mechanism
// either way, not OTA-specific despite the name history.
bool rebootPending = false;
uint32_t rebootAtMs = 0;

// T95 (2026-09-02) — temporary buzzer-path diagnostics. `/debug/buzz` sets
// buzzTestPending so loop() (never the AsyncTCP task — tone()/LEDC must not
// run there, same reason /restart defers ESP.restart()) fires a pattern on
// demand, isolating the hardware + startBuzzPattern() path from the
// agent-status trigger logic. `/debug/agent` is a read-only curl-able dump
// of the knob's own view of agent state. Remove both once T95 is closed.
volatile bool buzzTestPending = false;
volatile int buzzTestPattern = 0; // 0=needs-input, 1=error, 2=done
// Tracks the in-flight upload's success across chunk calls — a single
// global is fine, this endpoint only ever serves one upload at a time
// (dev tool, not a public multi-client service). Real bug found 2026-08-21:
// without this, a failed Update.begin() (e.g. a prior upload's session
// never cleanly finished) was silently ignored and every subsequent chunk
// got written anyway, which corrupted state badly enough that later upload
// attempts failed early (~64KB in) with the connection just dropping.
bool httpOtaUploadOk = false;

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
        bool ok = httpOtaUploadOk && !Update.hasError();
        request->send(ok ? 200 : 500, "text/plain", ok ? "OK" : "FAIL");
        // Restore whatever brightness applied before this upload dimmed it
        // (see the upload handler's own comment on why it dims) — respects
        // displayDimmed rather than assuming NORMAL, so this doesn't fight
        // the T40 idle-state machine if an upload happened to start while
        // already idle. Harmless in the success case too (board reboots
        // moments later anyway, resetting brightness fresh at boot).
        panel->setBrightness(displayDimmed ? DISPLAY_BRIGHTNESS_IDLE : DISPLAY_BRIGHTNESS_NORMAL);
        if (ok) {
          // Reboot from loop() after a short grace period, not inline here
          // — give the AsyncTCP task time to actually flush this response
          // to the socket before ESP.restart() tears everything down.
          rebootPending = true;
          rebootAtMs = millis() + 500;
        }
      },
      [](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final) {
        if (!index) {
          httpOtaUploadOk = false; // reset for this upload; only set true once begin() actually succeeds
          if (!request->authenticate("ota", OTA_PASSWORD)) {
            Serial.println("HTTP OTA: rejected, bad auth");
            return; // main handler above still runs and sends 401 once the (now-ignored) body finishes arriving
          }
          // 2026-08-25: dim for the duration of the write. Originally added
          // to mitigate a suspected power-brownout theory for this project's
          // recurring OTA wedge — that theory didn't hold up (a proper
          // dedicated 2A supply didn't fix it; the actual cause turned out to
          // be per-attempt device-side state, not power — see
          // bootResetReason's own comment and PROGRESS.md's T40 entry for the
          // full story). Left in regardless: flash writes still genuinely
          // draw more current than normal operation, so cutting the
          // display's own draw during that window is harmless and still a
          // reasonable margin to keep, even though it wasn't the fix this
          // particular wedge needed.
          panel->setBrightness(DISPLAY_BRIGHTNESS_OTA_DIM);
          Serial.printf("HTTP OTA: start, filename=%s\n", filename.c_str());
          httpOtaUploadOk = Update.begin(UPDATE_SIZE_UNKNOWN);
          if (!httpOtaUploadOk) Update.printError(Serial);
        }
        if (!request->authenticate("ota", OTA_PASSWORD)) return; // keep rejecting every chunk of a bad-auth upload
        if (!httpOtaUploadOk) return; // begin() already failed (or a prior chunk's write/end did) — stop touching Update
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
            httpOtaUploadOk = false;
          }
        }
      });

  Serial.println("HTTP OTA ready at /firmware-update");

  // T26 (2026-08-21): OTA getting wedged badly enough to need a physical
  // power cycle has now happened twice (T18, and again on T25) — both times
  // with this same AsyncWebServer (plain HTTP, port 80) still answering
  // fine throughout (confirmed via curl during the T25 wedge: espota and
  // /firmware-update both failing, but a plain GET to "/" kept returning
  // its normal 404 the whole time). So a GET route on this same server is a
  // real remote-recovery path, not just a hopeful guess. Homelab-LAN-only,
  // low-stakes — reuses OTA_PASSWORD via HTTP Basic Auth rather than a new
  // secret or scheme, same call as /firmware-update above (a query-param
  // shared secret would work just as well here, but a second auth style for
  // one more admin endpoint isn't worth the inconsistency).
  webServer.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->authenticate("ota", OTA_PASSWORD)) {
      return request->requestAuthentication();
    }
    request->send(200, "text/plain", "restarting");
    rebootPending = true;
    rebootAtMs = millis() + 500; // let the response flush before ESP.restart() tears the connection down
  });
  Serial.println("Remote restart ready at /restart");

  // T44 (2026-08-26): read-only diagnostic — confirms utcOffsetMinutes/
  // timeZoneName and this device's own computed local time remotely (e.g.
  // `curl` over SSH) without needing physical access to the round panel or a
  // WebSerial browser session. Same auth/host reasoning as /restart above.
  // getIdleLocalTime() is the exact same computation updateIdleElement()
  // uses for the idle screen itself, so this reads what the panel would
  // actually show, not a separate guess.
  webServer.on("/debug/time", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->authenticate("ota", OTA_PASSWORD)) {
      return request->requestAuthentication();
    }
    struct tm localTm;
    char body[192];
    if (getIdleLocalTime(&localTm)) {
      char localStr[48];
      strftime(localStr, sizeof(localStr), "%I:%M:%S %p %a %b %d %Y", &localTm);
      snprintf(body, sizeof(body), "utcOffsetMinutes=%d timeZoneName=\"%s\" localTime=\"%s\"\n", utcOffsetMinutes,
               timeZoneName.c_str(), localStr);
    } else {
      snprintf(body, sizeof(body), "utcOffsetMinutes=%d timeZoneName=\"%s\" localTime=unsynced (SNTP hasn't landed yet)\n",
               utcOffsetMinutes, timeZoneName.c_str());
    }
    request->send(200, "text/plain", body);
  });
  Serial.println("Debug time readout ready at /debug/time");

  // T95 (2026-09-02, temporary) — buzzer-path diagnostics, same auth as
  // /restart. Remove once T95 is closed.
  webServer.on("/debug/buzz", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->authenticate("ota", OTA_PASSWORD)) return request->requestAuthentication();
    int p = 0;
    if (request->hasParam("pattern")) {
      String v = request->getParam("pattern")->value();
      if (v == "error") p = 1;
      else if (v == "done") p = 2;
    }
    buzzTestPattern = p;
    buzzTestPending = true;
    request->send(200, "text/plain", String("buzz queued pattern=") + p + " buzzerEnabled=" + (buzzerEnabled ? 1 : 0) + "\n");
  });
  webServer.on("/debug/agent", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->authenticate("ota", OTA_PASSWORD)) return request->requestAuthentication();
    char body[640];
    uint32_t feedAgeMs = millis() - lastAgentPollOkMs;
    snprintf(body, sizeof(body),
             "agentAggregateStatus=%d agentAttentionCount=%d buzzerEnabled=%d buzzPlaying=%d "
             "attentionRenagMinutes=%u lastAttentionChimeMs=%u pcConnected=%d millis=%u\n"
             "ring: rendered=%d (0=hidden 1=working 2=attention) feedAgeMs=%u feedStaleMs=%u\n"
             "agentGet: code=%d bodyLen=%u parseErr=%s\n"
             "sessions: %s\n",
             (int)agentAggregateStatus, agentAttentionCount, buzzerEnabled ? 1 : 0, buzzPlaying ? 1 : 0,
             attentionRenagMinutes, lastAttentionChimeMs, pcConnected ? 1 : 0, (unsigned)millis(),
             agentRingRendered, (unsigned)feedAgeMs, (unsigned)AGENT_FEED_STALE_MS,
             dbgAgentGetCode, (unsigned)dbgAgentBodyLen, dbgAgentParseErr,
             dbgAgentSessions[0] ? dbgAgentSessions : "(none captured yet)");
    request->send(200, "text/plain", body);
  });
  Serial.println("Debug buzzer/agent readouts ready at /debug/buzz and /debug/agent");
}
