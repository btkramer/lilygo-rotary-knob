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
#include <time.h> // T40: struct tm/strftime for the idle screen's clock
#include "esp_system.h" // reset-reason diagnostic, see bootResetReason's own comment
#include "USB.h"
#include "USBHIDConsumerControl.h"
#include "secrets.h"
#include "pin_config.h"
#include "hid_media.h"
#include "input.h"
#include "display.h"
#include "debug_console.h"
#include "pc_state.h"
#include "agent_status.h"
#include "buzzer.h"
#include "screen_nav.h"
#include "screen_home.h"
#include "screen_idle.h"
#include "screen_volume.h"
#include "screen_mute.h"
#include "screen_now_playing.h"
#include "screen_quick_jump.h"
#include "hub_sync.h"
#include "main_decls.h"

constexpr uint32_t DEBOUNCE_MS = 30;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;

Preferences syncPrefs; // only ever touched from syncPollTask post-refactor — see setupSyncSettings()'s comment

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
  dimDelayMs = syncPrefs.getUInt("dimDelayMs", DIM_DELAY_MS_DEFAULT);
  utcOffsetMinutes = syncPrefs.getInt("utcOffMin", 0); // T44 — 0 (plain UTC) until a real value has ever synced
  buzzerEnabled = syncPrefs.getBool("buzzerEn", true); // T60 — on by default until a real value has ever synced
  attentionRenagMinutes = syncPrefs.getUInt("renagMin", ATTENTION_RENAG_MINUTES_DEFAULT); // T89
  Serial.printf("sync: loaded from NVS: volumeFuseMs=%u idleToHomeMs=%u dimDelayMs=%u utcOffsetMinutes=%d buzzerEnabled=%d attentionRenagMinutes=%u\n",
                volumeFuseMs, idleToHomeMs, dimDelayMs, utcOffsetMinutes, buzzerEnabled, attentionRenagMinutes);

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

void setup() {
  Serial.begin(115200);
  // First thing, before anything else has a chance to run — see
  // bootResetReason's own comment for why this exists.
  bootResetReason = esp_reset_reason();
  Serial.printf("Boot reset reason: %s\n", resetReasonStr(bootResetReason));
  setupSyncSettings(); // NVS load, no WiFi needed — operator-adjusted timeouts apply even if this boot never reaches pc-dash-hub
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_KEY, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderIsr, CHANGE);
  pinMode(PIN_BUZZER, OUTPUT); // T60 — tone()/noTone() drive this via LEDC; explicit pinMode for clarity, matches this project's style

  ConsumerControl.begin();
  USB.begin();

  setupDisplay();
  setupTouch();
  setupLVGL();
  setupOTA();
  setupTimeSync(); // T40 — needs WiFi, same as WebSerial/HTTP OTA below
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
  serviceBuzzer(); // T60 — non-blocking pattern stepper, must run every pass regardless of what else loop() is doing
  serviceBuzzSweep(); // temp — buzzer resonance sweep driven by the `buzzsweep` WebSerial command; remove with the rest once BUZZ_FREQ_HZ is tuned

  // T95 (2026-09-02, temporary) — service /debug/buzz. Deferred here off the
  // AsyncTCP task, same reason as /restart. Remove once T95 is closed.
  if (buzzTestPending) {
    buzzTestPending = false;
    const BuzzStep *pat = buzzTestPattern == 1 ? BUZZ_PATTERN_ERROR
                        : buzzTestPattern == 2 ? BUZZ_PATTERN_DONE
                                               : BUZZ_PATTERN_NEEDS_INPUT;
    size_t plen = buzzTestPattern == 1 ? sizeof(BUZZ_PATTERN_ERROR) / sizeof(BUZZ_PATTERN_ERROR[0])
                : buzzTestPattern == 2 ? sizeof(BUZZ_PATTERN_DONE) / sizeof(BUZZ_PATTERN_DONE[0])
                                       : sizeof(BUZZ_PATTERN_NEEDS_INPUT) / sizeof(BUZZ_PATTERN_NEEDS_INPUT[0]);
    Serial.printf("buzzTest: pattern=%d buzzerEnabled=%d buzzSweepActive=%d\n", buzzTestPattern, buzzerEnabled, buzzSweepActive);
    WebSerial.printf("buzzTest: pattern=%d buzzerEnabled=%d buzzSweepActive=%d\n", buzzTestPattern, buzzerEnabled, buzzSweepActive);
    startBuzzPattern(pat, plen);
  }

  // T90/T89 (2026-09-02): re-nag — while the agent aggregate still needs a
  // human and the PC is still reachable, repeat the attention chime every
  // attentionRenagMinutes (0 = disabled). The entry chime sets the anchor
  // (sync-apply block above); T91's PC-offline path can't reach here since
  // this is pcConnected-gated, and the aggregate reverting to working/idle
  // stops it naturally.
  if (pcConnected && attentionRenagMinutes > 0 &&
      (agentAggregateStatus == AGENT_STATUS_NEEDS_INPUT || agentAggregateStatus == AGENT_STATUS_ERROR) &&
      (millis() - lastAttentionChimeMs) >= attentionRenagMinutes * 60000UL) {
    startAttentionChime();
    lastAttentionChimeMs = millis();
  }

  // A successful HTTP-OTA upload or a /restart request (T26, set from the
  // AsyncTCP task — see setupHttpOta()'s comment) — reboot from here, not
  // inline in either handler, so the AsyncTCP task gets a chance to actually
  // flush the HTTP response to the socket first.
  if (rebootPending && (int32_t)(millis() - rebootAtMs) >= 0) {
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
    // T99 #3 — hoisted: the agent-apply block below runs independently of
    // result.ok (the pcState GET's success), so it needs the pre-update
    // pcConnected value too, for T91's offline-edge check.
    bool wasPcConnected = pcConnected;
    if (result.ok) {
      pcConnected = result.pcConnected;
      lastInputSecondsAgo = result.lastInputSecondsAgo; // T76
      lastSeenSecondsAgo = result.lastSeenSecondsAgo;
      volumeFuseMs = result.volumeFuseMs;
      idleToHomeMs = result.idleToHomeMs;
      dimDelayMs = result.dimDelayMs;
      // T48: result.utcOffsetMinutes only actually changes on a real sync
      // trigger (pollSyncState()'s own gate) — most polls it's just the
      // already-applied value echoed straight through, so this assignment
      // is a harmless no-op the rest of the time, not a fresh overwrite.
      utcOffsetMinutes = result.utcOffsetMinutes;
      if (!result.timeZoneName.isEmpty()) timeZoneName = result.timeZoneName; // cosmetic only; don't blank it out on a transient miss
      buzzerEnabled = result.buzzerEnabled;
      attentionRenagMinutes = result.attentionRenagMinutes; // T89

      // T99 #3: agent-status handling moved out of this result.ok block — see
      // the `if (result.agentPollOk)` block right after it. The agent-sessions
      // poll succeeds or fails independently of the pcState GET, so its
      // results must be applied even on a pcState miss.

      // Volume/mute/now-playing: only trust the payload once the companion
      // app has actually reported in on *this* poll (result.pcConnected) —
      // see ECOSYSTEM_PLAN.md's "Volume/mute/media reconciliation" section.
      // While disconnected, the local optimistic estimate (set by knob
      // rotation / button press / tap) is left uncorrected rather than
      // being overwritten by the hub's never-set defaults.
      bool pcDataChanged = false;
      if (pcConnected) {
        // T28/T29 (2026-08-21): volumePercent/speakerMuted/nowPlayingIsPlaying
        // are local optimistic guesses (knob rotation / mute button / tap —
        // see each write site's own comment). Hold each one against a poll
        // that still disagrees with it — see LOCAL_OPTIMISTIC_BACKSTOP_MS's
        // own comment for why this waits for actual agreement rather than
        // expiring after a fixed delay. A poll that already agrees applies
        // immediately (real confirmation, whenever it actually arrives); one
        // that still disagrees is only trusted once the backstop elapses.
        uint32_t nowMs = millis();
        bool volumeShouldHold = volumePercent != result.volumePercent &&
                                 (nowMs - lastVolumeLocalWriteMs) < LOCAL_OPTIMISTIC_BACKSTOP_MS;
        bool muteShouldHold = speakerMuted != result.speakerMuted &&
                               (nowMs - lastMuteLocalWriteMs) < LOCAL_OPTIMISTIC_BACKSTOP_MS;
        bool playPauseShouldHold = nowPlayingIsPlaying != result.nowPlayingIsPlaying &&
                                    (nowMs - lastPlayPauseLocalWriteMs) < LOCAL_OPTIMISTIC_BACKSTOP_MS;

        // T29 (2026-08-21): position/duration dropped out of this check —
        // they're no longer applied from every poll at all (see below), so
        // comparing them here would force a full redraw ~every ~1s poll
        // purely from ordinary playback progress, which is exactly the
        // per-poll churn this device's own local position tick (loop(),
        // below) exists to avoid.
        pcDataChanged = (!volumeShouldHold && volumePercent != result.volumePercent) ||
                         (!muteShouldHold && speakerMuted != result.speakerMuted) ||
                         (!playPauseShouldHold && nowPlayingIsPlaying != result.nowPlayingIsPlaying) ||
                         nowPlayingArtist != result.nowPlayingArtist ||
                         nowPlayingTrack != result.nowPlayingTrack ||
                         nowPlayingSource != result.nowPlayingSource;
        if (!volumeShouldHold) volumePercent = result.volumePercent;
        if (!muteShouldHold) speakerMuted = result.speakerMuted;
        if (!playPauseShouldHold) {
          bool playPauseChanged = nowPlayingIsPlaying != result.nowPlayingIsPlaying;
          nowPlayingIsPlaying = result.nowPlayingIsPlaying;
          // Only resync the tick anchor on an actual transition — this branch
          // also runs every poll once already-confirmed (should-hold false
          // because they already agree), and resetting the anchor on every
          // one of those would starve the local tick entirely (it'd never
          // see 1000ms of unbroken elapsed time between resets).
          if (playPauseChanged) lastPositionTickMs = nowMs;
        }
        bool trackChanged = nowPlayingArtist != result.nowPlayingArtist ||
                             nowPlayingTrack != result.nowPlayingTrack ||
                             nowPlayingSource != result.nowPlayingSource;
        nowPlayingArtist = result.nowPlayingArtist;
        nowPlayingTrack = result.nowPlayingTrack;
        nowPlayingSource = result.nowPlayingSource;

        // Reconcile the locally-ticking position/duration against the poll's
        // real values (Brian's design, 2026-08-21) — see this field's own
        // top-of-file comment for the full reasoning. Applied immediately on
        // the connect transition or a track change (no reason to wait —
        // there's no "might just be slow to confirm" ambiguity in either
        // case, unlike play/pause), otherwise only once the poll has
        // disagreed with the local clock by more than the tolerance on
        // POSITION_DRIFT_CONFIRM_COUNT consecutive polls, so one noisy poll
        // can't visibly yank the display.
        bool justConnected = !wasPcConnected;
        int32_t positionDiffMs = (int32_t)result.nowPlayingPositionMs - (int32_t)nowPlayingPositionMs;
        bool positionDrifted = (uint32_t)abs(positionDiffMs) > POSITION_DRIFT_TOLERANCE_MS;
        positionDriftStreak = positionDrifted ? positionDriftStreak + 1 : 0;
        if (justConnected || trackChanged || positionDriftStreak >= POSITION_DRIFT_CONFIRM_COUNT) {
          nowPlayingPositionMs = result.nowPlayingPositionMs;
          nowPlayingDurationMs = result.nowPlayingDurationMs;
          lastPositionTickMs = nowMs;
          positionDriftStreak = 0;
          nowPlayingProgressPercent = nowPlayingDurationMs > 0
              ? constrain((int)(((uint64_t)nowPlayingPositionMs * 100) / nowPlayingDurationMs), 0, 100)
              : 0;
          formatNowPlayingElapsed(nowPlayingPositionMs, nowPlayingDurationMs); // T22: "current/total"
          pcDataChanged = true; // make a real drift correction visible immediately, not on the next unrelated redraw
        }
        updateVolumeScreen();
        updateSpeakerMuteScreen();
      }

      if (pcConnected != wasPcConnected || pcDataChanged) {
        updateHomeScreen();
        updateNowPlayingScreen();
      }
    }

    // T60/T99 agent-monitor apply — runs whenever a fresh agent-sessions
    // snapshot landed (result.agentPollOk), INDEPENDENT of result.ok (#3): the
    // agent feed is pc-dash-app's hook forwarder (T58), a separate signal and
    // a separate request from the pcState GET, so a pcState miss must not
    // freeze agent status. Badge/ring reflect the latest aggregate; the
    // completion chime is per-session (#1); the attention chime fires on the
    // aggregate's edge into needs-input/error.
    if (result.agentPollOk) {
      lastAgentPollOkMs = millis(); // T99 #2 — feeds updateAgentAttentionRing()'s freshness gate
      bool wasAttention = (agentAggregateStatus == AGENT_STATUS_NEEDS_INPUT || agentAggregateStatus == AGENT_STATUS_ERROR);
      bool agentStatusChanged = (agentAggregateStatus != result.agentAggregateStatus);
      agentAggregateStatus = result.agentAggregateStatus;
      agentAttentionCount = result.agentAttentionCount;
      // T91 (2026-09-02): PC just went offline — silence any repeating
      // attention chime and drop to sleep (T92). The red attention ring is
      // hidden by updateAgentAttentionRing()'s own !pcConnected gate; the
      // orange working ring is NOT pcConnected-gated (T99 #2) but its own
      // freshness gate takes it down once the forwarder stops reporting.
      if (wasPcConnected && !pcConnected) {
        stopBuzzPattern();
        uint32_t nowMs = millis();
        lastAnyInteractionMs = (nowMs > dimDelayMs) ? (nowMs - dimDelayMs) : 0;
      }
      if (agentStatusChanged || pcConnected != wasPcConnected) {
        updateAgentStatusBadge();
        updateAgentAttentionRing();
        Serial.printf("agentStatus: newAgg=%d wasAttention=%d pcConnected %d->%d doneFlag=%d pcStateOk=%d\n",
                      (int)agentAggregateStatus, wasAttention, wasPcConnected, pcConnected, result.agentDoneChime, result.ok);
        WebSerial.printf("agentStatus: newAgg=%d wasAttention=%d pcConnected %d->%d doneFlag=%d pcStateOk=%d\n",
                         (int)agentAggregateStatus, wasAttention, wasPcConnected, pcConnected, result.agentDoneChime, result.ok);
      }
      bool isAttention = (agentAggregateStatus == AGENT_STATUS_NEEDS_INPUT || agentAggregateStatus == AGENT_STATUS_ERROR);
      // T99 #1: completion chime is driven purely by the per-session
      // working->done/idle edge detected in pollAgentSessions() — fires even
      // while other sessions (an always-on coordinator) stay working, which
      // the old aggregate-transition trigger structurally could not.
      // Suppressed while the aggregate needs a human, so a finish never steps
      // on an active needs-input chime. Two-tier (2026-09-03, Brian-approved
      // via the T98 audition bench): PARTIAL when another session is still
      // working, ALL_CLEAR when this was the last one — pollAgentSessions()
      // decides which (mutually exclusive) from the same poll's snapshot.
      bool partialFinished = (!isAttention && result.agentDoneChime);
      bool allClearFinished = (!isAttention && result.agentAllClearChime);
      if (!wasAttention && isAttention) {
        triggerAgentFlash(agentStatusColor(agentAggregateStatus));
        startAttentionChime();
        lastAttentionChimeMs = millis();
        Serial.printf("agentChime: ENTRY status=%d buzzerEnabled=%d\n", (int)agentAggregateStatus, buzzerEnabled);
        WebSerial.printf("agentChime: ENTRY status=%d buzzerEnabled=%d\n", (int)agentAggregateStatus, buzzerEnabled);
      } else if (allClearFinished) {
        startBuzzPattern(BUZZ_PATTERN_ALL_CLEAR, sizeof(BUZZ_PATTERN_ALL_CLEAR) / sizeof(BUZZ_PATTERN_ALL_CLEAR[0]));
        Serial.printf("agentChime: ALL_CLEAR newStatus=%d buzzerEnabled=%d\n", (int)agentAggregateStatus, buzzerEnabled);
        WebSerial.printf("agentChime: ALL_CLEAR newStatus=%d buzzerEnabled=%d\n", (int)agentAggregateStatus, buzzerEnabled);
      } else if (partialFinished) {
        startBuzzPattern(BUZZ_PATTERN_DONE, sizeof(BUZZ_PATTERN_DONE) / sizeof(BUZZ_PATTERN_DONE[0]));
        Serial.printf("agentChime: DONE (partial, per-session) newStatus=%d buzzerEnabled=%d\n", (int)agentAggregateStatus, buzzerEnabled);
        WebSerial.printf("agentChime: DONE (partial, per-session) newStatus=%d buzzerEnabled=%d\n", (int)agentAggregateStatus, buzzerEnabled);
      }
    }
  }

  // T99 #7 (2026-09-03): re-assert the attention/working ring on a fixed
  // cadence, not only on a sync-apply edge. A WORKING aggregate that settled
  // during a gap where the edge check didn't fire — or a feed that has just
  // gone stale (AGENT_FEED_STALE_MS) — still needs the ring to catch up.
  // updateAgentAttentionRing() is idempotent (its agentRingRendered guard), so
  // calling it on every eligible pass is free when nothing has changed.
  static uint32_t lastRingRefreshMs = 0;
  if (millis() - lastRingRefreshMs >= 1000) {
    lastRingRefreshMs = millis();
    updateAgentAttentionRing();
  }

  // T29 (2026-08-21): advance the displayed elapsed time by one real second
  // at a time while actually playing — independent of poll cadence entirely,
  // unlike the block above. Frozen (anchor reset to "now", no backlog
  // accrued) whenever not actually advancing — paused or disconnected — so
  // resuming playback later never bursts forward by however long it was
  // paused.
  //
  // Correction (2026-08-21): the first version of this also required
  // nowPlayingDurationMs > 0 to tick at all, on top of pcConnected/
  // nowPlayingIsPlaying — Brian reported the local clock not visibly running
  // at all (only the ~4s reconcile snap moved anything), which durationMs
  // legitimately being 0 for some sources (not every SMTC session reports a
  // track length) would fully explain: current position should still count
  // up even when the total is unknown, per the original ask, so gate only on
  // the two fields that actually mean "is real playback progressing" —
  // duration now only affects the cap/percent, both already guarded for 0
  // separately.
  //
  // Second correction (2026-08-21): Brian then reported the local clock
  // running slow — drifting a few seconds behind real elapsed time between
  // reconciles. The peer theory was a classic anchor-reset-to-wall-clock bug,
  // but that's not what this does (it accumulates `lastPositionTickMs +=
  // POSITION_TICK_MS`, not `= millis()`, precisely so a late tick doesn't
  // lose the delay). The actual gap: this used to do only *one* tick per
  // loop() iteration, on the assumption loop() runs far more often than once
  // a second so falling behind would catch up within a handful of
  // iterations — an assumption that isn't safe to lean on given loop() also
  // does LVGL rendering, HID, and OTA/WebSerial handling every pass and can
  // genuinely run slower than 1Hz at times. Switched to a bounded `while`
  // that catches up as many whole ticks as have actually elapsed in one
  // pass, so real elapsed time is what's tracked regardless of how often
  // loop() itself gets to check — no assumption about loop()'s own speed
  // required anymore. Redraws once per loop() pass (not once per caught-up
  // tick) to avoid redundant LVGL work when catching up several at once.
  if (pcConnected && nowPlayingIsPlaying) {
    bool ticked = false;
    while (millis() - lastPositionTickMs >= POSITION_TICK_MS &&
           (nowPlayingDurationMs == 0 || nowPlayingPositionMs < nowPlayingDurationMs)) {
      lastPositionTickMs += POSITION_TICK_MS;
      nowPlayingPositionMs = nowPlayingDurationMs > 0
          ? min(nowPlayingPositionMs + POSITION_TICK_MS, nowPlayingDurationMs)
          : nowPlayingPositionMs + POSITION_TICK_MS;
      ticked = true;
    }
    if (ticked) {
      nowPlayingProgressPercent = nowPlayingDurationMs > 0
          ? constrain((int)(((uint64_t)nowPlayingPositionMs * 100) / nowPlayingDurationMs), 0, 100)
          : 0;
      formatNowPlayingElapsed(nowPlayingPositionMs, nowPlayingDurationMs);
      updateNowPlayingScreen();
    }
  } else {
    lastPositionTickMs = millis();
  }

  // Debug aid (2026-08-21): prints the tick gate's own inputs every 5s,
  // including a real timestamp and how many loop() iterations happened in
  // that window, so a WebSerial/serial capture during a live test can show
  // definitively both what's blocking the tick (if anything) and whether
  // loop() itself is genuinely running slower than assumed.
  static uint32_t lastPositionTickDebugMs = 0;
  static uint32_t loopIterCount = 0;
  loopIterCount++;
  if (millis() - lastPositionTickDebugMs >= 5000) {
    uint32_t nowDebugMs = millis();
    uint32_t windowMs = nowDebugMs - lastPositionTickDebugMs;
    lastPositionTickDebugMs = nowDebugMs;
    Serial.printf("posTick: t=%ums pcConnected=%d playing=%d pos=%ums dur=%ums loopIters=%u/%ums\n",
                  nowDebugMs, pcConnected, nowPlayingIsPlaying, nowPlayingPositionMs, nowPlayingDurationMs,
                  loopIterCount, windowMs);
    WebSerial.printf("posTick: t=%ums pcConnected=%d playing=%d pos=%ums dur=%ums loopIters=%u/%ums\n",
                      nowDebugMs, pcConnected, nowPlayingIsPlaying, nowPlayingPositionMs, nowPlayingDurationMs,
                      loopIterCount, windowMs);
    loopIterCount = 0;
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
    // bootResetReason: see its own top-of-file comment — repeated on every
    // status line (not just once at boot) since this board normally has no
    // serial access once deployed; a WebSerial viewer connecting well after
    // boot still needs to see how this boot actually started.
    // freeHeap (2026-08-25): added while reopening the OTA-wedge
    // investigation — Brian pointed out he's seen failures shortly after a
    // restart too, which undercuts "just needs a clean boot" as the *whole*
    // explanation (a fresh-restart-immediately-before-flashing did fix T40's
    // own wedge this session — see PROGRESS.md — but that doesn't rule out
    // some slower-building resource pressure, e.g. heap fragmentation,
    // separately affecting things over longer uptime). This alone doesn't
    // diagnose anything by itself; it just starts accumulating real
    // heap-over-time data on every boot going forward, so a future
    // WebSerial capture can actually show whether free heap trends downward
    // over hours/days of normal operation rather than staying flat.
    Serial.printf("status: wifi=%s display=%s touch=%s ip=%s syncOk=%s pcConnected=%s lastSeen=%us resetReason=%s freeHeap=%u\n",
                   WiFi.status() == WL_CONNECTED ? "up" : "down",
                   displayOk ? "ok" : "FAIL",
                   touchOk ? "ok" : "FAIL",
                   WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "-",
                   lastSyncPollOk ? "yes" : "no",
                   pcConnected ? "yes" : "no",
                   lastSeenSecondsAgo,
                   resetReasonStr(bootResetReason),
                   (unsigned)ESP.getFreeHeap());
    WebSerial.printf("status: wifi=%s display=%s touch=%s ip=%s syncOk=%s pcConnected=%s lastSeen=%us resetReason=%s freeHeap=%u\n",
                      WiFi.status() == WL_CONNECTED ? "up" : "down",
                      displayOk ? "ok" : "FAIL",
                      touchOk ? "ok" : "FAIL",
                      WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "-",
                      lastSyncPollOk ? "yes" : "no",
                      pcConnected ? "yes" : "no",
                      lastSeenSecondsAgo,
                      resetReasonStr(bootResetReason),
                      (unsigned)ESP.getFreeHeap());
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
  //
  // T34 (Brian, 2026-08-22): while actually playing, keeps the standard
  // idleToHomeMs (30s default, operator-adjustable) — nothing changes there.
  // While paused/stopped, a much shorter fixed 5s takes over instead: once
  // there's nothing playing, lingering on this screen has little value, so
  // it falls back to Home quickly rather than waiting out the same timeout
  // as an actively-playing session. Confirmed with the coordinator this is
  // specifically this screen's own idle fuse, not the nav-cycle or volume
  // overlay's (separate, already-settled T31/T33 timeouts).
  uint32_t nowPlayingIdleMs = nowPlayingIsPlaying ? idleToHomeMs : NOW_PLAYING_STOPPED_IDLE_MS;
  // T35 debug (2026-08-22, temporary): Brian reports this fuse isn't
  // actually switching between the two durations on-device. Throttled
  // (2s) rather than every loop() — only fires while this screen is
  // active, so it's silent everywhere else. Shows the exact inputs
  // (nowPlayingIsPlaying, the resolved threshold, elapsed-since-activity)
  // so a live capture can confirm whether nowPlayingIsPlaying itself is
  // wrong at this point, or the threshold picked from it is. Remove once
  // T35 is root-caused or ruled out.
  if (lv_screen_active() == nowPlayingScreen) {
    static uint32_t lastNowPlayingFuseDebugMs = 0;
    uint32_t nowMs = millis();
    if (nowMs - lastNowPlayingFuseDebugMs >= 2000) {
      lastNowPlayingFuseDebugMs = nowMs;
      Serial.printf("nowPlayingFuse: t=%ums playing=%d idleMs=%u elapsed=%ums\n",
                    nowMs, nowPlayingIsPlaying, nowPlayingIdleMs, nowMs - lastNowPlayingActivityMs);
      WebSerial.printf("nowPlayingFuse: t=%ums playing=%d idleMs=%u elapsed=%ums\n",
                        nowMs, nowPlayingIsPlaying, nowPlayingIdleMs, nowMs - lastNowPlayingActivityMs);
    }
  }
  if (lv_screen_active() == nowPlayingScreen && (millis() - lastNowPlayingActivityMs) > nowPlayingIdleMs) {
    Serial.printf("nowPlayingFuse: FIRED t=%ums playing=%d idleMs=%u\n", millis(), nowPlayingIsPlaying, nowPlayingIdleMs);
    WebSerial.printf("nowPlayingFuse: FIRED t=%ums playing=%d idleMs=%u\n", millis(), nowPlayingIsPlaying, nowPlayingIdleMs);
    lv_screen_load(homeScreen);
  }

  int key = digitalRead(PIN_ENCODER_KEY);
  if (key != lastKey) {
    lastKey = key;
    uint32_t now = millis();
    if (key == LOW && (now - lastKeyMs) > DEBOUNCE_MS) {
      lastKeyMs = now;
      lastAnyInteractionMs = now; // T40: knob press counts as "not idle" — see lastAnyInteractionMs's own comment
      showSpeakerMuteOverlay(); // sends the HID mute pulse itself
      Serial.println("MUTE");
      WebSerial.println("MUTE");
    }
  }

  // T40 (2026-08-25): burn-in-mitigation idle screen — see
  // ECOSYSTEM_PLAN.md's "Burn-in mitigation" section and lastAnyInteractionMs's
  // own top-of-file comment. Deliberately global (checked regardless of which
  // screen is actually active, not gated to only fire from Home) rather than
  // "wait until every other fuse above has already settled on Home first" —
  // dimDelayMs is expected to always be configured longer than every other
  // timeout above, so in practice this only ever fires once everything else
  // has already returned to Home anyway; being unconditional here just means
  // it can't get stuck waiting on some future screen that doesn't have its
  // own idle-to-Home fuse yet, the same gap T35 found in an earlier screen.
  //
  // Driven by displayDimmed (see its own comment), not lv_screen_active() —
  // a rotation or button press that arrives while dimmed is drained by the
  // encoder/key-edge blocks *above* this one in the same loop() pass, and
  // showVolumeOverlay()/showSpeakerMuteOverlay() already jump straight to
  // Volume/Speaker Mute in that case (see their own idleScreen handling), so
  // by the time this runs, lv_screen_active() may already be neither
  // idleScreen nor Home — brightness still needs restoring regardless of
  // which screen that interaction actually landed on.
  // T76: stay awake as long as EITHER signal says "active" — the knob's own
  // local touch/rotation timer (unchanged, above) OR the PC itself having
  // had real input within the last 30 min (pcConnected-gated: an unreached/
  // never-reported hub must not be misread as "PC active", same reasoning
  // as pcConnected's own offline-dim gating elsewhere in this file).
  bool pcRecentlyActive = pcConnected && lastInputSecondsAgo <= PC_RECENT_INPUT_WINDOW_S;
  bool shouldBeIdle = (millis() - lastAnyInteractionMs) > dimDelayMs && !pcRecentlyActive;
  if (shouldBeIdle && !displayDimmed) {
    displayDimmed = true;
    panel->setBrightness(DISPLAY_BRIGHTNESS_IDLE);
    updateIdleScreen(); // set real content before it's ever shown, not a blank/placeholder first frame
    lv_screen_load(idleScreen);
  } else if (!shouldBeIdle && displayDimmed) {
    displayDimmed = false;
    panel->setBrightness(DISPLAY_BRIGHTNESS_NORMAL);
    // Only force Home if idleScreen is still what's actually showing (the
    // touch-wake path — idleScreen has no gesture/click handlers to redirect
    // it anywhere else). A rotation/button-press wake already loaded
    // Volume/Speaker Mute directly above; don't stomp that with a redundant
    // Home load.
    if (lv_screen_active() == idleScreen) lv_screen_load(homeScreen);
  } else if (shouldBeIdle && displayDimmed && lv_screen_active() == idleScreen) {
    updateIdleScreen(); // keep date/time each independently cross-fading to a new position while it stays shown
  }

  delay(5); // matches Arduino_GFX's own LVGL example's loop cadence
}
