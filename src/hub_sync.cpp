// hub_sync.cpp — pc-dash-hub polling task: state/settings/agent-session fetch and handoff to loop().

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "secrets.h"
#include "pc_state.h"
#include "agent_status.h"
#include "buzzer.h"
#include "screen_nav.h"
#include "screen_home.h"
#include "screen_volume.h"
#include "screen_now_playing.h"
#include "hub_sync.h"
#include "settings_store.h"

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
// T25 (Brian, 2026-08-21): dropped from a flat 20s to 1s — his feedback on T20-T23
// was that Now Playing (artist/track/position) lagged real changes by ~10-20s, with
// artist/track and position visibly landing several seconds apart from each other.
// T23 fixed a real redraw-gate bug, but the dominant remaining cause was this
// interval itself: every field in the combined settings+pcState blob (there's no
// per-field split — one endpoint, one poll, one interval) was only ever as fresh as
// the last poll, up to 20s stale. No architecture change needed to fix this — the
// blocking GET already runs on its own FreeRTOS task (syncPollTask, off loop()/LVGL
// entirely, see this section's top comment), so polling 20x more often just means
// 20x more small local-LAN requests, not a new stall risk. Kept as a poll rather than
// switching to a push/WebSocket model — plain HTTP GET is simpler to reason about
// and this cadence already gets within roughly a poll-and-a-half of Brian's ~1s
// "on-change, capped at 1Hz" target end-to-end.
constexpr uint32_t SYNC_POLL_INTERVAL_CONNECTED_MS = 1000;
constexpr uint32_t SYNC_POLL_INTERVAL_RETRY_MS = 5000;      // fast retry while the hub's unreachable — also what gives near-instant resync once it comes back
constexpr uint32_t SYNC_HTTP_TIMEOUT_MS = 4000;

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

// T60 (agent-monitor mode) — separate endpoint (GET /api/agent-sessions,
// T57) from the per-device settings+pcState blob above, polled on its own
// slower cadence (AGENT_SESSIONS_POLL_INTERVAL_MS) rather than doubling
// every 1s pcState poll's network cost: a human noticing/acting on "needs
// your input" doesn't need sub-second freshness the way Now Playing's
// position did (T25/T29). Only ever called from syncPollTask.
constexpr uint32_t AGENT_SESSIONS_POLL_INTERVAL_MS = 3000;
uint32_t lastAgentSessionsPollMs = 0;

// T99 #1 (2026-09-03) — per-session status tracking for the completion chime.
// The aggregate-only trigger this replaces (T87/T95) could not fire while any
// *other* session stayed `working` (e.g. an always-on coordinator session), so
// an individual session finishing produced no chime even though the phone
// (per-session, T93) beeped. Now any tracked session going working -> done/idle
// sets result.agentDoneChime. Fixed array — the many-concurrent-sessions case
// is rare on one desk; overflow just means a late-added session isn't tracked
// for its finish edge (no crash, no spurious chime).
struct AgentSessionSlot {
  char id[40]; // sessionId is a 36-char UUID
  AgentStatus status;
};
AgentSessionSlot agentSessionSlots[16] = {};

// T99 #5 (2026-09-03) — GET + parse the session list, streaming the body
// straight into ArduinoJson instead of buffering it in a String first
// (removes a latent truncation/heap failure mode on larger session lists).
// Falls back to getString() only if Content-Length is absent (chunked). Also
// records dbg* for /debug/agent. Returns true on GET+parse success.
bool agentSessionsFetch(const char *host, JsonDocument &doc) {
  HTTPClient http;
  String url = String("http://") + host + ":" + SYNC_PORT + "/api/agent-sessions";
  http.begin(url);
  http.setConnectTimeout(SYNC_HTTP_TIMEOUT_MS);
  http.setTimeout(SYNC_HTTP_TIMEOUT_MS);
  int httpCode = http.GET();
  dbgAgentGetCode = httpCode;
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("agentSessions: GET %s failed, code=%d\n", host, httpCode);
    http.end();
    return false;
  }
  int contentLen = http.getSize();
  DeserializationError err;
  if (contentLen > 0) {
    dbgAgentBodyLen = (size_t)contentLen;
    err = deserializeJson(doc, http.getStream());
  } else {
    String body = http.getString(); // chunked / unknown-length fallback
    dbgAgentBodyLen = body.length();
    err = deserializeJson(doc, body);
  }
  http.end();
  snprintf(dbgAgentParseErr, sizeof(dbgAgentParseErr), "%s", err.c_str());
  if (err) {
    Serial.printf("agentSessions: JSON parse failed: %s\n", err.c_str());
    return false;
  }
  return true;
}

// Fetch + aggregate the session list into one glanceable status (see
// agentStatusPriority()'s own comment for the aggregation rule) — only ever
// touches `result`, same "stage for loop() to apply, never touch LVGL/real
// globals here" discipline as pollSyncState() itself. Leaves result's
// agent* fields at whatever pollSyncState() seeded them with (the current
// global values) on any failure, so a transient miss doesn't blank the
// badge — same "preserved unless real value" shape as volumeFuseMs etc.
void pollAgentSessions(SyncResult &result) {
  JsonDocument doc;
  if (!agentSessionsFetch(SYNC_HOST, doc) && !agentSessionsFetch(SYNC_FALLBACK_IP, doc)) {
    return;
  }
  result.agentPollOk = true; // T99 #3 — a real, current agent-sessions snapshot landed
  JsonArray sessions = doc.as<JsonArray>(); // response body is a bare array, confirmed via T61's smoke test (200 [])
  AgentStatus worst = AGENT_STATUS_NONE;
  int attentionCount = 0;
  int activeCount = 0;
  bool slotSeen[16] = {false};
  bool anyFinishEdge = false; // T99 two-tier chime — some tracked session went working->done/idle this poll
  bool anyWorkingNow = false; // T99 two-tier chime — is any session (post-update) still working
  String sessList; // T99 instrumentation — "project:status,..." for /debug/agent
  for (JsonObject s : sessions) {
    const char *statusStr = s["status"] | "";
    AgentStatus st = agentStatusFromString(statusStr);

    if (sessList.length() < 200) {
      if (sessList.length()) sessList += ",";
      sessList += (const char *)(s["project"] | "?");
      sessList += ":";
      sessList += statusStr[0] ? statusStr : "?";
    }

    // T99 #1 — per-session finish-edge tracking. Runs for every session,
    // including `done` ones, so a working -> done transition is caught before
    // the aggregate `continue` below skips done/none sessions.
    const char *sid = s["sessionId"] | "";
    if (sid[0]) {
      int slot = -1;
      for (int i = 0; i < 16; i++) {
        if (agentSessionSlots[i].id[0] && !strcmp(agentSessionSlots[i].id, sid)) { slot = i; break; }
      }
      if (slot >= 0) {
        if (agentSessionSlots[slot].status == AGENT_STATUS_WORKING &&
            (st == AGENT_STATUS_DONE || st == AGENT_STATUS_IDLE)) {
          anyFinishEdge = true;
        }
        agentSessionSlots[slot].status = st;
        slotSeen[slot] = true;
      } else {
        for (int i = 0; i < 16; i++) {
          if (!agentSessionSlots[i].id[0]) { // first sighting — record, no chime
            snprintf(agentSessionSlots[i].id, sizeof(agentSessionSlots[i].id), "%s", sid);
            agentSessionSlots[i].status = st;
            slotSeen[i] = true;
            break;
          }
        }
      }
    }

    // T83 (2026-09-01, Brian's correction): deliberately does NOT skip
    // `stale` sessions anymore (used to: `if (s["stale"] | false)
    // continue;`). The hub's 120s stale timeout meant a needs-input/error
    // session that just sat ignored long enough silently dropped out of the
    // aggregate — a false "all clear," not an actual resolution. Attention
    // state should now only clear on a real status change (a hook event) or
    // the session being removed (SessionEnd -> DELETE), regardless of age.
    // Accepted trade-off: a session that hard-crashes without ever firing
    // SessionEnd leaves the ring/buzzer stuck lit with no auto-recovery —
    // Brian prefers that over this silent false-clear. Applies to both T60's
    // buzz/badge and T77's ring, since they share this function's output.
    if (st == AGENT_STATUS_DONE || st == AGENT_STATUS_NONE) continue; // finished/unrecognized sessions don't hold the ring (T87: `done` is transient-terminal, not active)
    activeCount++;
    if (st == AGENT_STATUS_WORKING) anyWorkingNow = true; // T99 two-tier chime
    if (st == AGENT_STATUS_NEEDS_INPUT || st == AGENT_STATUS_ERROR) attentionCount++;
    if (agentStatusPriority(st) > agentStatusPriority(worst)) worst = st;
  }
  // T99 #1 — a slot not seen in this snapshot means the session was removed
  // (SessionEnd -> DELETE); free it so its id can be reused and it can't
  // spuriously match a future session.
  for (int i = 0; i < 16; i++) {
    if (!slotSeen[i]) agentSessionSlots[i].id[0] = 0;
  }
  // T99 (2026-09-03): two-tier completion chime, decided once per poll from
  // this same snapshot — PARTIAL if some other tracked session is still
  // `working` right now, ALL_CLEAR if this poll's finish edge(s) left none
  // working. Mutually exclusive; loop() picks the buzz pattern from whichever
  // is set.
  if (anyFinishEdge) {
    if (anyWorkingNow) result.agentDoneChime = true;
    else result.agentAllClearChime = true;
  }
  result.agentAggregateStatus = (activeCount > 0) ? worst : AGENT_STATUS_NONE;
  result.agentAttentionCount = attentionCount;
  snprintf(dbgAgentSessions, sizeof(dbgAgentSessions), "%s", sessList.c_str());
}

// T48 (2026-08-26): utcOffsetMinutes now free-runs between syncs instead of
// being re-adopted from every ~1s poll — Brian's request, the same "keep a
// local-optimistic value, only occasionally reconciled" shape already used
// for volume/mute/media, just applied to a PC-reported *setting* instead of
// a user action. Sync (actually reading pcState.utcOffsetMinutes and
// persisting it) only happens when pcConnected transitions false->true, or
// otherwise at most once every UTC_OFFSET_RESYNC_INTERVAL_MS — timezone
// changes are rare, so there's no reason to touch NVS or even look at the
// field on every poll. Tracked here (not in loop()) so the actual
// syncPrefs.putInt() write stays on syncPollTask — see syncPrefs's own
// comment for why that invariant matters (never touched from loop()/the
// main task). Between syncs, getIdleLocalTime() still free-runs correctly
// off this device's own NTP-disciplined clock + whichever offset was last
// accepted; only the *acceptance* of a new offset is gated, not the clock.
bool syncTaskWasPcConnected = false;
uint32_t lastUtcOffsetSyncMs = 0;
constexpr uint32_t UTC_OFFSET_RESYNC_INTERVAL_MS = 3600000; // 1 hour

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
  result.dimDelayMs = dimDelayMs;
  result.utcOffsetMinutes = utcOffsetMinutes; // T44, same preserved-unless-real-value pattern
  // T76 — fresh every poll, not preserved (see the field's own comment).
  // Large enough that a missing/failed poll reads as "no recent input
  // known" rather than accidentally satisfying PC_RECENT_INPUT_WINDOW_S.
  result.lastInputSecondsAgo = UINT32_MAX;
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
  result.timeZoneName = ""; // cosmetic-only, safe to blip empty on a transient parse — unlike utcOffsetMinutes, not preserved
  result.ok = false;
  // T60: preserved-unless-a-real-value-arrives, same pattern as
  // volumeFuseMs/idleToHomeMs/etc above — agentAggregateStatus/
  // agentAttentionCount are only ever overwritten by pollAgentSessions() on
  // its own slower cadence (see AGENT_SESSIONS_POLL_INTERVAL_MS), and
  // buzzerEnabled only when the settings blob below actually has it.
  result.agentAggregateStatus = agentAggregateStatus;
  result.agentAttentionCount = agentAttentionCount;
  result.buzzerEnabled = buzzerEnabled;
  result.attentionRenagMinutes = attentionRenagMinutes; // T89 — preserved unless the settings blob carries a new value
  result.agentDoneChime = false;     // T87/T99 — event flag, only set true by pollAgentSessions() on a real per-session working->done/idle edge
  result.agentAllClearChime = false; // T99 — event flag, see agentDoneChime's own comment
  result.agentPollOk = false;    // T99 #3 — set true only on a successful agent-sessions GET+parse

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
          // T40: field name/default ("dimDelayMs", 60000ms) proposed to the
          // coordinator for pc-dash-hub to match — not yet confirmed live
          // end-to-end, so this falls back to the local default/NVS value
          // (same `| dimDelayMs` shape as the other two) until it does.
          result.dimDelayMs = timeouts["dimDelayMs"] | dimDelayMs;
          if (result.volumeFuseMs != volumeFuseMs) syncPrefs.putUInt("volFuseMs", result.volumeFuseMs);
          if (result.idleToHomeMs != idleToHomeMs) syncPrefs.putUInt("idleMs", result.idleToHomeMs);
          if (result.dimDelayMs != dimDelayMs) syncPrefs.putUInt("dimDelayMs", result.dimDelayMs);
        }
        // T60: first field read out of `settings` directly rather than
        // through `timeouts` — buzzerEnabled isn't a timeout. Field name is
        // this firmware's own guess at the wire shape pc-dash-hub already
        // reserved for it (see this function's pre-T60 comment, right
        // above, predating this feature); flagged to the coordinator for
        // pc-dash-hub to confirm/correct, same as T57/T58's own
        // coordinate-first wire-shape questions.
        result.buzzerEnabled = doc["settings"]["buzzerEnabled"] | buzzerEnabled;
        if (result.buzzerEnabled != buzzerEnabled) syncPrefs.putBool("buzzerEn", result.buzzerEnabled);
        // T89: hub-global re-nag interval, echoed read-only into this same
        // settings block (canonical store is /api/settings, which we don't
        // touch). Minutes, not Ms — deliberately not under `timeouts`.
        // Absent key keeps the current value; 0 is valid ("re-nag off").
        result.attentionRenagMinutes = doc["settings"]["attentionRenagMinutes"] | attentionRenagMinutes;
        if (result.attentionRenagMinutes != attentionRenagMinutes) syncPrefs.putUInt("renagMin", result.attentionRenagMinutes);
        // pc-dash-app (step 4) is live now (2026-08-20) — pcState carries
        // real volume/mute/now-playing values, reconciled against the local
        // knob-driven estimate in loop() once pcConnected is true (T14,
        // 2026-08-21; see each var's own comment for the prior placeholder
        // state, and ECOSYSTEM_PLAN.md's "Volume/mute/media reconciliation"
        // section for the design this implements).
        JsonObject pcState = doc["pcState"];
        result.pcConnected = pcState["pcConnected"] | false;
        result.lastSeenSecondsAgo = pcState["lastSeenSecondsAgo"] | 0;
        // T76 — raw seconds since real Windows keyboard/mouse input
        // (pc-dash-app's GetLastInputInfo), always sent once pc-dash-hub's
        // schema carries it through; missing key (hub not updated yet, or
        // pc-dash-app not reporting) falls back to the "no recent input
        // known" default set above, so this is a safe no-op until both legs
        // land — see PC_RECENT_INPUT_WINDOW_S's own comment.
        result.lastInputSecondsAgo = pcState["lastInputSecondsAgo"] | UINT32_MAX;
        // T44/T48 (2026-08-26): local-time source for the idle screen,
        // confirmed shape from pc-dash-app via the coordinator —
        // utcOffsetMinutes (signed minutes east of UTC, e.g. -420 for
        // Phoenix, AZ; see setupTimeSync()'s comment for why an offset
        // instead of a POSIX TZ string). T48: only actually read/accepted on
        // a real sync trigger (connect-transition or hourly — see this
        // function's own top comment); every other poll just carries the
        // already-applied value straight through unchanged, so loop()'s
        // unconditional apply below stays a harmless no-op most polls
        // rather than a fresh network-driven overwrite. timeZoneName is
        // cosmetic only — see its own top-of-file comment.
        bool justConnected = !syncTaskWasPcConnected && result.pcConnected;
        bool resyncDue = (millis() - lastUtcOffsetSyncMs) >= UTC_OFFSET_RESYNC_INTERVAL_MS;
        if (justConnected || resyncDue) {
          result.utcOffsetMinutes = pcState["utcOffsetMinutes"] | utcOffsetMinutes;
          result.timeZoneName = String((const char *)(pcState["timeZoneName"] | ""));
          if (result.utcOffsetMinutes != utcOffsetMinutes) syncPrefs.putInt("utcOffMin", result.utcOffsetMinutes);
          lastUtcOffsetSyncMs = millis();
        }
        // else: result.utcOffsetMinutes/timeZoneName stay at the neutral
        // defaults set at the top of this function (current global / "") —
        // not due for a resync, so this poll doesn't touch either field.
        syncTaskWasPcConnected = result.pcConnected;
        result.volumePercent = pcState["volumePercent"] | 0;
        result.speakerMuted = pcState["speakerMuted"] | false;
        result.nowPlayingIsPlaying = pcState["nowPlayingIsPlaying"] | false;
        JsonObject nowPlaying = pcState["nowPlaying"];
        // normalizeSmartPunctuation (T21): title/artist commonly carry
        // Windows-style curly quotes/dashes the on-device fonts can't render.
        result.nowPlayingTrack = normalizeSmartPunctuation(String((const char *)(nowPlaying["title"] | "")));
        result.nowPlayingArtist = normalizeSmartPunctuation(String((const char *)(nowPlaying["artist"] | "")));
        result.nowPlayingSource = normalizeSmartPunctuation(String((const char *)(nowPlaying["source"] | "")));
        result.nowPlayingPositionMs = nowPlaying["positionMs"] | 0;
        result.nowPlayingDurationMs = nowPlaying["durationMs"] | 0;
        result.ok = true;
      }
    }
    // T60: independent of whether the pcState GET above succeeded — a
    // separate endpoint/request, gated on its own slower cadence rather
    // than every 1s pass (see AGENT_SESSIONS_POLL_INTERVAL_MS's comment).
    if (millis() - lastAgentSessionsPollMs >= AGENT_SESSIONS_POLL_INTERVAL_MS) {
      lastAgentSessionsPollMs = millis();
      pollAgentSessions(result);
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
