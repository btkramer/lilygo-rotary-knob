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
#include "main_decls.h"

constexpr uint32_t DEBOUNCE_MS = 30;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;

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
// T28 (2026-08-21): after a local optimistic change (knob rotation / mute
// button / play-pause tap), loop()'s sync-apply step holds that field's
// local value against an incoming poll that still disagrees with it, rather
// than trusting the poll unconditionally. Confirmed real bug: a poll
// response can be honestly the most recent one this device has received and
// still be stale relative to a value the user just set locally moments ago,
// since the full app-detects-the-real-change -> PUT-to-hub -> this device's
// next poll round trip takes real time — with nothing tracking recency,
// loop() would previously overwrite a just-set-correct local value with that
// stale response, then self-correct on the next successful poll (see
// coordinator-reports/pc-dash.md's T28 entry for the fuller investigation).
//
// T29 (2026-08-21): originally a fixed *grace window* — hold for exactly
// LOCAL_OPTIMISTIC_GRACE_MS, then unconditionally trust whatever the very
// next poll says, agreeing or not. Went through 3000ms, then 8000ms, and
// each time the same failure recurred a bit later: play/pause advanced for
// roughly the grace duration, then a still-stale poll got trusted right as
// the window closed, flipping nowPlayingIsPlaying back and killing the local
// position tick (which requires it) — visible as "runs for ~N seconds, then
// falls back to reconcile-only cadence," where N tracked whatever the
// constant currently was. That's the signature of the real backend
// confirmation sometimes taking longer than *any* fixed window, not a wrong
// constant — bumping it again would very likely just recur later still.
//
// Changed the mechanism, not just the number: hold the local value
// indefinitely *while a poll still disagrees with it*, only ever applying a
// differing poll value once LOCAL_OPTIMISTIC_BACKSTOP_MS has passed since
// the local write — there's no longer a moment where the very next poll
// after some fixed delay gets trusted unconditionally regardless of
// agreement. A poll that already agrees is applied immediately (that's just
// real confirmation arriving, whenever it actually does). The backstop
// itself is a last-resort safety net for a genuinely wrong local guess (e.g.
// an HID command that silently didn't register), not the primary
// reconciliation path. This is a firmware-side mitigation, not a fix for
// whatever's making the real backend confirmation slow in the first place;
// see the pc-dash coordinator for pc-dash-app's parallel investigation into
// that.
//
// Considered a 2-consecutive-agreement rule here too (like position's own,
// below) — deliberately not applied: for a boolean field, "poll disagrees
// with local" and "poll agrees with the OTHER possible value" are the same
// condition, so it can't distinguish a real-but-slow confirmation from a
// wrong local guess the way it can for position (which is continuously
// reported, not a discrete pending transition — see position's own
// comment). Either failure mode produces an identical poll sequence here.
//
// 5000ms (2026-08-21, Brian's call, down from 30000ms): a knowing tradeoff,
// not a new finding — favors fast self-correction of a wrong local guess
// over holding out for a slow-but-real confirmation. Accepted risk: a real
// change taking longer than 5s to confirm (SMTC's play-state lag has no
// fixed ceiling — see T29's investigation) will occasionally get reverted
// and then re-applied once the real confirmation actually lands, rather
// than holding steady the whole time. Revisit the number, not the shape, if
// that tradeoff stops feeling right.
constexpr uint32_t LOCAL_OPTIMISTIC_BACKSTOP_MS = 5000;

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
  uint32_t dimDelayMs; // T40, see its own top-of-file comment
  int32_t utcOffsetMinutes; // T44, see its own top-of-file comment — preserved unless pcState has a real value
  String timeZoneName;      // T44, cosmetic only — see its own top-of-file comment
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
  // T60 (agent-monitor mode) — see pollAgentSessions()'s own comment for how
  // these are derived, and buzzerEnabled's declaration for the settings
  // field this rides in on. Both "preserved unless a real value arrives"
  // (same pattern as volumeFuseMs etc.) since agent-sessions is polled on
  // its own slower cadence, not every pcState poll — see
  // AGENT_SESSIONS_POLL_INTERVAL_MS.
  AgentStatus agentAggregateStatus;
  int agentAttentionCount;
  bool buzzerEnabled;
  uint32_t attentionRenagMinutes; // T89 — settings.attentionRenagMinutes; 0 = re-nag off
  bool agentDoneChime;            // T87/T99 — a tracked session crossed working->done/idle this poll, AND another session is still working (PARTIAL chime)
  bool agentAllClearChime;        // T99 — a tracked session crossed working->done/idle this poll, AND no session is still working (ALL_CLEAR chime)
  bool agentPollOk;              // T99 #3 — agent-sessions GET+parse succeeded this poll (independent of the pcState GET / result.ok)
  // T76 — see PC_RECENT_INPUT_WINDOW_S's own comment. Not "preserved unless
  // a real value arrives" like volumeFuseMs/idleToHomeMs above: absence
  // should read as "no recent PC input known," not "keep believing
  // whatever the last poll said," so this defaults fresh every poll.
  uint32_t lastInputSecondsAgo;
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
