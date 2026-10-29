// hub_sync.h — pc-dash-hub polling task: state/settings/agent-session fetch and handoff to loop().
#pragma once

#include <Arduino.h>
#include "agent_status.h"

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

extern bool lastSyncPollOk;
extern SemaphoreHandle_t syncResultMutex;
extern volatile bool syncResultPending;

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

extern SyncResult pendingSyncResult;
void syncPollTask(void *pvParameters);
