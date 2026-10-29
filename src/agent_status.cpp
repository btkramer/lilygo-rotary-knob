// agent_status.cpp — Claude Code agent-monitor status: aggregate state, Home badge, flash and global ring.

#include <Arduino.h>
#include <lvgl.h>
#include "pc_state.h"
#include "agent_status.h"

// The knob shows one badge, not a grid (that's the phone's job, T59) — many
// concurrent sessions collapse to whichever one most needs attention.
// needs-input outranks error: it's the state that specifically can't
// proceed without a human right now, which is this whole feature's reason
// to exist, whereas error (something broke) is bad but not necessarily
// blocked on YOU specifically. v1 heuristic, easy to reorder later if living
// with it says otherwise.
int agentStatusPriority(AgentStatus s) {
  switch (s) {
    case AGENT_STATUS_NEEDS_INPUT: return 5;
    case AGENT_STATUS_ERROR:       return 4;
    case AGENT_STATUS_WORKING:     return 3;
    case AGENT_STATUS_IDLE:        return 2;
    case AGENT_STATUS_DONE:        return 1;
    default:                       return 0;
  }
}

lv_color_t agentStatusColor(AgentStatus s) {
  switch (s) {
    // Amber/red match the Codex-Micro-style vocabulary this feature's design
    // was synthesized from (ECOSYSTEM_PLAN.md); blue/green reuse this
    // project's own existing ring-gauge colors (gpuArc/ramArc) rather than
    // inventing a third palette.
    case AGENT_STATUS_NEEDS_INPUT: return lv_color_hex(0xFFB300); // amber — needs you
    case AGENT_STATUS_ERROR:       return lv_color_hex(0xFF3B30); // red — something broke
    case AGENT_STATUS_WORKING:     return lv_color_hex(0x00C853); // green — active, fine
    case AGENT_STATUS_IDLE:        return lv_color_hex(0x0080FF); // blue — calm
    case AGENT_STATUS_DONE:        return lv_color_hex(0x555555); // dim gray
    default:                       return lv_color_hex(0x333333);
  }
}

AgentStatus agentStatusFromString(const char *s) {
  if (!s) return AGENT_STATUS_NONE;
  if (!strcmp(s, "idle")) return AGENT_STATUS_IDLE;
  if (!strcmp(s, "working")) return AGENT_STATUS_WORKING;
  if (!strcmp(s, "needs-input")) return AGENT_STATUS_NEEDS_INPUT;
  if (!strcmp(s, "error")) return AGENT_STATUS_ERROR;
  if (!strcmp(s, "done")) return AGENT_STATUS_DONE;
  return AGENT_STATUS_NONE; // unrecognized status string — treat like "no session", not a crash
}

// Aggregate across all tracked sessions, applied by loop() from the synced
// poll result — see pollAgentSessions()/pollSyncState() further down.
AgentStatus agentAggregateStatus = AGENT_STATUS_NONE;
int agentAttentionCount = 0; // sessions currently needs-input or error

// T99 (2026-09-03) — agent-feed freshness. The orange WORKING ring is driven
// off the agent-sessions feed alone (not pcConnected, which is pc-dash-app's
// separate SMTC signal — see updateAgentAttentionRing()), so it needs its own
// "is this data still current" gate: hide the working ring if no agent poll
// has succeeded within AGENT_FEED_STALE_MS. Set from loop()'s agent-apply block.
uint32_t lastAgentPollOkMs = 0;
int agentRingRendered = -1; // 0 hidden / 1 working / 2 attention — also read by /debug/agent

// T99 (2026-09-03, temporary) — /debug/agent instrumentation. Captured by
// pollAgentSessions() each poll so the fetch/parse can be inspected remotely
// without serial. Strip with the rest of the agent debug code once the ring
// is confirmed working end-to-end.
int dbgAgentGetCode = 0;
size_t dbgAgentBodyLen = 0;
char dbgAgentParseErr[24] = "none";
char dbgAgentSessions[256] = "";

// T90 (2026-09-02) — attention-indicator reframe. The global ring
// (agentAttentionRing, T77) now carries three meanings instead of only
// firing on needs-input/error:
//   - any session `working`, none needing input -> ORANGE pulse at the
//     original T77 speed (RING_PULSE_WORKING_MS)
//   - aggregate needs-input/error -> RED pulse, deliberately SLOWER than
//     orange (Brian: "a bit slower") so the two read differently at a glance
//   - otherwise -> ring hidden
// needs-input still wins over working (agentStatusPriority already does
// this). PC-offline (T91) hides the ring regardless of last-known state.
constexpr uint32_t RING_PULSE_WORKING_MS = 500;    // == T77's original pulse half-cycle
constexpr uint32_t RING_PULSE_ATTENTION_MS = 850;  // slower than working
constexpr uint32_t RING_COLOR_WORKING = 0xFF8C00;  // orange (matches cpuArc)
constexpr uint32_t RING_COLOR_ATTENTION = 0xFF3B30; // red (matches AGENT_STATUS_ERROR)

lv_obj_t *agentStatusBadge;  // T60 — pulse-ring pattern, see its own build-site comment
lv_obj_t *agentFlashOverlay; // T60 — one-shot full-panel tint on entering an attention state
lv_obj_t *agentAttentionRing; // T77 — persistent global ring, see its own build-site comment

// T60: pulse anim callback for agentStatusBadge — same technique as
// speakerMutePulseAnimCb (animate a plain lv_obj's border opacity, never an
// lv_arc — see speakerMuteRingPulse's own comment for why lv_arc is unsafe
// here for anything redrawn repeatedly).
void agentBadgePulseAnimCb(void *obj, int32_t value) {
  lv_obj_set_style_border_opa((lv_obj_t *)obj, (lv_opa_t)value, 0);
}

// Reflects agentAggregateStatus on Home's badge: hidden when there's
// nothing to show (AGENT_STATUS_NONE), a steady-lit ring for calm states
// (idle/working/done), an infinite pulse — faster than Speaker Mute's 900ms,
// deliberately more insistent — for the two states that actually need a
// human (needs-input/error). Called whenever agentAggregateStatus changes
// (loop()'s sync-apply step) — cheap to call redundantly (just restarts the
// same anim), but callers only do so on an actual change, matching this
// file's general "only touch LVGL on real change" convention.
void updateAgentStatusBadge() {
  lv_anim_delete(agentStatusBadge, agentBadgePulseAnimCb);
  if (agentAggregateStatus == AGENT_STATUS_NONE) {
    lv_obj_add_flag(agentStatusBadge, LV_OBJ_FLAG_HIDDEN);
    return;
  }
  lv_obj_remove_flag(agentStatusBadge, LV_OBJ_FLAG_HIDDEN);
  lv_obj_set_style_border_color(agentStatusBadge, agentStatusColor(agentAggregateStatus), 0);
  bool attention = (agentAggregateStatus == AGENT_STATUS_NEEDS_INPUT || agentAggregateStatus == AGENT_STATUS_ERROR);
  if (attention) {
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, agentStatusBadge);
    lv_anim_set_exec_cb(&a, agentBadgePulseAnimCb);
    lv_anim_set_values(&a, LV_OPA_30, LV_OPA_COVER);
    lv_anim_set_time(&a, 500);
    lv_anim_set_playback_time(&a, 500); // fade back down, not a hard cut — same shape as speakerMuteRingPulse
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_start(&a);
  } else {
    lv_obj_set_style_border_opa(agentStatusBadge, LV_OPA_COVER, 0);
  }
}

// T60: one-shot full-panel tint, fired only on transitioning INTO an
// attention state (loop()'s sync-apply step) — the "flash" of "buzz/flash/
// pulse on state changes". A translucent tint, not LV_OPA_COVER, so the
// CPU/GPU/RAM readout stays legible through it rather than being fully
// blanked. Round (LV_RADIUS_CIRCLE), matching the physical round panel —
// square corners would show where the display can't actually render.
void agentFlashAnimCb(void *obj, int32_t value) {
  lv_obj_set_style_bg_opa((lv_obj_t *)obj, (lv_opa_t)value, 0);
}

// T77: thin pulsing ring around the panel edge, global — lives on
// lv_layer_top(), not any one screen's object tree, so it renders above
// whichever screen is currently active and needs no per-screen wiring.
// Persistent while the attention state holds (unlike agentFlashOverlay's
// one-shot tint), and visible on every screen (unlike agentStatusBadge,
// deliberately Home-only). Reuses agentAggregateStatus/agentStatusColor —
// the same "needs-input or error" trigger T60's badge/flash already fire
// on and T72's phone banner is scoped to reuse — so all three stay
// consistent rather than re-deriving the edge condition three ways.
// agentStatusBadge/agentFlashOverlay are left as-is; this is additive.
// Reuses agentBadgePulseAnimCb (already a generic border-opacity animator,
// not badge-specific) rather than duplicating the same three-line callback.
// T99 (2026-09-03): idempotent — tracks the last-rendered state in
// agentRingRendered and no-ops when it hasn't changed, so loop()'s periodic
// refresh (#7) can call this every pass without restarting the pulse anim.
// Also decouples the WORKING ring from pcConnected (#2): the orange ring is
// driven purely by the agent-sessions feed + its own freshness
// (AGENT_FEED_STALE_MS), since agent sessions arrive via pc-dash-app's hook
// forwarder — a separate signal from the SMTC/Core-Audio readback pcConnected
// tracks. The needs-input/error (red) ring KEEPS the pcConnected gate (T91):
// a PC hibernating must not leave it flashing forever. A session that crashed
// without SessionEnd still holds the red ring (T83) — pcConnected is true in
// that case.
void updateAgentAttentionRing() {
  bool attention = (agentAggregateStatus == AGENT_STATUS_NEEDS_INPUT || agentAggregateStatus == AGENT_STATUS_ERROR);
  bool working = (agentAggregateStatus == AGENT_STATUS_WORKING);
  bool agentFeedFresh = lastAgentPollOkMs != 0 && (millis() - lastAgentPollOkMs) < AGENT_FEED_STALE_MS;
  bool showAttention = attention && pcConnected;      // T91 — unchanged
  bool showWorking = working && agentFeedFresh;       // T99 #2 — no pcConnected term
  int desired = showAttention ? 2 : (showWorking ? 1 : 0); // attention wins over working
  if (desired == agentRingRendered) return;           // idempotent — nothing to do
  agentRingRendered = desired;

  lv_anim_delete(agentAttentionRing, agentBadgePulseAnimCb);
  if (desired == 0) {
    lv_obj_add_flag(agentAttentionRing, LV_OBJ_FLAG_HIDDEN);
    return;
  }
  bool isAttentionRing = (desired == 2);
  lv_obj_remove_flag(agentAttentionRing, LV_OBJ_FLAG_HIDDEN);
  lv_obj_set_style_border_color(agentAttentionRing,
                                lv_color_hex(isAttentionRing ? RING_COLOR_ATTENTION : RING_COLOR_WORKING), 0);
  uint32_t pulseMs = isAttentionRing ? RING_PULSE_ATTENTION_MS : RING_PULSE_WORKING_MS;
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, agentAttentionRing);
  lv_anim_set_exec_cb(&a, agentBadgePulseAnimCb);
  lv_anim_set_values(&a, LV_OPA_30, LV_OPA_COVER);
  lv_anim_set_time(&a, pulseMs);
  lv_anim_set_playback_time(&a, pulseMs); // fade back down, same shape as agentStatusBadge's own pulse
  lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
  lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
  lv_anim_start(&a);
}

void triggerAgentFlash(lv_color_t color) {
  lv_obj_set_style_bg_color(agentFlashOverlay, color, 0);
  lv_anim_delete(agentFlashOverlay, agentFlashAnimCb);
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, agentFlashOverlay);
  lv_anim_set_exec_cb(&a, agentFlashAnimCb);
  lv_anim_set_values(&a, 0, LV_OPA_60);
  lv_anim_set_time(&a, 220);
  lv_anim_set_playback_time(&a, 380);
  lv_anim_set_repeat_count(&a, 1); // one-shot, not a loop — agentStatusBadge's own pulse is the ongoing indicator
  lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
  lv_anim_start(&a);
}
