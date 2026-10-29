// agent_status.h — Claude Code agent-monitor status: aggregate state, Home badge, flash and global ring.
#pragma once

#include <Arduino.h>
#include <lvgl.h>

// Agent-monitor mode (T60, 2026-08-28) — knob-side glanceable per-session
// status indicator for Claude Code (and later other agent-tool) sessions,
// fed by pc-dash-hub's new agent-session endpoints (T57/T58, both live and
// live-verified 2026-08-28). See ECOSYSTEM_PLAN.md's "Agent-monitor mode"
// section and the pc-dash coordinator's TASKS.md T56/T57/T60 rows for the
// full cross-project design. v1 is notify-only, per that design: a status
// badge (pulse-ring pattern, same technique as speakerMuteRingPulse) on
// Home, plus buzz+flash on transitioning into a state that needs a human.
// No approve/deny/interrupt control yet.
enum AgentStatus {
  AGENT_STATUS_NONE = 0, // no active sessions (or hub unreachable / feature not live) — badge stays hidden
  AGENT_STATUS_IDLE,
  AGENT_STATUS_WORKING,
  AGENT_STATUS_DONE,
  AGENT_STATUS_ERROR,
  AGENT_STATUS_NEEDS_INPUT,
};

int agentStatusPriority(AgentStatus s);
lv_color_t agentStatusColor(AgentStatus s);
AgentStatus agentStatusFromString(const char *s);
extern AgentStatus agentAggregateStatus;
extern int agentAttentionCount;
extern uint32_t lastAgentPollOkMs;
extern int agentRingRendered;
extern int dbgAgentGetCode;
extern size_t dbgAgentBodyLen;
extern char dbgAgentParseErr[24];
extern char dbgAgentSessions[256];

constexpr uint32_t AGENT_FEED_STALE_MS = 9000;      // T99 — 3x AGENT_SESSIONS_POLL_INTERVAL_MS; working ring hides if the feed is older

extern lv_obj_t *agentStatusBadge;
extern lv_obj_t *agentFlashOverlay;
extern lv_obj_t *agentAttentionRing;
void updateAgentStatusBadge();
void updateAgentAttentionRing();
void triggerAgentFlash(lv_color_t color);
