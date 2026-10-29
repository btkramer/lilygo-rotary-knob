// pc_state.h — Live PC state synced from pc-dash-hub (temps, pcConnected, last input).
#pragma once

#include <Arduino.h>

extern float cpuTempC;
extern float gpuTempC;
extern float ramPercent;
extern bool pcConnected;
extern uint32_t lastSeenSecondsAgo;
extern uint32_t lastInputSecondsAgo;

// pc-dash-app verified live on real Windows hardware 2026-08-20 (reported via
// the pc-dash coordinator) — offline dim treatment re-enabled now that there's
// a real PC to go offline from. pcConnected itself, the sync poll, and NVS
// persistence were never touched by this flag; only the *display* of the dim
// treatment was suppressed while testing (Brian, 2026-08-19).
constexpr bool PC_OFFLINE_DIM_ENABLED = true;
