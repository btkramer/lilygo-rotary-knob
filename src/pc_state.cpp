// pc_state.cpp — Live PC state synced from pc-dash-hub (temps, pcConnected, last input).

#include <Arduino.h>
#include "pc_state.h"

// Placeholder values until the companion app protocol exists (see
// PROGRESS.md's "Next" list) — proving the LVGL screen rendering works on
// real hardware first, same as every other piece of this firmware so far.
// CPU/GPU are temperatures (°C), not percentages — matches the original
// mockup, which shows "CPU 42°"/"GPU 54°" but "RAM 61%". Named accordingly
// (fixed 2026-08-06 after the Home screen mislabeled all three as %).
float cpuTempC = 42;
float gpuTempC = 54;
float ramPercent = 61;

// Whether pc-dash-app (the Windows companion, not built yet — see
// PROGRESS.md's "Next" list) has reported in recently, per the Docker app
// sync protocol's pcState.pcConnected. PC-dependent display data (these
// rings, Now Playing's track info) shows a dimmed/offline treatment when
// false instead of quietly showing stale numbers; PC-independent data
// (clock, weather, lights, settings) is unaffected either way.
bool pcConnected = false;
uint32_t lastSeenSecondsAgo = 0;
// T76 — see PC_RECENT_INPUT_WINDOW_S's own comment and pollSyncState()'s
// parsing of pcState.lastInputSecondsAgo. UINT32_MAX default (not 0) so
// "no data yet" never reads as "PC just used" and forces the knob awake.
uint32_t lastInputSecondsAgo = UINT32_MAX;
