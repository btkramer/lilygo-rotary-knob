// screen_nav.h — Shared screen objects, return-screen state, restoreScreen(), screen timeouts and swipe gestures.
#pragma once

#include <Arduino.h>
#include <lvgl.h>

extern lv_obj_t *homeScreen;
extern lv_obj_t *volumeScreen;
extern lv_obj_t *speakerMuteScreen;
extern lv_obj_t *quickJumpScreen;
extern lv_obj_t *nowPlayingScreen;
extern uint32_t lastNowPlayingActivityMs;
extern lv_obj_t *volumeReturnScreen;
extern lv_obj_t *speakerMuteReturnScreen;
extern bool speakerMuted;
extern uint32_t lastMuteLocalWriteMs;
extern lv_obj_t *quickJumpReturnScreen;
void restoreScreen(lv_obj_t *target);

// Operator-adjustable from the Docker app, same as volumeFuseMs above.
constexpr uint32_t IDLE_TO_HOME_MS_DEFAULT = 30000; // standard non-Home idle timeout, see PROGRESS.md

extern uint32_t idleToHomeMs;

// T40 (2026-08-25): how long since the last real interaction (any screen,
// including Home) before the burn-in-mitigation idle screen takes over — see
// ECOSYSTEM_PLAN.md's "Burn-in mitigation" section and loop()'s idle-state
// machine below. Operator-adjustable from pc-dash-app, same top-level-under-
// "timeouts" schema shape as volumeFuseMs/idleToHomeMs above (per the T31
// lesson about that shape) — field name/default proposed to the coordinator
// for pc-dash-hub to match, not yet confirmed live end-to-end.
constexpr uint32_t DIM_DELAY_MS_DEFAULT = 60000;

extern uint32_t dimDelayMs;
extern bool displayDimmed;

// T34 (2026-08-22): Now Playing's own idle-to-Home fuse specifically, used
// only while nowPlayingIsPlaying is false (paused/stopped) — see loop()'s
// own comment on this. Not operator-adjustable like idleToHomeMs/volumeFuseMs
// (no request for that yet); a plain hardcoded constant until it is.
constexpr uint32_t NOW_PLAYING_STOPPED_IDLE_MS = 5000;

// T76 (2026-08-30): "stay awake while Brian's actively on the PC" — a
// second, independent gate on top of the knob's own local idle timer
// (dimDelayMs above), not a replacement for it. Threshold is Brian's own
// decision (30 min), not proposed/adjustable like dimDelayMs/idleToHomeMs,
// so a plain hardcoded constant, same as NOW_PLAYING_STOPPED_IDLE_MS above.
// Consumes pcState.lastInputSecondsAgo — field shape confirmed directly with
// pc-dash-app (cross-participant, same pattern as T8/T44/T52/T57; see
// coordinator-reports/pc-dash.md): raw seconds since last real Windows
// keyboard/mouse input (GetLastInputInfo), deliberately not a precomputed
// bool, so the 30-min threshold stays knob-side tunable policy without an
// app redeploy — same reasoning as utcOffsetMinutes (T44). Always sent,
// excluded from pc-dash-app's own change-detection. Also needs pc-dash-hub
// to add the field to its schema before it passes through end-to-end —
// parsed defensively below (large default) so this stays a no-op until then.
constexpr uint32_t PC_RECENT_INPUT_WINDOW_S = 30 * 60;

void makeGestureBubbleThrough(lv_obj_t *obj);
void quickJumpGestureCb(lv_event_t *e);
void screenCycleGestureCb(lv_event_t *e);
