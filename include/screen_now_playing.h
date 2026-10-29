// screen_now_playing.h — Now Playing screen and its HID transport controls.
#pragma once

#include <Arduino.h>
#include <lvgl.h>

extern String nowPlayingArtist;
extern String nowPlayingTrack;
extern String nowPlayingSource;
extern int nowPlayingProgressPercent;
extern uint32_t nowPlayingPositionMs;
extern uint32_t nowPlayingDurationMs;
extern uint32_t lastPositionTickMs;
extern uint32_t positionDriftStreak;

constexpr uint32_t POSITION_TICK_MS = 1000;

constexpr uint32_t POSITION_DRIFT_TOLERANCE_MS = 1500; // normal poll/network jitter shouldn't count as real drift

constexpr uint32_t POSITION_DRIFT_CONFIRM_COUNT = 2;

void formatNowPlayingElapsed(uint32_t positionMs, uint32_t durationMs);
String normalizeSmartPunctuation(const String &in);
extern bool nowPlayingIsPlaying;
extern uint32_t lastPlayPauseLocalWriteMs;
void updateNowPlayingScreen();
void buildNowPlayingScreen(lv_obj_t *scr);
void showNowPlayingScreen();
