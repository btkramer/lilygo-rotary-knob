// main_decls.h — Declarations still defined in main.cpp; shrinks as modules are extracted.
#pragma once

#include <Arduino.h>
#include <lvgl.h>

void buildHomeScreen(lv_obj_t *scr);
extern lv_obj_t *idleScreen;
void buildIdleScreen(lv_obj_t *scr);
extern uint32_t lastVolumeActivityMs;
extern uint32_t lastSpeakerMuteActivityMs;
extern uint32_t lastQuickJumpActivityMs;
void buildVolumeScreen(lv_obj_t *scr);
void buildSpeakerMuteScreen(lv_obj_t *scr);
void buildNowPlayingScreen(lv_obj_t *scr);
void showNowPlayingScreen();
void buildQuickJumpScreen(lv_obj_t *scr);
void showQuickJumpMenu();
