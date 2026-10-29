// main_decls.h — Declarations still defined in main.cpp; shrinks as modules are extracted.
#pragma once

#include <Arduino.h>
#include <lvgl.h>

void buildHomeScreen(lv_obj_t *scr);
extern lv_obj_t *idleScreen;
void buildIdleScreen(lv_obj_t *scr);
extern lv_obj_t *homeScreen;
extern lv_obj_t *volumeScreen;
extern lv_obj_t *speakerMuteScreen;
extern lv_obj_t *quickJumpScreen;
extern lv_obj_t *nowPlayingScreen;
extern lv_obj_t *volumeReturnScreen;
extern lv_obj_t *speakerMuteReturnScreen;
extern lv_obj_t *quickJumpReturnScreen;
void buildVolumeScreen(lv_obj_t *scr);
void buildSpeakerMuteScreen(lv_obj_t *scr);
void buildNowPlayingScreen(lv_obj_t *scr);
void buildQuickJumpScreen(lv_obj_t *scr);
void makeGestureBubbleThrough(lv_obj_t *obj);
void quickJumpGestureCb(lv_event_t *e);
void screenCycleGestureCb(lv_event_t *e);
