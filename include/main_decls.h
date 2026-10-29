// main_decls.h — Declarations still defined in main.cpp; shrinks as modules are extracted.
#pragma once

#include <Arduino.h>
#include <lvgl.h>

void buildSpeakerMuteScreen(lv_obj_t *scr);
void buildNowPlayingScreen(lv_obj_t *scr);
void showNowPlayingScreen();
void buildQuickJumpScreen(lv_obj_t *scr);
void showQuickJumpMenu();
