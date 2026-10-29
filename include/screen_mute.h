// screen_mute.h — Speaker-mute overlay screen.
#pragma once

#include <Arduino.h>
#include <lvgl.h>

void updateSpeakerMuteScreen();
void buildSpeakerMuteScreen(lv_obj_t *scr);
void showSpeakerMuteOverlay();
