// screen_idle.h — Burn-in-safe idle clock/date screen.
#pragma once

#include <Arduino.h>
#include <lvgl.h>
#include <time.h> // T40: struct tm/strftime for the idle screen's clock

extern lv_obj_t *idleScreen;
void buildIdleScreen(lv_obj_t *scr);
bool getIdleLocalTime(struct tm *outTm);
void updateIdleScreen();
