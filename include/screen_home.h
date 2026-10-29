// screen_home.h — Home screen: CPU/GPU/RAM ring gauges.
#pragma once

#include <Arduino.h>
#include <lvgl.h>

lv_obj_t *makeRingGauge(lv_obj_t *parent, int16_t diameter, int16_t thickness, lv_color_t color);
void updateHomeScreen();
void buildHomeScreen(lv_obj_t *scr);
extern int32_t utcOffsetMinutes;
extern String timeZoneName;
