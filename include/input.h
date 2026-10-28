// input.h — Encoder ISR, the global interaction clock, CST816 touch and LVGL's touch read callback.
#pragma once

#include <Arduino.h>
#include <lvgl.h>

extern portMUX_TYPE encoderMux;
extern volatile int32_t encoderStepAccum;
extern volatile uint32_t lastAnyInteractionMs;
void IRAM_ATTR encoderIsr();
extern bool touchOk;
void setupTouch();
extern uint32_t touchPressCounter;
void lvTouchpadRead(lv_indev_t *indev, lv_indev_data_t *data);
