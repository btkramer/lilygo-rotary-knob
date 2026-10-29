// http_ota.h — HTTP firmware upload, /restart and temporary /debug endpoints.
#pragma once

#include <Arduino.h>

extern bool rebootPending;
extern uint32_t rebootAtMs;
extern volatile bool buzzTestPending;
extern volatile int buzzTestPattern;
void setupHttpOta();
