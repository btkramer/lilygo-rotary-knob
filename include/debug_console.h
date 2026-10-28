// debug_console.h — Boot reset-reason diagnostic and the WebSerial console (with its test commands).
#pragma once

#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp_system.h" // reset-reason diagnostic, see bootResetReason's own comment

extern esp_reset_reason_t bootResetReason;
const char *resetReasonStr(esp_reset_reason_t reason);
extern AsyncWebServer webServer;
void setupWebSerial();
