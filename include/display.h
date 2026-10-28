// display.h — CO5300 AMOLED panel, LVGL buffers/flush/tick, brightness levels, setupDisplay()/setupLVGL().
#pragma once

#include <Arduino.h>
#include <Arduino_GFX_Library.h>

extern Arduino_CO5300 *panel;
extern bool displayOk;

// T40/T41 (2026-08-25): three brightness levels for three different purposes
// — deliberately not the same constant reused, since "dim for burn-in
// ambient legibility" and "dim to cut power draw during a flash write" want
// very different values. DISPLAY_BRIGHTNESS_NORMAL matches this board's
// original, already-tuned boot-time value (unchanged).
// DISPLAY_BRIGHTNESS_IDLE is the idle-state machine's (loop()) ambient
// level — T40 originally used a very low value here (reusing what's now
// DISPLAY_BRIGHTNESS_OTA_DIM) that Brian found unreadably dim in practice;
// T41 bumped it to roughly 75% of the full 0-255 range instead, close to
// DISPLAY_BRIGHTNESS_NORMAL on purpose — the idle screen's own alternating
// content/font/position already does the actual burn-in-relevant work (see
// idleScreen's own top comment), so this level exists for a softer "resting"
// look, not to meaningfully cut power draw.
// DISPLAY_BRIGHTNESS_OTA_DIM keeps the original very-low value — used only
// during an active OTA write (see setupHttpOta()/setupOTA()'s onStart/onEnd/
// onError), where cutting power draw is the actual point and nobody's
// expected to be reading the screen anyway.
// None of these are operator-adjustable (no request for that yet, unlike
// dimDelayMs) — plain constants until there's a reason to change that.
constexpr uint8_t DISPLAY_BRIGHTNESS_NORMAL = 200;

constexpr uint8_t DISPLAY_BRIGHTNESS_IDLE = 191; // ~75% of 0-255

constexpr uint8_t DISPLAY_BRIGHTNESS_OTA_DIM = 20;

void setupDisplay();
void setupLVGL();
