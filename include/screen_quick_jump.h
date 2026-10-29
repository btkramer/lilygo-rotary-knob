// screen_quick_jump.h — Quick-jump grid screen.
#pragma once

#include <Arduino.h>
#include <lvgl.h>

// Longer than volumeFuseMs (5s default) — this is a browsed menu you look at
// and choose from, not a transient nudge like Volume/Speaker Mute.
constexpr uint32_t QUICK_JUMP_RETURN_MS = 8000;

void buildQuickJumpScreen(lv_obj_t *scr);
void showQuickJumpMenu();
