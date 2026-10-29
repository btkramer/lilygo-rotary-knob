// screen_volume.h — Volume overlay screen.
#pragma once

#include <Arduino.h>
#include <lvgl.h>

// Volume screen: the only screen reachable by rotating the knob, from any
// other screen, TV-OSD style (see PROGRESS.md's design decisions — rotate is
// claimed globally, not per-screen). No companion app yet (see PROGRESS.md's
// "Next" list), so there's no real OS volume level to read back — this is a
// **local estimate only**, seeded at 50 and nudged by VOLUME_STEP per detent.
// It will drift from the PC's actual volume over time since the HID
// increment/decrement pulses we send are fire-and-forget with no readback.
// Good enough until the companion app can report the real value over serial.
constexpr int VOLUME_STEP = 2;

// Shared by Volume and Speaker Mute — both are transient, globally-triggered
// overlays (rotate / knob-press) that restore whatever they interrupted
// after this long. Renamed from VOLUME_RETURN_MS (2026-08-06) once Speaker
// Mute started using the same fuse. Operator-adjustable from the Docker app
// (see "Docker app sync protocol" in ECOSYSTEM_PLAN.md) — mutable, not
// constexpr, so a synced value can override this hardcoded default; the
// default is what's used until NVS/a sync poll has ever supplied one.
constexpr uint32_t VOLUME_FUSE_MS_DEFAULT = 5000;

extern uint32_t volumeFuseMs;
extern int volumePercent;
extern uint32_t lastVolumeLocalWriteMs;
extern uint32_t lastVolumeActivityMs;
extern uint32_t lastSpeakerMuteActivityMs;
extern uint32_t lastQuickJumpActivityMs;

// Volume level icons (src/img_volume_waves.c) — real FontAwesome 5 pixels,
// same style as LV_SYMBOL_VOLUME_MID/MAX (confirmed perfect on the Speaker
// Mute screen, since those render as pre-baked bitmap glyphs rather than a
// live lv_arc widget — the actual root cause of the tearing on every earlier
// attempt here was lv_arc specifically, not curved shapes in general). Only
// wave2 (the 2-wave middle tier) needed synthesizing — FA5 doesn't ship one —
// done by pixel-diffing the 1-wave and 3-wave glyphs to isolate just the
// middle wave arc and compositing it onto the 1-wave glyph, so it's real
// font pixels throughout, not a hand-drawn approximation. Two earlier
// attempts at this (small lv_arc wave marks, then a rectangle-built
// speaker+bars composite) are superseded by this — see PROGRESS.md for the
// full story of how this converged.
extern const lv_image_dsc_t img_volume_wave1; // 1 wave (low)

extern const lv_image_dsc_t img_volume_wave2; // 2 waves (medium)

extern const lv_image_dsc_t img_volume_wave3; // 3 waves (high)

void updateVolumeScreen();
void buildVolumeScreen(lv_obj_t *scr);
void showVolumeOverlay(int delta);
