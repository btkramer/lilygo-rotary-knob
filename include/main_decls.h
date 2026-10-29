// main_decls.h — Declarations still defined in main.cpp; shrinks as modules are extracted.
#pragma once

#include <Arduino.h>
#include <lvgl.h>

// Buzzer (T60) — plain tone()/noTone() on PIN_BUZZER (17, confirmed in
// pin_config.h). HARDWARE.md also notes a vibration motor on this board,
// "unused so far" — deliberately NOT wired here too: its GPIO isn't
// documented anywhere this project has confirmed on real hardware (only the
// buzzer pin made it into pin_config.h), and this project's own hard-won
// rule is not to guess hardware pins (see arduino/CLAUDE.md's ACL saga for
// an unrelated example of the same "verify, don't guess" norm). Flagged as
// an open question in coordinator-reports/pc-dash.md rather than guessed.
//
// Non-blocking: loop() must never block (ArduinoOTA.handle() only gets
// serviced when loop() reaches it — see the sync section's own comment on
// why that matters). serviceBuzzer(), called every loop() pass, steps
// through a short on/off pattern purely by comparing millis() against a
// deadline — same "never delay(), always millis()-gated" shape as every
// other timed effect in this file (e.g. showVolumeOverlay's fuse).
struct BuzzStep { uint16_t freqHz; uint16_t onMs; uint16_t offMs; };

// Passive magnetic buzzer on IO17 — its acoustic output falls off steeply
// away from mechanical resonance, so both tiers are driven at one tuned
// frequency rather than the old 2200/1100 Hz split (the 1100 Hz "error" tone
// in particular was well down the response curve and barely audible).
// LilyGo's own factory firmware drives this exact buzzer at 2000 Hz, so
// that's the starting point; refine it with the temporary `buzzsweep`
// WebSerial command (see setupWebSerial()) and fold the loudest value back
// into BUZZ_FREQ_HZ. Tiers stay distinguishable by *rhythm* — two short
// beeps vs. three longer ones — not pitch, per ECOSYSTEM_PLAN.md's
// "distinct patterns per notification tier" ask.
constexpr uint16_t BUZZ_FREQ_HZ = 2000;

constexpr BuzzStep BUZZ_PATTERN_NEEDS_INPUT[] = {{BUZZ_FREQ_HZ, 120, 90}, {BUZZ_FREQ_HZ, 120, 90}};

void startBuzzPattern(const BuzzStep *pattern, size_t len);

// T98 (2026-09-02) — TEMP WebSerial buzzer-pattern audition bench. Lets Brian
// try candidate beep rhythms by ear (via `snd ...` in WebSerial, see
// setupWebSerial()) before the beep vocabulary is locked. Deliberately ignores
// buzzerEnabled — it's a manual test trigger, not a real notification. Strip
// this helper, the `sndTestSteps` buffer, and the `snd ...` command block once
// the beep vocab is finalized (same category as `buzzsweep`/`/debug/buzz`).
constexpr size_t SND_TEST_MAX_STEPS = 16;

void playSndTest(const BuzzStep *src, size_t len);
void startBuzzSweep();
void buzzSpotTest(uint16_t freqHz);
void buildHomeScreen(lv_obj_t *scr);
extern lv_obj_t *idleScreen;
void buildIdleScreen(lv_obj_t *scr);
extern lv_obj_t *homeScreen;
extern lv_obj_t *volumeScreen;
extern lv_obj_t *speakerMuteScreen;
extern lv_obj_t *quickJumpScreen;
extern lv_obj_t *nowPlayingScreen;
extern lv_obj_t *volumeReturnScreen;
extern lv_obj_t *speakerMuteReturnScreen;
extern lv_obj_t *quickJumpReturnScreen;
void buildVolumeScreen(lv_obj_t *scr);
void buildSpeakerMuteScreen(lv_obj_t *scr);
void buildNowPlayingScreen(lv_obj_t *scr);
void buildQuickJumpScreen(lv_obj_t *scr);
void makeGestureBubbleThrough(lv_obj_t *obj);
void quickJumpGestureCb(lv_event_t *e);
void screenCycleGestureCb(lv_event_t *e);
