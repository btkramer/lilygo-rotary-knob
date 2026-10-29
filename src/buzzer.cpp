// buzzer.cpp — Non-blocking buzzer patterns, attention chime, audition/sweep test helpers.

#include <Arduino.h>
#include <WebSerial.h>
#include "pin_config.h"
#include "agent_status.h"
#include "buzzer.h"

const BuzzStep *buzzPattern = nullptr;
size_t buzzPatternLen = 0;
size_t buzzStepIndex = 0;
uint32_t buzzStepUntilMs = 0;
bool buzzPlaying = false;
bool buzzInOnPhase = false;
// Settings-synced (see pollSyncState()'s settings parsing) — defaults on
// until a real value has synced. Field name (`buzzerEnabled`) is this
// firmware's own guess at the wire shape pc-dash-hub already reserved for
// it (see pollSyncState()'s pre-existing comment predating this feature);
// flagged to the coordinator for pc-dash-hub to confirm/correct, same
// coordinate-first pattern as T57/T58's own wire-shape questions.
bool buzzerEnabled = true;

uint32_t attentionRenagMinutes = ATTENTION_RENAG_MINUTES_DEFAULT;
// millis() of the last attention chime (entry or re-nag) — see loop()'s
// re-nag block and the sync-apply attention-edge handler.
uint32_t lastAttentionChimeMs = 0;

// --- Temporary buzzer resonance sweep (added 2026-08-31) -------------------
// `buzzsweep` over WebSerial walks the buzzer from 1500 Hz to 4500 Hz in
// 250 Hz steps, ~600ms each, printing every frequency so the loudest can be
// picked by ear and folded into BUZZ_FREQ_HZ above; `buzz <hz>` spot-tests
// one frequency; `buzz` alone replays the needs-input pattern. Serviced from
// loop() like serviceBuzzer(), so it never blocks WebSerial/AsyncTCP.
// Remove all of this once BUZZ_FREQ_HZ is tuned.
constexpr uint16_t BUZZ_SWEEP_START_HZ = 1500;
constexpr uint16_t BUZZ_SWEEP_END_HZ = 4500;
constexpr uint16_t BUZZ_SWEEP_STEP_HZ = 250;
constexpr uint16_t BUZZ_SWEEP_DWELL_MS = 600;
bool buzzSweepActive = false;
uint16_t buzzSweepFreq = 0;
uint32_t buzzSweepStepUntilMs = 0;

void startBuzzPattern(const BuzzStep *pattern, size_t len) {
  if (!buzzerEnabled || len == 0 || buzzSweepActive) return;
  buzzPattern = pattern;
  buzzPatternLen = len;
  buzzStepIndex = 0;
  buzzPlaying = true;
  buzzInOnPhase = true;
  tone(PIN_BUZZER, buzzPattern[0].freqHz);
  buzzStepUntilMs = millis() + buzzPattern[0].onMs;
}

// T91 (2026-09-02): hard-stop any in-progress chime — used when the PC goes
// offline so a mid-pattern attention beep doesn't keep sounding after the
// trigger is gone. Safe to call when nothing is playing.
void stopBuzzPattern() {
  if (!buzzPlaying) return;
  noTone(PIN_BUZZER);
  buzzPlaying = false;
  buzzInOnPhase = false;
}

// T90 (2026-09-02) — fire the attention chime for the current aggregate
// (needs-input vs error rhythm). Used both on the entry edge and by loop()'s
// re-nag timer, so the pattern selection isn't duplicated.
void startAttentionChime() {
  bool isError = (agentAggregateStatus == AGENT_STATUS_ERROR);
  startBuzzPattern(isError ? BUZZ_PATTERN_ERROR : BUZZ_PATTERN_NEEDS_INPUT,
                   isError ? sizeof(BUZZ_PATTERN_ERROR) / sizeof(BUZZ_PATTERN_ERROR[0])
                           : sizeof(BUZZ_PATTERN_NEEDS_INPUT) / sizeof(BUZZ_PATTERN_NEEDS_INPUT[0]));
}

BuzzStep sndTestSteps[SND_TEST_MAX_STEPS];
void playSndTest(const BuzzStep *src, size_t len) {
  if (len == 0 || len > SND_TEST_MAX_STEPS || buzzSweepActive) return;
  for (size_t i = 0; i < len; i++) sndTestSteps[i] = src[i];
  bool prev = buzzerEnabled;
  buzzerEnabled = true;                 // force-play regardless of the setting
  startBuzzPattern(sndTestSteps, len);  // only reads buzzerEnabled at entry
  buzzerEnabled = prev;
}

void serviceBuzzer() {
  if (!buzzPlaying) return;
  if (millis() < buzzStepUntilMs) return;
  if (buzzInOnPhase) {
    noTone(PIN_BUZZER);
    buzzInOnPhase = false;
    buzzStepUntilMs = millis() + buzzPattern[buzzStepIndex].offMs;
  } else {
    buzzStepIndex++;
    if (buzzStepIndex >= buzzPatternLen) {
      buzzPlaying = false;
      return;
    }
    buzzInOnPhase = true;
    tone(PIN_BUZZER, buzzPattern[buzzStepIndex].freqHz);
    buzzStepUntilMs = millis() + buzzPattern[buzzStepIndex].onMs;
  }
}

void startBuzzSweep() {
  buzzPlaying = false; // preempt any normal pattern
  buzzSweepActive = true;
  buzzSweepFreq = BUZZ_SWEEP_START_HZ;
  tone(PIN_BUZZER, buzzSweepFreq);
  buzzSweepStepUntilMs = millis() + BUZZ_SWEEP_DWELL_MS;
  Serial.printf("buzzsweep: %u Hz\n", buzzSweepFreq);
  WebSerial.printf("buzzsweep: %u Hz\n", buzzSweepFreq);
}

void serviceBuzzSweep() {
  if (!buzzSweepActive) return;
  if (millis() < buzzSweepStepUntilMs) return;
  if (buzzSweepFreq >= BUZZ_SWEEP_END_HZ) {
    noTone(PIN_BUZZER);
    buzzSweepActive = false;
    Serial.println("buzzsweep: done");
    WebSerial.println("buzzsweep: done");
    return;
  }
  buzzSweepFreq += BUZZ_SWEEP_STEP_HZ;
  tone(PIN_BUZZER, buzzSweepFreq);
  buzzSweepStepUntilMs = millis() + BUZZ_SWEEP_DWELL_MS;
  Serial.printf("buzzsweep: %u Hz\n", buzzSweepFreq);
  WebSerial.printf("buzzsweep: %u Hz\n", buzzSweepFreq);
}

void buzzSpotTest(uint16_t freqHz) {
  buzzSweepActive = false;
  buzzPlaying = false;
  tone(PIN_BUZZER, freqHz, 600); // core schedules its own noTone after 600ms
  Serial.printf("buzz: %u Hz for 600ms\n", freqHz);
  WebSerial.printf("buzz: %u Hz for 600ms\n", freqHz);
}
