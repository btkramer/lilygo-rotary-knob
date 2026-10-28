// input.cpp — Encoder ISR, the global interaction clock, CST816 touch and LVGL's touch read callback.

#include <Arduino.h>
#include <TouchDrv.hpp>
#include <lvgl.h>
#include <WebSerial.h>
#include "pin_config.h"
#include "input.h"

// Encoder reading moved from loop()-polling to a hardware interrupt
// (2026-08-07) — polling missed/misread transitions on fast spins (Brian:
// "sometimes skip a number or two or go in the wrong direction... usually
// only when I turn it quickly"). loop() does real, variably-slow work every
// iteration (LVGL's full-390x390-panel redraw+flush, ArduinoOTA.handle(),
// WebSerial.loop()), so digitalRead(PIN_ENCODER_A) wasn't being sampled
// often/regularly enough to catch every real quadrature transition during a
// fast rotation — a transition landing between two loop() iterations is
// either missed entirely (skipped step) or, worse, read alongside a B value
// that's already moved on to the *next* transition (wrong direction), since
// this decode assumes A and B are sampled close enough together to reflect
// one single real transition. An interrupt captures each transition the
// instant it happens, independent of whatever loop() is currently busy
// doing. Direction logic (`a != b`) is unchanged from the polling version —
// see loop()'s old comment history / PROGRESS.md for how that was derived.
constexpr uint32_t ENCODER_ISR_DEBOUNCE_MS = 2; // contact-bounce filter only — NOT a rotation-speed throttle like the old 30ms was
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t encoderStepAccum = 0; // net detents since loop() last drained it; ISR writes, loop() reads+clears
volatile uint32_t lastEncoderIsrMs = 0;

// T40 (2026-08-25): global "was there any real interaction recently" clock,
// separate from every per-screen *ActivityMs variable below (those each
// gate their own screen's own return-to-Home/return-to-previous fuse) — this
// one drives the burn-in-mitigation idle screen (see ECOSYSTEM_PLAN.md's
// "Burn-in mitigation" section), which can trigger regardless of which
// screen is currently active. Set from all three real interaction sources:
// this ISR (rotation), lvTouchpadRead()'s press edge (touch), and the
// physical-button press branch in loop() (knob click) — "wake = any
// existing touch/knob interaction" per the design, no new input path needed.
// A plain volatile, not mutex-guarded, matching lastEncoderIsrMs just above:
// only ever written with a fresh millis() value from whichever source last
// fired, so a rare torn/stale read costs at most one loop() iteration's
// worth of idle-detection lag, not a correctness bug.
volatile uint32_t lastAnyInteractionMs = 0;

void IRAM_ATTR encoderIsr() {
  uint32_t now = millis();
  if (now - lastEncoderIsrMs < ENCODER_ISR_DEBOUNCE_MS) return;
  lastEncoderIsrMs = now;
  lastAnyInteractionMs = now;
  bool a = digitalRead(PIN_ENCODER_A) == HIGH;
  bool b = digitalRead(PIN_ENCODER_B) == HIGH;
  portENTER_CRITICAL_ISR(&encoderMux);
  encoderStepAccum += (a != b) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

// Touch bring-up only at this stage — same "prove it works before building
// on it" approach as the encoder and display. LilyGo's README says this
// board ships with either CHSC5816 or CST816 touch IC depending on batch;
// CHSC5816 failed init on this actual board (confirmed via the periodic
// status log, 2026-08-06) — consistent with the display turning out to be
// CO5300 rather than SH8601, so trying CST816 next. Declared before the LVGL
// section below since its input-device read callback needs `touch` in scope.
TouchDrvCST816 touch;
bool touchOk = false;

void setupTouch() {
  touch.setPins(TOUCH_RST, TOUCH_INT);
  touchOk = touch.begin(Wire, CST816_SLAVE_ADDRESS, TOUCH_SDA, TOUCH_SCL);
  if (!touchOk) {
    Serial.println("Touch init FAILED (CST816)");
    return;
  }
  Serial.println("Touch init OK");
  Serial.printf("Touch model: %s, chip ID: 0x%02X\n", touch.getModelName(), touch.getChipID());
}

// T30 fix (2026-08-22): incremented on every PRESSED edge (see below).
// screenCycleGestureCb() uses this to recognize "I already handled a
// gesture for this exact touch-down" and ignore a second LV_EVENT_GESTURE
// dispatch for the same physical touch — see that function's own comment
// for the live-capture evidence this fix is based on.
uint32_t touchPressCounter = 0;

void lvTouchpadRead(lv_indev_t *indev, lv_indev_data_t *data) {
  // Deliberately does NOT gate on touch.isPressed(): with an IRQ pin
  // configured (ours is, see setupTouch()), that call does a raw
  // digitalRead() of the interrupt line rather than real touch state (see
  // TouchDrvInterface.hpp's isPressed() doc comment: "suitable for screens
  // with an interrupt pin... For devices without an interrupt pin, simply
  // call getTouchPoints"). The CST816's INT line pulses briefly per touch
  // *report*, it isn't held low for the whole drag, so most ~30ms polls
  // caught it HIGH mid-swipe (looking "released") even with a finger still
  // down — this was the root cause of swipe-up gestures being unreliable
  // (2026-08-06, diagnosed via temporary on-screen instrumentation, since
  // removed — see PROGRESS.md). getTouchPoints() does a genuine fresh I2C
  // register read every call, independent of the interrupt pin, so it alone
  // is enough to detect press state reliably.
  lv_indev_state_t newState = LV_INDEV_STATE_RELEASED;
  if (touchOk) {
    TouchPoints points = touch.getTouchPoints();
    if (points.hasPoints()) {
      const TouchPoint &p = points.getPoint(0);
      newState = LV_INDEV_STATE_PRESSED;
      data->point.x = p.x;
      data->point.y = p.y;
    }
  }
  data->state = newState;

  // T30 debug instrumentation (2026-08-22, temporary — remove once T30's root
  // cause is confirmed or ruled out): edge-triggered log of every press/
  // release transition, so a WebSerial/serial capture during a live repro can
  // show whether a single physical swipe actually produces more than one
  // press-release cycle (a genuine touch-sensing blip mid-swipe, this
  // project's leading unconfirmed theory for T30 — see PROGRESS.md/
  // coordinator-reports/pc-dash.md). Logging every ~30ms poll unconditionally
  // would be far too spammy to read live; logging only on the state actually
  // changing keeps this readable while still catching a real blip.
  static lv_indev_state_t lastLoggedTouchState = LV_INDEV_STATE_RELEASED;
  if (newState != lastLoggedTouchState) {
    lastLoggedTouchState = newState;
    if (newState == LV_INDEV_STATE_PRESSED) {
      touchPressCounter++; // T30 fix, see its own comment above
      lastAnyInteractionMs = millis(); // T40: any touch counts as "not idle" — see lastAnyInteractionMs's own comment
    }
    Serial.printf("touchEdge: t=%ums %s\n", millis(), newState == LV_INDEV_STATE_PRESSED ? "PRESSED" : "RELEASED");
    WebSerial.printf("touchEdge: t=%ums %s\n", millis(), newState == LV_INDEV_STATE_PRESSED ? "PRESSED" : "RELEASED");
  }
}
