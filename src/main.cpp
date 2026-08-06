// First bring-up sketch. Deliberately minimal: only touches pins we've
// confirmed from LilyGo's own pin table (encoder A/B/key), so this proves
// the toolchain + upload path works before any display/touch driver choices
// get made. Display (SH8601A AMOLED) and touch (CHSC5816) come next, once
// this flashes and the encoder reads correctly.

#include <Arduino.h>

constexpr uint8_t PIN_ENCODER_A = 1;
constexpr uint8_t PIN_ENCODER_B = 2;
constexpr uint8_t PIN_ENCODER_KEY = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_KEY, INPUT_PULLUP);
  Serial.println("T-Encoder Pro bring-up: encoder pins ready");
}

void loop() {
  static int lastA = -1;
  static int lastKey = -1;

  int a = digitalRead(PIN_ENCODER_A);
  int key = digitalRead(PIN_ENCODER_KEY);

  if (a != lastA) {
    Serial.printf("A=%d B=%d\n", a, digitalRead(PIN_ENCODER_B));
    lastA = a;
  }
  if (key != lastKey) {
    Serial.printf("KEY=%d\n", key);
    lastKey = key;
  }
}
