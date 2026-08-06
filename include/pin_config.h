// Pin assignments for the LilyGo T-Encoder Pro, taken from LilyGo's own
// README pin table (Xinyuan-LilyGO/T-Encoder-Pro, arduino-esp32-libs_V3.0.7
// branch) — their repo's own examples reference a pin_config.h that doesn't
// actually exist in the repo (a known gap, see their issue #20), so this is
// reconstructed from the README table directly, not copied from a file.
//
// Display driver IC is SH8601 *or* CO5300, and touch IC is CHSC5816 *or*
// CST816, depending on board batch — LilyGo ships both. This board is
// assumed SH8601 + CHSC5816 (matches this project's earlier hardware
// research) until proven otherwise on real hardware.
#pragma once

// Rotary encoder
constexpr uint8_t PIN_ENCODER_A = 1;
constexpr uint8_t PIN_ENCODER_B = 2;
constexpr uint8_t PIN_ENCODER_KEY = 0;

// Buzzer
constexpr uint8_t PIN_BUZZER = 17;

// Display (QSPI) — SH8601/CO5300 AMOLED, 390x390 round
constexpr uint8_t SCREEN_SDIO0 = 11;
constexpr uint8_t SCREEN_SDIO1 = 13;
constexpr uint8_t SCREEN_SDIO2 = 7;
constexpr uint8_t SCREEN_SDIO3 = 14;
constexpr uint8_t SCREEN_SCLK = 12;
constexpr uint8_t SCREEN_CS = 10;
constexpr uint8_t SCREEN_RST = 4;
constexpr uint8_t SCREEN_EN = 3; // VCI EN — screen power enable, must be
                                  // driven HIGH before gfx->begin()
constexpr uint16_t SCREEN_WIDTH = 390;
constexpr uint16_t SCREEN_HEIGHT = 390;

// Touch (I2C) — CHSC5816/CST816
constexpr uint8_t TOUCH_SDA = 5;
constexpr uint8_t TOUCH_SCL = 6;
constexpr uint8_t TOUCH_RST = 8;
constexpr uint8_t TOUCH_INT = 9;
