// ########################################
//  02.03.26 - Basic AS5600 reader
//  Measures the angle and just prints the RPM to Serial
//  Commented by ChatGPT, 02.03.26
// ########################################
#pragma once

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "i2c.hpp"

#define todo assert(false)

constexpr uint8_t ENCODER_ADDR = 0x36;  // I2C address of AS5600 encoder
constexpr uint8_t ANGLE_ADDR = 0x0C;    // Register address for raw angle MSB

// pin numbers of encoder
enum class EncoderSel {
    FL = 18,
    FR = 19,
    BL = 20,
    BR = 21,
};
constexpr uint FL_PIN = (uint)EncoderSel::FL;
constexpr uint FR_PIN = (uint)EncoderSel::FR;
constexpr uint BL_PIN = (uint)EncoderSel::BL;
constexpr uint BR_PIN = (uint)EncoderSel::BR;
constexpr uint32_t PIN_MASK =
    (1 << FR_PIN) | (1 << FL_PIN) | (1 << BR_PIN) | (1 << BL_PIN);

class Encoder {
   public:
    Encoder(uint pin) : pin(pin) {}
    const uint pin;

    // private:
    uint16_t prev = 0, curr = 0;
    float speed_rpm = 0.0f;
};

void init_encoder();

void update_encoders();

// Reads the raw 12-bit angle value from AS5600 via I2C,
// Returns true if read successful, false otherwise
bool read_raw_angle(uint16_t& output, uint pin);

// Calculates the wrapped delta between two 12-bit angle values
int16_t calc_wrap_delta(uint16_t current, uint16_t prev);