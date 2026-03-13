// ########################################
//  02.03.26 - Basic AS5600 reader
//  Measures the angle and just prints the RPM to Serial
//  Commented by ChatGPT, 02.03.26
// ########################################
#pragma once

#include "pico/stdlib.h"
#include "i2c.hpp"

#define todo assert(false)

constexpr uint8_t ENCODER_ADDR = 0x36;  // I2C address of AS5600 encoder
constexpr uint8_t ANGLE_ADDR = 0x0C;    // Register address for raw angle MSB
uint32_t sample_freq = 500;             // Sampling frequency in Hz
uint32_t samplePeriodUS =
    1000000UL / sample_freq;  // Sampling period in microseconds

// Calculates the wrapped delta between two 12-bit angle values
static inline int16_t calc_wrap_delta(uint16_t current, uint16_t prev) {
    int16_t delta = (int16_t)current - (int16_t)prev;
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    return delta;
}

// Reads the raw 12-bit angle value from AS5600 via I2C,
// Returns true if read successful, false otherwise
bool read_raw_angle(uint16_t& raw) {
    // i2c to address, request angle address
    write_byte_i2c1(ENCODER_ADDR, ANGLE_ADDR);

    uint16_t angle;
    i2c_read_blocking(i2c1, ENCODER_ADDR, reinterpret_cast<uint8_t*>(&angle), 2,
                      false);
    // read MSB and LSB
    // uint8_t msb = Wire.read();
    // uint8_t lsb = Wire.read();

    // // Combine MSB (lower 4 bits) and LSB into 12-bit value
    // raw = ((msb & 0x0F) << 8) | lsb;
    return true;
}

// void loop() {
//     uint32_t next_sample_us = micros();
//     bool have_last = false;
//     uint16_t last_raw = 0;

//     uint32_t now_us = micros();

//     // Check if it's time to take a sample
//     if ((now_us - next_sample_us) >= 0) {
//         next_sample_us += samplePeriodUS;

//         uint16_t raw;
//         if (readRawAngle(raw)) {  // First reading: just store the value
//             if (!have_last) {
//                 have_last = true;
//                 last_raw = raw;
//             } else {
//                 int16_t delta_counts = wrapDelta(
//                     raw,
//                     last_raw);  // Calculate angular delta and convert to RPM
//                 last_raw = raw;

//                 float dt_s = samplePeriodUS * 1e-6;
//                 float rps = fabsf((delta_counts / 4096.0f) /
//                                   dt_s);  // Convert counts to revolutions
//                                   per
//                                           // second, then to RPM
//                 float rpm = rps * 60;

//                 Serial.println(rpm);
//             }
//         }
//     }
// }