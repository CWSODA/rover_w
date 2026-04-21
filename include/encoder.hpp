// ########################################
//  02.03.26 - Basic AS5600 reader
//  Measures the angle and just prints the RPM to Serial
//  Commented by ChatGPT, 02.03.26
// ########################################
#pragma once

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "i2c.hpp"

constexpr uint8_t ENCODER_ADDR = 0x36;  // I2C address of AS5600 encoder
constexpr uint8_t ANGLE_ADDR = 0x0C;    // Register address for raw angle MSB

constexpr uint32_t PIN_MASK = (1 << ENCODER_FR_PIN) | (1 << ENCODER_FL_PIN) |
                              (1 << ENCODER_BR_PIN) | (1 << ENCODER_BL_PIN);

class Encoder {
   public:
    Encoder(uint pin) : pin(pin) {}
    const uint pin;

    bool update_speed(float delta_time);
    float get_speed() const { return speed_rpm_; }

   private:
    // Reads the raw 12-bit angle value from AS5600 via I2C,
    // Returns true if read successful, false otherwise
    bool read_raw_angle();

    uint16_t prev_ = 0, curr_ = 0;
    float speed_rpm_ = 0.0f;
};

void init_encoder_pins();