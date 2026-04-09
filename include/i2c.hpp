#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "global.hpp"

constexpr uint I2C1_BAUDRATE = 100e3;  // max 1M for encoder
constexpr uint I2C1_SDA_PIN = 2;
constexpr uint I2C1_SCL_PIN = 3;

void init_i2c1();

void write_byte_i2c1(const uint8_t addr, const uint8_t byte);