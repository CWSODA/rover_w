#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "settings.hpp"

void init_i2c1();

int write_byte_i2c1(const uint8_t addr, const uint8_t byte);