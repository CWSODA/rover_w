#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

constexpr uint I2C1_BAUDRATE = 100e3;
constexpr uint I2C1_SDA_PIN = 2;
constexpr uint I2C1_SCL_PIN = 3;

void init_i2c1() {
    i2c_init(i2c1, I2C1_BAUDRATE);
    gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA_PIN);
    gpio_pull_up(I2C1_SCL_PIN);
    i2c_set_slave_mode(i2c1, false, 0);  // set to master mode
}

void write_byte_i2c1(const uint8_t addr, const uint8_t byte) {
    // uint size = i2c_get_write_available(i2c1);
    // i2c_write_byte_raw(i2c1, byte);

    i2c_write_blocking(i2c1, addr, &byte, 1, false);
}