#include "i2c.hpp"

void init_i2c1() {
    i2c_init(i2c1, I2C1_BAUDRATE);
    gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);

    // enable pull ups
    gpio_pull_up(I2C1_SDA_PIN);
    gpio_pull_up(I2C1_SCL_PIN);

    // enable schmitt trigger
    gpio_set_input_hysteresis_enabled(I2C1_SDA_PIN, true);
    gpio_set_input_hysteresis_enabled(I2C1_SCL_PIN, true);

    // enable slew rate limit
    gpio_set_slew_rate(I2C1_SDA_PIN, GPIO_SLEW_RATE_SLOW);
    gpio_set_slew_rate(I2C1_SCL_PIN, GPIO_SLEW_RATE_SLOW);

    // set to master mode
    i2c_set_slave_mode(i2c1, false, 0);
}

// returns false if not all bytes were sent
bool write_bytes_i2c1(const uint8_t addr, const uint8_t* src, size_t len) {
    if (i2c_write_blocking(i2c1, addr, src, len, false) != 1) {
        DBG("Error: no byte was written to I2C1\n");
        return false;
    }
    return true;
}

// returns bytes written, 0 = fail, 1 = success
int write_byte_i2c1(const uint8_t addr, const uint8_t byte) {
    // uint size = i2c_get_write_available(i2c1);
    // i2c_write_byte_raw(i2c1, byte);
    if (i2c_write_blocking(i2c1, addr, &byte, 1, false) != 1) {
        DBG("Error: no byte was written to I2C1\n");
        return 0;
    }
    return 1;
}