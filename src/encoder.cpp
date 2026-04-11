#include "encoder.hpp"

// must have i2c initated first!!!
void init_encoder() {
    gpio_set_function_masked(PIN_MASK, GPIO_FUNC_SIO);  // set gpio func
    gpio_set_dir_out_masked(PIN_MASK);      // set all masked pins as output
    gpio_put_masked(PIN_MASK, UINT32_MAX);  // set all masked pins high
}

Encoder encoder_arr[4] = {
    Encoder(ENCODER_FL_PIN),
    Encoder(ENCODER_FR_PIN),
    Encoder(ENCODER_BL_PIN),
    Encoder(ENCODER_BR_PIN),
};

// void update_encoders() {
//     static auto prev_time = get_absolute_time();
//     auto time = get_absolute_time();
//     float delta_time_us = absolute_time_diff_us(prev_time, time);
//     prev_time = time;

//     for (auto& encoder : encoder_arr) {
//         read_raw_angle(encoder.curr, encoder.pin);

//         int16_t delta = calc_wrap_delta(encoder.curr, encoder.prev);
//         DBG("(c: %d)(p: %d)(d: %d)\n", encoder.curr, encoder.prev, delta);
//         encoder.prev = encoder.curr;

//         // speed = angle / time
//         encoder.speed_rpm = delta * (360.0f / 4096.0f) / (delta_time_us *
//         1e-6); printf("\nspeed: %f\n", encoder.speed_rpm);
//     }
// }

void update_encoders() {
    for (auto& encoder : encoder_arr) {
        read_raw_angle(encoder.curr, encoder.pin);

        DBG("Encoder on pin %d read: %zu\n", encoder.pin, encoder.curr);
    }
}

// Reads the raw 12-bit angle value from AS5600 via I2C,
// Returns true if read successful, false otherwise
bool read_raw_angle(uint16_t& output, uint pin) {
    // select encoder, reset first
    gpio_set_dir_out_masked(PIN_MASK);
    gpio_put_masked(PIN_MASK, UINT32_MAX);  // drive 1s high
    gpio_set_dir(pin, GPIO_IN);  // set dir to input for high impedance

    // read angle MSB and LSB
    uint8_t msb = 0, lsb = 0;
    write_byte_i2c1(ENCODER_ADDR, ANGLE_ADDR);
    i2c_read_blocking(i2c1, ENCODER_ADDR, &msb, 1, false);
    i2c_read_blocking(i2c1, ENCODER_ADDR, &lsb, 1, false);

    // combine MSB and LSB into uint16
    output = ((uint16_t)msb << 8) | lsb;

    return true;
}

int16_t calc_wrap_delta(uint16_t current, uint16_t prev) {
    // delta in range -4096 to +4096 (maximum of 12-bit integer)
    int16_t delta = (int16_t)current - prev;
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    return delta;
}