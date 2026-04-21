#include "encoder.hpp"
#include "timer.hpp"

// Calculates the wrapped delta between two 12-bit angle values
int16_t calc_wrap_delta(uint16_t curr, uint16_t prev);

// handles encoder select pins
void init_encoder_pins() {
    gpio_set_function_masked(PIN_MASK, GPIO_FUNC_SIO);  // set gpio func
    gpio_set_dir_out_masked(PIN_MASK);      // set all masked pins as output
    gpio_put_masked(PIN_MASK, UINT32_MAX);  // set all masked pins high
}

// updates the speed of the encoder by reading I2C + calculation
// returns false if fail
// delta_time in seconds
bool Encoder::update_speed(float delta_time) {
    if (!this->read_raw_angle()) return false;  // failed to read angle
    int16_t delta = calc_wrap_delta(curr_, prev_);

    // 360 degrees split into ±4096
    // speed = delta_degree / time
    speed_rpm_ = delta * (360.0f / 4096.0f) / (delta_time);

#ifdef DEBUG_ENCODER
    DBG("prev: %zu, curr: %zu, delta: %d, delta_time: %f, speed: %f\n", prev_,
        curr_, delta, delta_time, speed_rpm_);
#endif

    prev_ = curr_;  // change prev for next cycle
    return true;
}

// Reads the raw 12-bit angle value from AS5600 via I2C,
// Updates curr_
// Returns true if read successful, false otherwise
// DOES NOT UPDATE curr_ IF INVALID
bool Encoder::read_raw_angle() {
    // select encoder, reset first
    gpio_set_dir_out_masked(PIN_MASK);
    gpio_put_masked(PIN_MASK, UINT32_MAX);  // drive 1s high
    gpio_set_dir(pin, GPIO_IN);  // set dir to input for high impedance

    // read angle MSB and LSB
    uint8_t msb = 0, lsb = 0;
    uint count = 0;
    count += write_byte_i2c1(ENCODER_ADDR, ANGLE_ADDR);
    count += i2c_read_blocking(i2c1, ENCODER_ADDR, &msb, 1, false);
    count += i2c_read_blocking(i2c1, ENCODER_ADDR, &lsb, 1, false);
    if (count != 3) return false;  // not all bytes successfull

    // combine MSB and LSB into uint16
    this->curr_ = ((uint16_t)msb << 8) | lsb;

    return true;
}

int16_t calc_wrap_delta(uint16_t curr, uint16_t prev) {
    // delta in range -4096 to +4096 (maximum of 12-bit integer)
    int16_t delta = (int16_t)curr - (int16_t)prev;
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    return delta;
}