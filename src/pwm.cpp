#include "pwm.hpp"

#include <cstdio>
#include <math.h>

PWM_Channel::PWM_Channel(uint pin) : pin_(pin) {
    gpio_set_function(pin_, GPIO_FUNC_PWM);  // set pin as PWM function
    gpio_set_dir(pin, true);                 // set pin dir to output

    slice_ = pwm_gpio_to_slice_num(pin_);  // get slice
    channel_ = pwm_gpio_to_channel(pin_);  // get channel

    pwm_set_wrap(slice_, top_);
    pwm_set_chan_level(slice_, channel_, 0);

    // prescaler
    // clock speed of about 120 MHz
    // 120e6 / 2^16 / 64 = 28 Hz
    // 120e6 / 2^16 = 1831 Hz
    pwm_set_clkdiv(slice_, 1.0f);
}

void PWM_Channel::set_duty(float duty) {
    // calculate level value
    uint16_t new_level = roundf(duty / 100.0f * top_);

    // set level
    pwm_set_chan_level(slice_, channel_, new_level);

#if DEBUG_PWM
    if (duty != 0) {
        DBG("Set duty: %f, level: %zu, top: %zu\n", duty, new_level, top_);
    }
#endif

    // ensures count always lower than level
    // if (pwm_get_counter(slice_) > new_level) {
    //     pwm_set_counter(slice_, 0);
    // }
}