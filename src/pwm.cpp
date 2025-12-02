#include "pwm.hpp"

#include <cstdio>
#include <math.h>

PWM_Channel::PWM_Channel(uint pin) : pin_(pin) {
    // set pin as PWM function
    gpio_set_function(pin_, GPIO_FUNC_PWM);

    slice_ = pwm_gpio_to_slice_num(pin_);
    channel_ = pwm_gpio_to_channel(pin_);

    pwm_set_wrap(slice_, top_);
    pwm_set_chan_level(slice_, channel_, 0);

    // prescaler
    pwm_set_clkdiv(slice_, 64.0f);
}

void PWM_Channel::set_duty(float duty) {
    // calculate level value
    uint16_t new_level = roundf(duty / 100.0f * top_);

    // set level
    pwm_set_chan_level(slice_, channel_, new_level);
    printf("Set duty: %f, level: %u, top: %u\n", duty, new_level, top_);

    // ensures count always lower than TOP
    pwm_set_counter(slice_, 0);
}