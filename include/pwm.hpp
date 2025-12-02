#pragma once
#include "pico/stdlib.h"
#include "hardware/pwm.h"

class PWM_Channel {
   public:
    // initializes PWM
    // PWM channel / slice are hardware defined and CANNOT change
    // choose correct pins if seperate channels are desired
    // default duty cycle of 0%
    PWM_Channel(uint pin);

    // DISABLES SLICE IF FALSE (NOT CHANNEL)
    // To disable channel, set duty to 0
    void enable(bool val = true) { pwm_set_enabled(slice_, true); }

    // sets duty cycle
    // make sure duty is between 0 and 100
    void set_duty(float duty);

    // debug
    uint get_slice() { return slice_; }
    uint get_channel() { return channel_; }

   private:
    uint pin_;
    uint slice_;
    uint channel_;

    float prescaler_ = 1.0f;
    float top_ = UINT16_MAX;
};