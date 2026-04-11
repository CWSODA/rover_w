#pragma once
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <math.h>

#include "global.hpp"
#include "pwm.hpp"
#include "encoder.hpp"

// Motor Pins:
// assuming we are driving the motors at the same frequency
// need any 4 channels + 4 directional channels, 8 total
// careful to not have overlapping channels
// Front L/R, Back L/R pins

class Motor {
   public:
    Motor(uint pwm_pin, uint dir_pin)
        : dir_pin_(dir_pin), pwm_channel_(PWM_Channel(pwm_pin)) {
        assert(dir_pin != pwm_pin);
        // init dir pin as output I/O pin
        gpio_set_function(dir_pin, GPIO_FUNC_SIO);
        gpio_set_dir(dir_pin, true);
        pwm_channel_.enable();
    }

    // sets pwm for the motor
    // value is set as the duty cycle
    // negative values represent reverse
    void drive(float val) {
        // set direction depending on val sign
        gpio_put(dir_pin_, (val > 0) ? true : false);
        pwm_channel_.set_duty(std::abs(val));
    }

   private:
    PWM_Channel pwm_channel_;
    const uint dir_pin_;

    float set_speed = 0.0f;
};

class MotorControl {
   public:
    void update_motors() {
        static float val = 100.0f;
        motorFL.drive(val);
        motorFR.drive(val);
        motorBL.drive(val);
        motorBR.drive(val);
    }

   private:
    Motor motorFL = Motor(MOTOR_FL_PWM_PIN, MOTOR_FL_DIR_PIN);
    Motor motorFR = Motor(MOTOR_FR_PWM_PIN, MOTOR_FR_DIR_PIN);
    Motor motorBL = Motor(MOTOR_BL_PWM_PIN, MOTOR_BL_DIR_PIN);
    Motor motorBR = Motor(MOTOR_BR_PWM_PIN, MOTOR_BR_DIR_PIN);
};
