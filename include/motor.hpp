#pragma once
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <math.h>

#include "global.hpp"
#include "pwm.hpp"
#include "encoder.hpp"
#include "timer.hpp"

// Motor Pins:
// assuming we are driving the motors at the same frequency
// need any 4 channels + 4 directional channels, 8 total
// careful to not have overlapping channels
// Front L/R, Back L/R pins
class Motor {
   public:
    Motor(uint pwm_pin, uint dir_pin, uint encoder_pin);

    // sets pwm duty cycle for the motor (0 to ±100)
    // negative values represent reverse
    void drive(float val);

    // sets target speed of the motor
    void set_speed(float speed) { tgt_speed_ = speed; }

    // PID adjust motor duty cycle based on target speed
    void update_motor_pid();

    // prevents reassigning encoder
    Encoder& get_encoder() { return encoder_; }

   private:
    PWM_Channel pwm_channel_;
    const uint dir_pin_;
    Encoder encoder_;

    // target speed
    float tgt_speed_ = 0.0f;
};

// handles overall motor controls
// completely handles encoders and their init
class MotorControl {
   public:
    // initializes controller, related pins are set via defaults
    MotorControl() { init_encoder_pins(); }

    // turns the rover (0 to ±100)
    // negative is left, positive is right
    // 100 would have the motors on either side drive in full reverse
    // linear
    void turn(float turn_strength) {
        float left;
        float right;
    }

    void update_motors() {
        update_encoders();

        return;

        for (auto motor : motor_vec_) {
            motor->update_motor_pid();
        }
    }

   private:
    // preinitialize all motor pins
    Motor motorFL_ = Motor(MOTOR_FL_PWM_PIN, MOTOR_FL_DIR_PIN, ENCODER_FL_PIN);
    Motor motorFR_ = Motor(MOTOR_FR_PWM_PIN, MOTOR_FR_DIR_PIN, ENCODER_FR_PIN);
    Motor motorBL_ = Motor(MOTOR_BL_PWM_PIN, MOTOR_BL_DIR_PIN, ENCODER_BL_PIN);
    Motor motorBR_ = Motor(MOTOR_BR_PWM_PIN, MOTOR_BR_DIR_PIN, ENCODER_BR_PIN);

    // array of motors for looping
    Motor* motor_vec_[4] = {&motorFL_, &motorFR_, &motorBL_, &motorBR_};

    Timer timer_;  // timer used for calculating encoder speed
    // updates the speed for all encoders
    void update_encoders() {
        auto delta_time_us = timer_.clock_us();
        if (delta_time_us == None) return;  // ignore first call

        // unwrap and convert to seconds
        float delta_time = delta_time_us.value() * 1e-6;

        for (auto motor : motor_vec_) {
            motor->get_encoder().update_speed(delta_time);
        }
    }
};
