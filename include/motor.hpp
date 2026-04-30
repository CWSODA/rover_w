#pragma once
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include "settings.hpp"
#include "pwm.hpp"
#include "timer.hpp"

// Motor Pins:
// assuming we are driving the motors at the same frequency
// need any 4 channels + 4 directional channels, 8 total
// careful to not have overlapping channels
// Front L/R, Back L/R pins
class Motor {
   public:
    Motor(uint pwm_pin, uint dir_pin);

    // sets pwm duty cycle for the motor (0 to ±100)
    // negative values represent reverse
    void drive(float val);

   private:
    PWM_Channel pwm_channel_;
    const uint dir_pin_;
};

// handles overall motor controls
// completely handles encoders and their init
class MotorControl {
   public:
    // initializes controller, related pins are set via defaults
    MotorControl() {}

    // allows IMU to disable and enable motors
    void disable() {
        stop_motors();
        is_disabled_ = true;
    }
    void enable() { is_disabled_ = false; }

    // new gen functions :>
    void turn_in_place(float turn_speed);
    void drive_forward(float speed, float yaw);
    void stop_motors() {
        fwd_spd_ = 0.0f;
        motorFL_.drive(fwd_spd_);
        motorBL_.drive(fwd_spd_);
        motorFR_.drive(fwd_spd_);
        motorBR_.drive(fwd_spd_);
    }

    void update_motors(float yaw);

    void steer(float speed, float turn_strength);

    // TCP controls
    void steer_with_timeout(float speed, float turn_strength);
    void disable_manual() {  // disable manual and stops motors
        is_manual_ = false;
        stop_motors();
    }

    void test() {
        motorBL_.drive(30);
        motorBR_.drive(60);
    }

   private:
    // preinitialize all motor pins
    Motor motorFL_ = Motor(MOTOR_FL_PWM_PIN, MOTOR_FL_DIR_PIN);
    Motor motorFR_ = Motor(MOTOR_FR_PWM_PIN, MOTOR_FR_DIR_PIN);
    Motor motorBL_ = Motor(MOTOR_BL_PWM_PIN, MOTOR_BL_DIR_PIN);
    Motor motorBR_ = Motor(MOTOR_BR_PWM_PIN, MOTOR_BR_DIR_PIN);

    // array of motors for looping
    Motor* motor_vec_[4] = {&motorFL_, &motorFR_, &motorBL_, &motorBR_};

    bool is_disabled_ = true;  // disable from IMU and emergency stop

    // manual TCP control
    TimeoutTimer manual_drive_timer_ = TimeoutTimer(MANUAL_DRIVE_TIMEOUT_MS);
    bool is_manual_ = false;

    // algo drive vars
    TimeoutTimer drive_timer_;  // used to time driving forward and turning
    float fwd_spd_ = 0.0f;      // if zero, dont adjust for straight
    float tgt_yaw_ = 0.0f;      // yaw before driving forwards
};
