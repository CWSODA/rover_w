#pragma once
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <math.h>
#include <queue>

#include "settings.hpp"
#include "pwm.hpp"
#include "timer.hpp"
#include "lidar_parser.hpp"

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

    // PID adjust motor duty cycle based on target speed
    void update_motor_pid();

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

    // turns rover in place with given speed
    // left = +turn_speed, right = -turn_speed
    void turn_in_place(float turn_speed) {
        float left = turn_speed;
        float right = -turn_speed;
        motorFL_.drive(left);
        motorBL_.drive(left);
        motorFR_.drive(right);
        motorBR_.drive(right);

        fwd_spd_ = 0.0f;  // disables forward adjusting
    }

    // drives forward, sets fwd_speed to non_zero value
    void drive_forward(float speed) {
        fwd_spd_ = speed;
        motorFL_.drive(fwd_spd_);
        motorBL_.drive(fwd_spd_);
        motorFR_.drive(fwd_spd_);
        motorBR_.drive(fwd_spd_);
    }

    // motor update
    // runs algorithm (lidar data required) runs motor PID
    void update_motors(std::queue<DataPoint>& lidar_data);

    // turns the rover (0 to ±100)
    // negative is left, positive is right
    // 100 would have the motors on either side drive in full reverse
    // linear
    void steer(float speed, float turn_strength);

    // TCP controls
    void steer_with_timeout(float speed, float turn_strength);
    void enable_algo() {
        is_algo_on = true;
        is_manual = false;
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

    bool is_algo_on = false;
    bool is_manual = false;

    // timer used for control timeout
    TimeoutTimer manual_drive_timer_ = TimeoutTimer(MANUAL_DRIVE_TIMEOUT_MS);

    // other
    float fwd_spd_ = 50.0f;  // if zero, dont adjust for straight
};
