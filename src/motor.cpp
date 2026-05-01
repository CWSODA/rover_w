#include "motor.hpp"

#include <math.h>

/* ------------------------------------------------------ */
/*                         MOTORS                         */
/* ------------------------------------------------------ */
Motor::Motor(uint pwm_pin, uint dir_pin)
    : dir_pin_(dir_pin), pwm_channel_(PWM_Channel(pwm_pin)) {
    // init dir pin as output I/O pin
    gpio_set_function(dir_pin, GPIO_FUNC_SIO);
    gpio_set_dir(dir_pin, true);
    pwm_channel_.enable();
    pwm_channel_.set_duty(0);  // start stationary
};

constexpr bool MOTOR_FORWARD = false;
constexpr bool MOTOR_BACKWARD = true;
void Motor::drive(float val) {
    // set direction depending on val sign
    gpio_put(dir_pin_, (val > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD);
    pwm_channel_.set_duty(std::abs(val));
}

/* ------------------------------------------------------ */
/*                      MOTOR CONTROL                     */
/* ------------------------------------------------------ */
void MotorControl::update_motors(float yaw) {
    if (is_disabled_) return;  // ignore if motors disabled
    if (is_manual_) {          // manual drive
        // check timeout if it has not expired yet
        // prevents calling steer if it has been set off before
        if (!manual_drive_timer_.has_expired() &&
            manual_drive_timer_.check_expired()) {
            stop_motors();  // turn off motors
        }
        return;
    }
    if (combo_turn_count_ >= 3) {  // 3 consequtive turns
        jiggle_state_ = 1;
        drive_forward(-JIGGLE_SPEED, yaw);  // drive backwards briefly
        drive_timer_.set_timeout_ms(JIGGLE_TIMEOUT_MS);
    } else if (jiggle_state_ = 1 && drive_timer_.check_expired()) {
        jiggle_state_ = 2;
        drive_forward(JIGGLE_SPEED, yaw);  // drive forwards briefly
        drive_timer_.set_timeout_ms(JIGGLE_TIMEOUT_MS);
    } else if (jiggle_state_ = 2 && drive_timer_.check_expired()) {
        jiggle_state_ = 0;
        stop_motors();
    }

    // correct for straightness ONLY if driving straight
    if (fwd_spd_ == 0.0f) return;

    // calculate yaw error
    float left = fwd_spd_;
    float right = fwd_spd_;
    float yaw_err = yaw - tgt_yaw_;
    yaw_err *= YAW_ERR_P;
    if (yaw_err > 0) {
        left += yaw_err;
    } else {
        right -= yaw_err;
    }
    drive_side(left, right);
}

// drive forward at given speed (0 to 100)
// enables IMU yaw adjusting to drive straight
void MotorControl::drive_forward(float speed, float yaw) {
    if (is_disabled_ && jiggle_state_ != 0)
        return;  // ignore if motors disabled or jiggling
    fwd_spd_ = speed;
    tgt_yaw_ = yaw;

    motorFL_.drive(fwd_spd_);
    motorBL_.drive(fwd_spd_);
    motorFR_.drive(fwd_spd_);
    motorBR_.drive(fwd_spd_);
    combo_turn_count_ = 0;
}

// turns rover in place with given speed
// left = +turn_speed, right = -turn_speed
void MotorControl::turn_in_place(float turn_speed) {
    if (is_disabled_ && jiggle_state_ != 0)
        return;  // ignore if motors disabled or jiggling
    float left = turn_speed;
    float right = -turn_speed;
    motorFL_.drive(left);
    motorBL_.drive(left);
    motorFR_.drive(right);
    motorBR_.drive(right);

    fwd_spd_ = 0.0f;  // disables forward adjusting
    combo_turn_count_++;
}

// speed of each motor from 0 to speed
// negative speed is backwards
// turns the rover, strength between (0 to ±100)
// negative is left, positive is right
// linear
void MotorControl::steer(float speed, float turn_strength) {
    if (is_disabled_ && jiggle_state_ != 0)
        return;  // ignore if motors disabled or jiggling

    // left is max for ranges 100 to 0, linear decrease from 0 to -100
    float left = 1.0f;
    float right = 1.0f;

    // remap from 1.0 to -1.0, turn factor between 0 and 2
    float turn_factor = 2 * abs(turn_strength) / 100.0f;
    if (turn_strength < 0) {  // left turn, reduce left motors
        left -= turn_factor;
    } else {  // right turn, reduce left motors
        right -= turn_factor;
    }

    left *= speed;
    right *= speed;

    motorFL_.drive(left);
    motorBL_.drive(left);
    motorFR_.drive(right);
    motorBR_.drive(right);
}

// steers but has timeout to turn off the motors
// sets "is_manual"
void MotorControl::steer_with_timeout(float speed, float turn_strength) {
    if (is_disabled_ && jiggle_state_ != 0)
        return;  // ignore if motors disabled or jiggling
    is_manual_ = true;

    steer(speed, turn_strength);
    manual_drive_timer_.reset();

    static Timer t;
    float dt = t.clock_us().value_or(0) * 1e-3;

#if DEBUG_MOTOR_LATENCY
    if (dt > 30) {
        WDBG("manual drive elapsed: %fms\n", dt);
    }
#endif
}