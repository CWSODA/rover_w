#include "motor.hpp"

/* ------------------------------------------------------ */
/*                         MOTORS                         */
/* ------------------------------------------------------ */
Motor::Motor(uint pwm_pin, uint dir_pin, uint encoder_pin)
    : dir_pin_(dir_pin),
      pwm_channel_(PWM_Channel(pwm_pin)),
      encoder_(Encoder(encoder_pin)) {
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

void Motor::update_motor_pid() {
    float error = tgt_speed_ - encoder_.get_speed();

    float output = tgt_speed_ + error * MOTOR_P;
}

/* ------------------------------------------------------ */
/*                      MOTOR CONTROL                     */
/* ------------------------------------------------------ */
void MotorControl::update_motors() {
    if (is_algo_on) {
        // run algorithm
    } else if (is_manual) {
        // manual drive
        // check timeout if it has not expired yet
        if (!manual_drive_timer_.has_expired() &&
            manual_drive_timer_.check_expired()) {
            steer(0, 0);  // turn off motors
        }
    }

    // update_encoders();
    // for (auto& motor : motor_vec_) {
    //     motor->update_motor_pid();
    // }
}

void MotorControl::update_encoders() {
    auto delta_time_us = timer_.clock_us();
    if (delta_time_us == None) return;  // ignore first call

    // unwrap and convert to seconds
    float delta_time = delta_time_us.value() * 1e-6;

    for (auto motor : motor_vec_) {
        motor->get_encoder().update_speed(delta_time);
    }
}

// speed of each motor from 0 to speed
// negative speed is backwards
// turns the rover, strength between (0 to ±100)
// negative is left, positive is right
// linear
void MotorControl::steer(float speed, float turn_strength) {
    // float speed_abs = abs(speed);

    // left is max for ranges 100 to 0, linear decrease from 0 to -100
    float left = 1.0f;
    float right = 1.0f;

    // remap from 1.0 to -1.0, turn factor between 0 and 2
    float turn_factor = 2 * turn_strength / 100.0f;
    if (turn_strength < 0) {  // left turn, reduce left motors
        left -= turn_factor;
    } else {  // right turn, reduce left motors
        right -= turn_factor;
    }

    left *= speed;
    right *= speed;

    motorFL_.set_target_speed(left);
    motorBL_.set_target_speed(left);
    motorFR_.set_target_speed(right);
    motorBR_.set_target_speed(right);
}

void MotorControl::steer_with_timeout(float speed, float turn_strength) {
    is_algo_on = false;
    is_manual = true;

    steer(speed, turn_strength);
    manual_drive_timer_.reset();

    static Timer t;
    float dt = t.clock_us().value_or(0) * 1e-3;
    if (dt > 30) {
        WDBG("elapsed: %fms\n", dt);
    }
}