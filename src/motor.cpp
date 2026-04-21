#include "motor.hpp"

Motor::Motor(uint pwm_pin, uint dir_pin, uint encoder_pin)
    : dir_pin_(dir_pin),
      pwm_channel_(PWM_Channel(pwm_pin)),
      encoder_(Encoder(encoder_pin)) {
    // init dir pin as output I/O pin
    gpio_set_function(dir_pin, GPIO_FUNC_SIO);
    gpio_set_dir(dir_pin, true);
    pwm_channel_.enable();
};

constexpr bool MOTOR_FORWARD = true;
constexpr bool MOTOR_BACKWARD = false;
void Motor::drive(float val) {
    // set direction depending on val sign
    gpio_put(dir_pin_, (val > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD);
    pwm_channel_.set_duty(std::abs(val));
}

void Motor::update_motor_pid() {
    float error = tgt_speed_ - encoder_.get_speed();

    float output = tgt_speed_ + error * MOTOR_P;
}