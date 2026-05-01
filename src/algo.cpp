#include "algo.hpp"

#include <vector>
#include <math.h>

#include "settings.hpp"
#include "lidar_objects.hpp"
#include "motor.hpp"

// lidar data is DataPoint from one rotation
// yaw used to calculate target heading
// algorithm needs to control motors
void Algo::update(RotationBuffer& rot_buf, float yaw,
                  MotorControl& motor_ctrl) {
    if (!is_algo_on_) return;            // algo not on
    if (!rot_buf.has_new_buf()) return;  // no new data

    DataBuffer& data = rot_buf.get_complete_buffer();

    for (size_t idx = 0; idx < data.count; idx++) {
        auto& p = data.buf[idx];

        // ignore invalid data
        if (p.distance > DIST_THRESHOLD || p.sig_strength < SIG_STR_THRESHOLD)
            continue;

        // if within a cone
        float fov = 10;
        if (p.angle < fov || p.angle > 360 - fov) {
            // motor_ctrl.drive_forward();
        }
    }
}

// updates motor control with calculated vector
// clears vector afterwards
void Algo::update_motor_ctrl(Vec2& vec, MotorControl& motor_ctrl) {
    // convert back to length + angle (rad)
    float length = sqrtf(vec.x * vec.x + vec.y * vec.y);
    float angle = atan2f(vec.y, vec.x);  // range of -PI to + PI

    // add target vector IF no immediate threat
    float LENGTH_THRESHOLD = 10.0;
    if (length < LENGTH_THRESHOLD) {
        vec += Vec2(0.0f, 1.0f);  // go straight
    }

    float speed = length * SPEED_MULTIPLIER;
    float turn_val = angle * TURN_MULTIPLIER;

    // motor_ctrl.set_speed();
    motor_ctrl.steer(speed, turn_val);

    vec = Vec2(0, 0);  // reset vector
}