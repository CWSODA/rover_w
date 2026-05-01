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
    DBG("Running Algorithm\n");

    DataBuffer& data = rot_buf.get_complete_buffer();

    float FRONT_THESHOLD = 0.4f;
    float FRONT_FOV = 20.0f;
    float SIDE_FOV = 20.0f;

    float front = FRONT_THESHOLD;
    float left = 0.0f;
    float right = 0.0f;
    for (size_t idx = 0; idx < data.count; idx++) {
        auto& p = data.buf[idx];

        // ignore weak signal
        if (p.sig_strength < SIG_STR_THRESHOLD) continue;

        // front cone
        float fov = 40;
        if (p.angle < FRONT_FOV || p.angle > (360 - FRONT_FOV)) {
            front = std::min(front, p.distance);
        }

        // right cone
        if (p.angle > FRONT_FOV && p.angle < FRONT_FOV + SIDE_FOV) {
            right += p.distance;
        }

        // left cone
        if (p.angle < (360.0f - FRONT_FOV) &&
            p.angle < (360.0f - FRONT_FOV - SIDE_FOV)) {
            left += p.distance;
        }
    }
    if (front < FRONT_THESHOLD) {
        // turn towards best direction
        float TURN_SPEED = 50.0f;
        DBG("(%f)(%f)\n", left, right);
        if (left > right) {
            // motor_ctrl.turn_in_place(-TURN_SPEED);
        } else {
            // motor_ctrl.turn_in_place(+TURN_SPEED);
        }
    } else {
        DBG("Nothing in front\n");
        motor_ctrl.drive_forward(50.0f, yaw);
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