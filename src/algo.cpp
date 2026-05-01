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

    // normalize yaw to range (0 to 359°)
    yaw = normalize_angle(yaw);

    float DRIVE_DISTANCE_THESHOLD = 0.4f;
    float FRONT_FOV = 30.0f;  // shared with heading cone
    float SIDE_FOV = 30.0f;
    float YAW_THRESHOLD = 10.0f;
    float HEADING_FOV = 30.0f;
    float TURN_SPEED = 50.0f;

    // only calculate heading if yaw of 0° is behind the rover
    // AND if correct heading is off by more than a certain threshold
    bool is_behind =
        in_range(yaw, FRONT_FOV + SIDE_FOV, (360.0f - FRONT_FOV - SIDE_FOV));
    bool is_check_heading = (yaw > YAW_THRESHOLD) && (is_behind);
    float heading = DRIVE_DISTANCE_THESHOLD;
    DBG("has heading: %u\n", is_check_heading);

    float front = DRIVE_DISTANCE_THESHOLD;
    float left = 0.0f;
    float right = 0.0f;
    for (size_t idx = 0; idx < data.count; idx++) {
        auto& p = data.buf[idx];

        // ignore weak signal
        if (p.sig_strength < SIG_STR_THRESHOLD) continue;

        // heading cone
        if (is_check_heading) {
            if (in_range(p.angle, yaw - HEADING_FOV, yaw + HEADING_FOV)) {
                heading = std::min(heading, p.distance);
            }
        }

        // front cone
        if (in_range(p.angle, 360.0f - FRONT_FOV, FRONT_FOV)) {
            front = std::min(front, p.distance);
        }

        // right cone
        if (in_range(p.angle, FRONT_FOV, FRONT_FOV + SIDE_FOV)) {
            right += p.distance;
        }

        // left cone
        if (in_range(p.angle, 360.0f - FRONT_FOV - SIDE_FOV,
                     360.0f - FRONT_FOV)) {
            left += p.distance;
        }
    }
    if (heading > DRIVE_DISTANCE_THESHOLD) {
        // prioritize going to heading if it is open
        // yaw < 180 ? turn right, or turn left
        float turn = (yaw < 180.0) ? -TURN_SPEED : TURN_SPEED;
        motor_ctrl.turn_in_place(turn);
        return;
    }
    if (front > DRIVE_DISTANCE_THESHOLD) {
        // nothing in front
        motor_ctrl.drive_forward(50.0f, yaw);
        return;
    }

    // turn towards best direction if front is blocked
    if (left > right) {
        motor_ctrl.turn_in_place(-TURN_SPEED);
    } else {
        motor_ctrl.turn_in_place(+TURN_SPEED);
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