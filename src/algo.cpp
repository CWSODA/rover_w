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

    // only calculate heading if yaw of 0° is behind the rover
    // AND if correct heading is off by more than a certain threshold
    bool is_behind =
        in_range(yaw, FRONT_FOV + SIDE_FOV, (360.0f - FRONT_FOV - SIDE_FOV));
    bool is_check_heading = (yaw > YAW_THRESHOLD) && (is_behind);
    is_check_heading = false;
    float heading = DRIVE_DIST_THESHOLD;
    DBG("has heading: %u\n", is_check_heading);

    float front = DRIVE_DIST_THESHOLD;
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
    if (is_check_heading && heading >= DRIVE_DIST_THESHOLD) {
        // prioritize going to heading if it is open
        // yaw < 180 ? turn right, or turn left
        float turn = (yaw < 180.0) ? -TURN_SPEED : TURN_SPEED;
        motor_ctrl.turn_in_place(turn);
        return;
    }
    WDBG("F(%f), L(%f), R(%f)\n", front, left, right);
    if (front >= DRIVE_DIST_THESHOLD) {
        // nothing in front
        motor_ctrl.drive_forward(DRIVE_FORWARD_SPEED, yaw);
        return;
    }

    // turn towards best direction if front is blocked
    if (left > right) {
        motor_ctrl.turn_in_place(-TURN_SPEED);
    } else {
        motor_ctrl.turn_in_place(+TURN_SPEED);
    }
}