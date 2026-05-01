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
    if (!is_algo_on) return;             // algo not on
    if (!rot_buf.has_new_buf()) return;  // no new data
    DBG("ALGO\n");

    DataBuffer& data = rot_buf.get_complete_buffer();

    // normalize yaw to range (0 to 359°)
    yaw = normalize_angle(yaw);

    // only calculate heading if yaw of 0° is behind the rover
    // AND if correct heading is off by more than a certain threshold
    bool is_behind =
        in_range(yaw, FRONT_FOV + SIDE_FOV, (360.0f - FRONT_FOV - SIDE_FOV));
    bool is_check_heading = (yaw > YAW_THRESHOLD) && (is_behind);
    float heading = DRIVE_DIST_THESHOLD;

    if (!is_heading) is_check_heading = false;  // turn off heading
    // DBG("has heading: %u\n", is_check_heading);

    float front_closest = DRIVE_DIST_THESHOLD;
    float right_closest = 12.0f;
    float left_closest = 12.0f;
    float left = 0.0f;
    float right = 0.0f;
    for (size_t idx = 0; idx < data.count; idx++) {
        auto& p = data.buf[idx];
        if (p.sig_strength < SIG_STR_THRESHOLD) continue;  // ignore weak signal

        /* -------------------- HEADING CONE -------------------- */
        if (is_check_heading)
            if (in_range(p.angle, yaw - HEADING_FOV, yaw + HEADING_FOV))
                heading = std::min(heading, p.distance);
        /* --------------------- FRONT CONE --------------------- */
        if (in_range(p.angle, 360.0f - FRONT_FOV, FRONT_FOV)) {
            front_closest = std::min(front_closest, p.distance);
            DBG("%u, %f, %f\n", p.sig_strength, p.angle, p.distance);
        }
        /* --------------------- RIGHT CONE --------------------- */
        if (in_range(p.angle, FRONT_FOV, FRONT_FOV + SIDE_FOV)) {
            right += p.distance;
            right_closest = std::min(right_closest, p.distance);
        }
        /* ---------------------- LEFT CONE --------------------- */
        if (in_range(p.angle, 360.0f - FRONT_FOV - SIDE_FOV,
                     360.0f - FRONT_FOV)) {
            left += p.distance;
            left_closest = std::min(left_closest, p.distance);
        }
    }
    DBG("C: (%f)(%f)(%f)\n", front_closest, left_closest, right_closest);
    if (front_closest <= MIN_TURNABLE_DIST) {
        // move backwards for space to turn
        motor_ctrl.drive_forward(-DRIVE_FORWARD_SPEED, yaw);
    }
    if (is_check_heading && heading >= DRIVE_DIST_THESHOLD) {
        // prioritize going to heading if it is open
        // yaw < 180 ? turn right, or turn left
        float turn = (yaw < 180.0) ? TURN_SPEED : -TURN_SPEED;
        motor_ctrl.turn_in_place(turn);
        return;
    }
    if (front_closest >= DRIVE_DIST_THESHOLD) {  // nothing in front
        motor_ctrl.drive_forward(DRIVE_FORWARD_SPEED, yaw);
        DBG("F\n");
        return;
    }
    // turn towards best direction if front is blocked
    bool left_valid = left_closest > DRIVE_DIST_THESHOLD;
    bool right_valid = right_closest > DRIVE_DIST_THESHOLD;
    if (left_valid && right_valid) {
        // use closest for tie breaker
        float turn = TURN_SPEED * ((left_closest > right_closest) ? -1 : 1);
        motor_ctrl.turn_in_place(turn);
        return;
    }
    if (left_valid) {
        motor_ctrl.turn_in_place(-TURN_SPEED);
        return;
    }
    if (right_valid) {
        motor_ctrl.turn_in_place(TURN_SPEED);
        return;
    }
    // nothing valid go backwards
    motor_ctrl.drive_forward(-50.0f, 0.0f);
}