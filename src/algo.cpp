#include <vector>
#include <math.h>

#include "settings.hpp"
#include "lidar_parser.hpp"
#include "motor.hpp"

struct Vec2 {
    float x, y;

    Vec2(float x, float y) : x(x), y(y) {}
    Vec2 operator+(const Vec2& other) { return Vec2(x + other.x, y + other.y); }
    void operator+=(const Vec2& other) {
        this->x += other.x;
        this->y += other.y;
    }
    void operator-=(const Vec2& other) {
        this->x -= other.x;
        this->y -= other.y;
    }
    Vec2 operator*(const float factor) { return Vec2(x * factor, y * factor); }
    void operator*=(const float factor) {
        this->x *= factor;
        this->y *= factor;
    }
};

void update_motor_ctrl(Vec2& vec, MotorControl& motor_ctrl);

OPT;
// can use queue<list> instead of queue<deque> because FIFO data is always read
// sequentially
void run_algorithm(std::queue<DataPoint>& lidar_data,
                   MotorControl& motor_ctrl) {
    static Vec2 calc_vec(0, 0);       // vector used for calculation
    static float last_angle = -1.0f;  // -1 ensures no angle will be smaller

    // empty all vectors
    while (!lidar_data.empty()) {
        DataPoint data = lidar_data.front();  // get oldest
        lidar_data.pop();                     // and pop to remove

        // check for within threshold
        bool is_dist = data.distance < DIST_THRESHOLD;
        bool is_str = data.sig_strength > SIG_STR_THRESHOLD;
        bool is_fov = data.angle <= FOV && data.angle >= (360 - FOV);
        if (is_dist && is_str && is_fov) {
            // convert to vector form for summing
            float angle_in_rad = data.angle * 3.1415f / 180.0f;
            Vec2 data_vec(cosf(angle_in_rad), sinf(angle_in_rad));

            // attenuate effect based on distance
            // negative to repel
            calc_vec -= data_vec * (1.0f / (data.distance + 1.0f));
        }

        // check if angle has wrapped back to 0, indicating full rotation
        if (data.angle < last_angle) {
            /* if multiple rotations within same update loop, only
            calculate the later one. Although this should be pretty rare
            since update loop should be much faster than the LiDAR */
            OPT;

            update_motor_ctrl(calc_vec, motor_ctrl);
            last_angle = -1.0f;  // -1 ensures no angle will be smaller
        } else {
            last_angle = data.angle;  // update angle
        }
    }
}

// updates motor control with calculated vector
// clears vector afterwards
void update_motor_ctrl(Vec2& vec, MotorControl& motor_ctrl) {
    // add target vector
    // straight for testing
    vec += Vec2(0.0f, 1.0f);

    // convert back to length + angle (rad)
    float length = sqrtf(vec.x * vec.x + vec.y * vec.y);
    float angle = atan2f(vec.y, vec.x);  // range of -PI to + PI

    float speed = length * SPEED_MULTIPLIER;
    float turn_val = angle * TURN_MULTIPLIER;

    // motor_ctrl.set_speed();
    motor_ctrl.steer(speed, turn_val);

    vec = Vec2(0, 0);  // reset vector
}