#pragma once

#include <queue>

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

class Algo {
   public:
    bool is_algo_on_;

    void update(RotationBuffer& rot_buf, float yaw, MotorControl& motor_ctrl);

   private:
    void update_motor_ctrl(Vec2& vec, MotorControl& motor_ctrl);
};

// checks if angle is within range
// all angles should be clamped from (0 to 359°)
inline bool in_range(float angle, float from, float to) {
    if (from < to) {  // normal rotation
        if (angle > from && angle < to) return true;
    } else {  // wrapping rotation
        if (angle > from || angle < to) return true;
    }
    return false;
}

// clamp angle in range (0 to 359°)
inline float normalize_angle(float angle) {
    // note floor rounds down negative numbers so ceil() is used for negative
    // numbers
    int multiple = (angle > 0.0f) ? std::floorf(angle / 360.0f)
                                  : std::ceilf(angle / 360.0f);
    return angle - (multiple * 360.0f);
}