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