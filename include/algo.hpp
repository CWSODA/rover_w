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

void update_motor_ctrl(Vec2& vec, MotorControl& motor_ctrl);

void run_algorithm(std::queue<DataPoint>& lidar_data, MotorControl& motor_ctrl);

// updates motor control with calculated vector
// clears vector afterwards
void update_motor_ctrl(Vec2& vec, MotorControl& motor_ctrl);