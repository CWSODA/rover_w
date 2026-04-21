#include <vector>
#include <math.h>

#include "settings.hpp"

struct Polar {
    float distance = 0.0f;  // distance in meters
    float angle = 0.0f;     // angle in RADIANS!!!

    Polar(float dist, float angle) : distance(dist), angle(angle) {}
};

struct Vec2 {
    float x, y;

    Vec2(float x, float y) : x(x), y(y) {}
    Vec2 operator+(const Vec2& other) { return Vec2(x + other.x, y + other.y); }
    void operator+=(const Vec2& other) {
        this->x += other.x;
        this->y + other.y;
    }
    Vec2 operator*(const float factor) { return Vec2(x * factor, y * factor); }
    void operator*=(const float factor) {
        this->x *= factor;
        this->y *= factor;
    }
};

void calc_vec(std::vector<Polar> lidar_data) {
    // lidar inputs angle + distance
    // sums -angle for repulsion
    // modulated by 1 / distance
    // can be quadratic: 1 / (ax^2 + bx + c)
    // depending on how much it needs to be tuned

    // calculate overall vector
    Vec2 vector(0, 0);

    for (auto& data : lidar_data) {
        // filter data by distance, can do this in parser
        if (data.distance > DIST_THRESHOLD) return;

        // convert to vector form
        Vec2 data_vec(cosf(data.angle), sinf(data.angle));

        // 1/(x + 1) modulation
        vector += data_vec * (1.0f / (data.distance + 1.0f));
    }

    // convert back to length + angle (rad)
    float length = sqrtf(vector.x * vector.x + vector.y * vector.y);
    float angle = atan2f(vector.y, vector.x);  // range of -PI to + PI

    // float turn = angle * multiplier;

    // slow down if angle is high
    // float speed = length * multiplier / (angle + 1);
}