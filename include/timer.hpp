#include "pico/stdlib.h"

#include <optional>

// cursed rust syntax
using Some = std::optional<float>;
inline constexpr Some None = std::nullopt;

class Timer {
   public:
    // returns time between last call in us
    // returns 0 if first call (invalid call)!!!
    std::optional<float> clock_us() {
        auto time = get_absolute_time();
        // ignore first call
        if (is_first_) {
            prev_time_ = time;
            is_first_ = false;
            return None;
        }
        float delta_time_us = absolute_time_diff_us(prev_time_, time);
        prev_time_ = time;

        return Some(delta_time_us);
    }

   private:
    absolute_time_t prev_time_;
    bool is_first_ = true;
};

class CooldownTimer {
   public:
    CooldownTimer(float cooldown_ms) : cooldown_ms_(cooldown_ms) {}

    // returns true if elapsed time is past cooldown
    // auto resets cooldown
    // always true on first call!
    bool check() {
        auto time = get_absolute_time();
        if (is_first_) {
            prev_time_ = time;
            is_first_ = false;
            return true;
        }

        float dt = absolute_time_diff_us(prev_time_, time);
        if (dt * 1E3 >= cooldown_ms_) {
            prev_time_ = time;
            return true;
        }

        return false;
    }

   private:
    absolute_time_t prev_time_;
    float cooldown_ms_;
    bool is_first_ = true;
};