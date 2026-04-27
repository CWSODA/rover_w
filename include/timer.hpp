#pragma once

#include "pico/stdlib.h"
#include <optional>

// cursed rust syntax
using Some = std::optional<float>;
inline constexpr Some None = std::nullopt;

// for measuring elapsed time between each call
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

// for doing stuff every interval
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
        if (dt * 1E-3 >= cooldown_ms_) {
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

// for checking if an amount of time has passed
class TimeoutTimer {
   public:
    // creates timeout timer given timeout in ms
    TimeoutTimer(float timeout_ms = 1000.0f) : timeout_ms_(timeout_ms) {}

    // checks if timer has expired
    bool check_expired() {
        if (has_expired_ == true) return true;

        auto time = get_absolute_time();
        float dt = absolute_time_diff_us(start_time_, time) * 1e-3;
        if (dt > timeout_ms_) {
            has_expired_ = true;
            WDBG("timeout: %f\n", dt);
            return true;
        }
        return false;
    }

    // refresh timer
    void reset() {
        start_time_ = get_absolute_time();
        has_expired_ = false;
    }

    // change timeout time
    void set_timeout_ms(float timeout_ms) { timeout_ms_ = timeout_ms; }

    bool has_expired() { return has_expired_; };

   private:
    absolute_time_t start_time_;
    float timeout_ms_;
    bool has_expired_ = true;
};