#pragma once

#include <vector>

#include "settings.hpp"
#include "pwm.hpp"
#include "timer.hpp"

enum class LED_INDICATOR {
    POWER_ON,
    LOOP_NO_WIFI,
    LOOP_WITH_WIFI,
    MOTOR_ERR,
    GYRO_CALIBRATION,
};

struct Color {
    float r, g, b;
    Color(float red, float green, float blue) {
        r = red;
        g = green;
        b = blue;
    }

    void rainbow_update() {
        r += 1;
        g += 1;
        b += 1;
        if (r > 100) r = 0;
        if (g > 100) g = 0;
        if (b > 100) b = 0;
    }
};

class LED {
   public:
    // sets default indicator
    bool has_wifi = false;

    // initiates pwm channels and turns light red
    LED() {
        pwm_r.enable();
        pwm_g.enable();
        pwm_b.enable();
        set_indicator(LED_INDICATOR::POWER_ON);
    }

    // changes LED duty cycle, (0 -> 100)
    void set(Color color) {
        pwm_r.set_duty(color.r);
        pwm_g.set_duty(color.g);
        pwm_b.set_duty(color.b);
    }

    // sets default loop indicators depending on if there is wifi
    void set_default() {
        set_indicator(has_wifi ? LED_INDICATOR::LOOP_WITH_WIFI
                               : LED_INDICATOR::LOOP_NO_WIFI);
    }

    // changes LED with given indicator
    // warning indicators will flash for some time
    void set_indicator(LED_INDICATOR opt) {
        switch (opt) {
            case LED_INDICATOR::POWER_ON: {
                set(Color(100, 0, 0));  // red
                break;
            }
            case LED_INDICATOR::LOOP_NO_WIFI: {
                set(Color(0, 100, 0));  // green
                break;
            }
            case LED_INDICATOR::LOOP_WITH_WIFI: {
                setup_led_flash(
                    std::vector<Color>{Color(0, 100, 0), Color(0, 0, 100)}, -1,
                    500);
                break;
            }
            case LED_INDICATOR::MOTOR_ERR: {
                setup_led_flash(
                    std::vector<Color>{Color(100, 0, 0), Color(0, 0, 0)}, 10,
                    300);
                break;
            }
            case LED_INDICATOR::GYRO_CALIBRATION: {
                setup_led_rainbow(IMU_GYRO_CALIBRATION_TIME_MS);
                break;
            }
            default: {
                set(Color(100, 0, 100));  // purple
            }
        }
    }

    void update() {
        if (is_rainbow_) {
            if (flash_cd_.check()) {
                is_rainbow_ = false;
                set_default();
                return;
            }
            // do the rainbow :3
            if (!rainbow_cd_.check()) return;  // not yet
            rainbow_.rainbow_update();
            set(rainbow_);
            return;
        }
        if (count_ == 0) return;         // dont change LEDs
        if (!flash_cd_.check()) return;  // wait more

        // switch to next color
        idx_++;
        if (idx_ >= colors_.size()) {
            idx_ = 0;                  // wrap around to zero
            if (count_ > 0) count_--;  // decrement count if not infinite
            if (count_ == 0) {
                set_default();
            }
        }
        set(colors_[idx_]);
    }

   private:
    CooldownTimer flash_cd_ = CooldownTimer(1.0f);
    std::vector<Color> colors_;
    size_t idx_ = 0;
    int count_ = 0;

    // sets up LED flashing alternating colors n times at set intervals
    // set n to -1 for indefinate
    void setup_led_flash(std::vector<Color> colors, int n, float interval_ms) {
        colors_ = std::move(colors);  // dont copy vector, move it
        count_ = n;
        flash_cd_.set_cd_ms(interval_ms);

        set(colors_[0]);
        idx_ = 0;
    }

    Color rainbow_ = Color(33, 66, 0);
    bool is_rainbow_ = false;
    CooldownTimer rainbow_cd_ = CooldownTimer(10.0f);
    void setup_led_rainbow(float total_time_ms) {
        flash_cd_.set_cd_ms(total_time_ms);
        is_rainbow_ = true;
        rainbow_ = Color(33, 66, 0);
    }

    PWM_Channel pwm_r = PWM_Channel(LED_R_PIN);
    PWM_Channel pwm_g = PWM_Channel(LED_G_PIN);
    PWM_Channel pwm_b = PWM_Channel(LED_B_PIN);
};