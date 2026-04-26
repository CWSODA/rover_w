#include "pwm.hpp"

#include <vector>

#include "settings.hpp"
#include "timer.hpp"

enum class LED_INDICATOR {
    POWER_ON,
    LOOP_NO_WIFI,
    LOOP_WITH_WIFI,
    MOTORS_OVERCURRENT,
};

struct Color {
    float r, g, b;
    Color(float red, float green, float blue) {
        r = red;
        g = green;
        b = blue;
    }
};

class LED {
   public:
    LED() {
        pwm_r.enable();
        pwm_g.enable();
        pwm_b.enable();
    }

    // changes LED duty cycle, (0 -> 100)
    void set(Color color) {
        pwm_r.set_duty(color.r);
        pwm_g.set_duty(color.g);
        pwm_b.set_duty(color.b);
    }

    // changes LED with given indicator
    // warning indicators will flash for some time
    void set_indicator(LED_INDICATOR opt) {
        switch (opt) {
            case LED_INDICATOR::POWER_ON: {
                set(Color(100, 0, 0));  // red
            }
            case LED_INDICATOR::LOOP_NO_WIFI: {
                set(Color(0, 100, 0));  // green
            }
            case LED_INDICATOR::LOOP_WITH_WIFI: {
                setup_led_flash(
                    std::vector<Color>{Color(0, 100, 0), Color(0, 0, 100)}, -1,
                    500);
            }
            case LED_INDICATOR::MOTORS_OVERCURRENT: {
                setup_led_flash(
                    std::vector<Color>{Color(0, 100, 0), Color(0, 0, 0)}, 10,
                    300);
            }
        }
    }

    bool has_wifi = false;
    void update() {
        if (count_ == 0) return;                    // dont change LEDs
        if (!flash_timer_.check_expired()) return;  // wait more

        // switch to next color
        idx_++;
        if (idx_ >= colors_.size()) {
            idx_ = 0;                  // wrap around to zero
            if (count_ > 0) count_--;  // decrement count if not infinite
            if (count_ == 0) {
                has_wifi ? set_indicator(LED_INDICATOR::LOOP_WITH_WIFI)
                         : set_indicator(LED_INDICATOR::LOOP_NO_WIFI);
            }
        }
        set(colors_[idx_]);

        flash_timer_.reset();
    }

   private:
    TimeoutTimer flash_timer_;
    std::vector<Color> colors_;
    size_t idx_ = 0;
    int count_ = 0;

    // sets up LED flashing alternating colors n times at set intervals
    // set n to -1 for indefinate
    void setup_led_flash(std::vector<Color> colors, int n, float interval_ms) {
        colors_ = std::move(colors);  // dont copy vector, move it
        count_ = n;
        flash_timer_.set_timeout_ms(interval_ms);
        flash_timer_.reset();

        set(colors_[0]);
        idx_ = 0;
    }

    PWM_Channel pwm_r = PWM_Channel(LED_R_PIN);
    PWM_Channel pwm_g = PWM_Channel(LED_G_PIN);
    PWM_Channel pwm_b = PWM_Channel(LED_B_PIN);
};