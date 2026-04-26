#include "pwm.hpp"

#include "settings.hpp"

enum class LED_INDICATOR {
    POWER_ON,
    START_LOOP,
    WIFI_CONNECTED,
};

class LED {
   public:
    LED() {
        pwm_r.enable();
        pwm_g.enable();
        pwm_b.enable();
    }

    // changes LED duty cycle
    void set(float r, float g, float b) {
        pwm_r.set_duty(r);
        pwm_g.set_duty(g);
        pwm_b.set_duty(b);
    }

    // changes LED with given indicator
    void set_indicator(LED_INDICATOR opt) {
        switch (opt) {
            case LED_INDICATOR::POWER_ON: {
                set(1.0f, 0.0f, 0.0f);
            }
            case LED_INDICATOR::START_LOOP: {
                set(0.0f, 1.0f, 0.0f);
            }
            case LED_INDICATOR::WIFI_CONNECTED: {
                set(1, 1, 0);
            }
        }
    }

   private:
    PWM_Channel pwm_r = PWM_Channel(LED_R_PIN);
    PWM_Channel pwm_g = PWM_Channel(LED_G_PIN);
    PWM_Channel pwm_b = PWM_Channel(LED_B_PIN);
};