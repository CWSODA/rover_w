#include "pico/stdlib.h"

class Timer {
   public:
    // returns time between last call in us
    float clock() {
        auto time = get_absolute_time();
        float delta_time_us = absolute_time_diff_us(prev_time, time);
        prev_time = time;

        return delta_time_us;
    }

   private:
    absolute_time_t prev_time;
};