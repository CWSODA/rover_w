#pragma once

#include "pico/stdlib.h"
#include <pico/critical_section.h>
#include <deque>

// required for control
#include "motor.hpp"
#include "algo.hpp"
#include "imu.hpp"
#include "led.hpp"

class TCP_Buffer {
   public:
    TCP_Buffer() { critical_section_init(&lock_); }

    void parse_tcp_buffer(MotorControl& motor_ctrl, Algo& algo, IMU& imu,
                          LED& led);

    void push_byte(uint8_t byte);

   private:
    std::deque<uint8_t> buffer_;
    critical_section_t lock_;  // lock to ensure data integrity for ISR

    // lock safe access, ALWAYS use
    uint8_t get_byte(size_t idx);

    // lock safe access, ALWAYS use
    size_t buf_size();

    // pops front buffer until next command '$'
    void erase_to_next_cmd();

    // efficiently pops to given index, exclusive
    void pop_to_idx(size_t idx);
};