#include "tcp_parser.hpp"

#include "pico/stdlib.h"
#include <pico/critical_section.h>
#include <deque>

#include "settings.hpp"
#include "motor.hpp"

void set_ctrl_from_dir(uint8_t dir, float& speed, float& turn);

// parses TCP buffer
// can change motor control and toggle algorithm
void TCP_Buffer::parse_tcp_buffer(MotorControl& motor_ctrl) {
    // loop until buffer consumed
    while (buf_size() >= 2) {  // start byte + command
        if (get_byte(0) != '$') {
            erase_to_next_cmd();
            continue;
        }

        // switch command, size of frame
        // return if not full command received, REMEMBER TO EXIT lock
        uint8_t cmd = get_byte(1);
        switch (cmd) {
            case 'C': {  // drive control, 2 + speed(1) + dir(1) = 4
                if (buf_size() < 4) {
                    return;  // wait for all bytes
                }

                uint8_t speed_byte = get_byte(2);
                uint8_t dir_byte = get_byte(3);
                pop_to_idx(4);
                // convert
                // N, NE, E, SE, S, SW, W, NW
                // WDBG("ctrl: %d, %d\n", speed, dir);
                float turn;
                float speed = speed_byte;
                set_ctrl_from_dir(dir_byte, speed, turn);

                motor_ctrl.steer_with_timeout(speed, turn);

                break;
            }
            case 'A': {  // turn on algorithm, 2 + 0 = 2
                pop_to_idx(2);
                motor_ctrl.enable_algo();
                WDBG("toggle algo\n");
                break;
            }
        }
    }
}

void TCP_Buffer::push_byte(uint8_t byte) {
    critical_section_enter_blocking(&lock_);
    buffer_.push_back(byte);
    critical_section_exit(&lock_);
}

// lock safe access, ALWAYS use
uint8_t TCP_Buffer::get_byte(size_t idx) {
    critical_section_enter_blocking(&lock_);
    uint8_t byte = buffer_.at(idx);
    // WDBG("got: (%02X)\n", byte);
    critical_section_exit(&lock_);
    return byte;
}

// lock safe access, ALWAYS use
size_t TCP_Buffer::buf_size() {
    critical_section_enter_blocking(&lock_);
    size_t size = buffer_.size();
    critical_section_exit(&lock_);
    return size;
}

// pops front buffer until next command '$'
void TCP_Buffer::erase_to_next_cmd() {
    critical_section_enter_blocking(&lock_);
    size_t start_byte_idx = buffer_.size() - 1;  // final index
    for (size_t idx = 0; idx < buffer_.size(); idx++) {
        uint8_t byte = buffer_.at(idx);
        if (byte == '$') {
            start_byte_idx = idx;
            break;
        }
    }
    if (start_byte_idx >= buffer_.size() - 1)
        buffer_.clear();
    else {
        pop_to_idx(start_byte_idx);
    }
    critical_section_exit(&lock_);
}

// efficiently pops to given index, exclusive
void TCP_Buffer::pop_to_idx(size_t idx) {
    critical_section_enter_blocking(&lock_);
    for (size_t count = 0; count < idx; count++) {
        buffer_.pop_front();
    }
    critical_section_exit(&lock_);
}

// decodes dir_byte into speed and turn
// sets speed to negative for south
void set_ctrl_from_dir(uint8_t dir, float& speed, float& turn) {
    switch (dir) {
        case (1 << 0): {  // N
            turn = 0.0f;
            break;
        }
        case (1 << 1): {  // NE
            turn = 50.0f;
            break;
        }
        case (1 << 2): {  // E
            turn = 100.0f;
            break;
        }
        case (1 << 3): {  // SE
            turn = 50.0f;
            speed *= -1;
            break;
        }
        case (1 << 4): {  // S
            turn = 0.0f;
            speed *= -1;
            break;
        }
        case (1 << 5): {  // SW
            turn = -50.0f;
            speed *= -1;
            break;
        }
        case (1 << 6): {  // W
            turn = -100.0f;
            break;
        }
        case (1 << 7): {  // NW
            turn = -50.0f;
            break;
        }
        default: {
            speed = 0;  // unknown, turn off motors
        }
    }
}