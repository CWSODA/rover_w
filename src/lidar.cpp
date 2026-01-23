#include "lidar.hpp"

#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#include "global.h"
#include "sender.hpp"

#define SEND_LIDAR_DATA

#define DEBUG_STATES false

void uart1_rx_handler() {
    while (uart_is_readable(uart1)) {
        uint8_t ch = uart_getc(uart1);

        // echo on printf
        // printf("RX: %02X\n", ch);

        lidar_parser.parse_byte(ch);
    }
}

// initiate UART RX ONLY
void init_lidar_rx() {
    gpio_set_function(LIDAR_RX_PIN, GPIO_FUNC_UART);
    uart_init(uart1, 115200);

    uart_set_translate_crlf(uart1, false);  // disable translations

    irq_set_exclusive_handler(UART1_IRQ, uart1_rx_handler);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irqs_enabled(uart1, true, false);  // enable rx interrupts
}

void LidarParser::parse_byte(uint8_t byte) {
    // ignore if checksum bytes
    if (state != State::CHECKSUM) {
        checksum_total += byte;
        ++frame_byte_count;
    }

    switch (state) {
        case (State::START):
            assert(byte == 0xAA);
            advance_state();
            break;
        case (State::FRAME_LENGTH):
            if (frame_len_buf.insert(byte)) advance_state();
            break;
        case (State::VERSION):
            // assert(byte == 0x10); // not true
            advance_state();
            break;
        case (State::TYPE):
            assert(byte == 0x61);
            advance_state();
            break;
        case (State::COMMAND):
            if (byte == 0xAD) {
                // normal, parse data
                is_valid_data = true;
            } else if (byte == 0xAE) {
                // health status
                is_valid_data = false;
            } else {  // unknown command???
                assert(false);
            }
            advance_state();
            break;
        case (State::DATA_LENGTH):
            if (data_len_buf.insert(byte)) {
                data_state = DataState::ROT_SPEED;
                if (is_valid_data)
                    state = State::DATA;
                else
                    state = State::HEALTH;

                // ensures data length is consistent with frame length
                // data length should be 1+2+1+1+1+2 = 8 bytes shorter
                // printf("Data length: %d, Frame Length: %d\n",
                //        data_len_buf.val(), frame_len_buf.val());
                assert(data_len_buf.val() == frame_len_buf.val() - 8);
            }
            break;
        case (State::DATA):
            if (parse_data(byte)) advance_state();
            break;
        case (State::HEALTH):
            // assert(data_len_buf.val() == 1);
            data_byte_count++;
            rotation_speed = byte * 0.05f;
            advance_state();
            break;
        case (State::CHECKSUM):
            // enforce checksum
            if (checksum_buf.insert(byte)) {
                advance_state();
            }
            break;
    }

// debug state
#if DEBUG_STATES
    print_state();
#endif
}

bool LidarParser::parse_data(uint8_t byte) {
    ++data_byte_count;
    switch (data_state) {
        case (DataState::ROT_SPEED):
            rotation_speed = byte * 0.05f;
            // assert(rotation_speed != 0);  // shouldnt be 0 in here
            advance_d_state();
            break;
        case (DataState::ANGLE):
            if (angle_buf.insert(byte)) advance_d_state();
            break;
        case (DataState::START_ANGLE):
            if (start_angle_buf.insert(byte)) {
                start_angle = start_angle_buf.val() * 0.01f;
                temp_point.angle = start_angle;
                advance_d_state();
            }
            break;
        case (DataState::END_ANGLE):
            if (end_angle_buf.insert(byte)) {
                end_angle = end_angle_buf.val() * 0.01f;
                uint16_t n_measurements = (data_len_buf.val() - 7) / 3;
                assert(n_measurements != 0);  // should not have zero data
                delta_angle = (end_angle - start_angle) / n_measurements;
                printf("Delta angle: %f\n", delta_angle);
                advance_d_state();
            }
            break;
        // parts below loop
        case (DataState::SIG_STRENGTH):
            temp_point.sig_strength = byte;
            advance_d_state();
            break;
        case (DataState::DIST):
            if (dist_buf.insert(byte)) {
                // byte finished processing
                // save data to vector
                // increments of 0.25mm = 0.00025 m
                temp_point.distance = dist_buf.val() * 0.00025f;

                // data_vec.push_back(temp_point);

#ifdef SEND_LIDAR_DATA
                // send opcode L (lidar)
                // send sig_str, dist, angle
                send_byte('$');
                send_byte('L');
                send_byte(temp_point.sig_strength);
                send_float(temp_point.distance);
                send_float(temp_point.angle);
#endif

                // increment angle for next insert
                temp_point.angle += delta_angle;

                advance_d_state();
            }
            break;
    }
    if (data_byte_count == data_len_buf.val()) return true;
    return false;
}

void LidarParser::advance_state() {
    if (state == State::CHECKSUM) {
        // resets state machine
        state = State::START;

// ensure data is correct
#define DEBUG_LENGTH false
#if DEBUG_LENGTH
        printf("CS > tot: %d, exp: %d\n", checksum_total, checksum_buf.val());
        printf("FL > tot: %d, exp: %d\n", frame_byte_count,
               frame_len_buf.val());
        printf("DL > tot: %d, exp: %d\n", data_byte_count, data_len_buf.val());
#endif
        // assert(checksum_total == checksum_buf.val());
        // assert(frame_byte_count == frame_len_buf.val());
        // assert(data_byte_count == data_len_buf.val());

        // reset values
        frame_byte_count = 0;
        data_byte_count = 0;
        checksum_total = 0;
        return;
    } else if (state == State::HEALTH) {
        // printf("HS: %f\n", rotation_speed);
        state = State::CHECKSUM;
        return;
    }

    state = static_cast<State>(static_cast<int>(state) + 1);
}

void LidarParser::advance_d_state() {
    // loops between sig_strength and dist for subsequent measurements
    if (data_state == DataState::DIST) {
        data_state = DataState::SIG_STRENGTH;
        return;
    }

    data_state = static_cast<DataState>(static_cast<int>(data_state) + 1);
}

void LidarParser::print_state() {
    switch (state) {
        case (State::START):
            printf("START\n");
            break;
        case (State::FRAME_LENGTH):
            printf("FRAME LENGTH\n");
            break;
        case (State::VERSION):
            printf("VERSION\n");
            break;
        case (State::TYPE):
            printf("TYPE\n");
            break;
        case (State::COMMAND):
            printf("COMMAND\n");
            break;
        case (State::DATA_LENGTH):
            printf("DATA LENGTH\n");
            break;
        case (State::DATA):
            switch (data_state) {
                case (DataState::ROT_SPEED):
                    printf("DATA: ROT SPEED\n");
                    break;
                case (DataState::ANGLE):
                    printf("DATA: ANGLE\n");
                    break;
                case (DataState::START_ANGLE):
                    printf("DATA: START ANGLE\n");
                    break;
                case (DataState::END_ANGLE):
                    printf("DATA: END ANGLE\n");
                    break;
                // parts below loop
                case (DataState::SIG_STRENGTH):
                    printf("DATA: SIGNAL STRENGTH\n");
                    break;
                case (DataState::DIST):
                    printf("DATA: DISTANCE\n");
                    break;
            }
            break;
        case (State::HEALTH):
            printf("HEALTH\n");
            break;
        case (State::CHECKSUM):
            printf("CHECKSUM\n");
            break;
    }
}