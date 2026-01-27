#include "lidar.hpp"

#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#include "global.h"
#include "sender.hpp"
#include "uart1_ringbuf.h"  // has uart1_rx_handler

#define SEND_LIDAR_DATA

#define DEBUG_STATES false
#define DEBUG_LENGTH false

const int LIDAR_RX_BAUDRATE = 115200;

void lidar_update() {
    static uint16_t n = 0;
    if (n != 0) return;

    uint8_t byte;
    while (ringbuf_pop(&byte)) {
        // send_hex(byte);
        lidar_parser.parse_byte(byte);
    }

    uint16_t overflow_count = ringbuf_get_overflow_count();
    if (n != overflow_count) {
        DBG("OVERFLOWED %d TIMES!\n", overflow_count);
        n = overflow_count;
    }
}

// initiate UART RX ONLY
void init_lidar_rx() {
    gpio_set_function(LIDAR_RX_PIN, GPIO_FUNC_UART);
    uart_init(uart1, LIDAR_RX_BAUDRATE);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);

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

    // debug state
#if DEBUG_STATES
    print_state();
#endif

    switch (state) {
        case (State::START): {
            assert(byte == 0xAA);
            state = State::FRAME_LENGTH;
            break;
        }
        case (State::FRAME_LENGTH): {
            if (frame_len_buf.insert(byte)) state = State::VERSION;
            break;
        }
        case (State::VERSION): {
            // assert(byte == 0x10); // not true
            state = State::TYPE;
            break;
        }
        case (State::TYPE): {
            assert(byte == 0x61);
            state = State::COMMAND;
            break;
        }
        case (State::COMMAND): {
            if (byte == 0xAD) {
                // normal, parse data
                is_valid_data = true;
            } else if (byte == 0xAE) {
                // health status
                is_valid_data = false;
            } else {  // unknown command???
                assert(false);
            }
            state = State::DATA_LENGTH;
            break;
        }
        case (State::DATA_LENGTH): {
            if (!data_len_buf.insert(byte)) break;

            if (is_valid_data) {
                // remember to reset data state to intial rot speed
                data_state = DataState::ROT_SPEED;
                data_byte_count = 0;
                state = State::DATA;
            } else
                state = State::HEALTH;

            // ensures data length is consistent with frame length
            // data length should be 1+2+1+1+1+2 = 8 bytes shorter
            // DBG("Data length: %d, Frame Length: %d\n",
            //        data_len_buf.val(), frame_len_buf.val());
            assert(data_len_buf.val() == frame_len_buf.val() - 8);

            break;
        }
        case (State::DATA): {
            if (parse_data(byte)) state = State::CHECKSUM;
            break;
        }
        case (State::HEALTH): {
            // assert(data_len_buf.val() == 1);
            ++data_byte_count;
            rotation_speed = byte * 0.05f;
            state = State::CHECKSUM;
            break;
        }
        case (State::CHECKSUM): {
            // enforce checksum
            if (checksum_buf.insert(byte)) {
                // TODO
                reset_state();
            }
            break;
        }
    }
}

bool LidarParser::parse_data(uint8_t byte) {
    ++data_byte_count;
    switch (data_state) {
        case (DataState::ROT_SPEED): {
            rotation_speed = byte * 0.05f;
            // assert(rotation_speed != 0);  // shouldnt be 0 in here
            data_state = DataState::ANGLE;
            break;
        }
        case (DataState::ANGLE): {
            if (angle_buf.insert(byte)) data_state = DataState::START_ANGLE;
            break;
        }
        case (DataState::START_ANGLE): {
            if (!start_angle_buf.insert(byte)) break;

            start_angle = start_angle_buf.val() * 0.01f;
            temp_point.angle = start_angle;
            data_state = DataState::END_ANGLE;

            break;
        }
        case (DataState::END_ANGLE): {
            if (!end_angle_buf.insert(byte)) break;

            end_angle = end_angle_buf.val() * 0.01f;

            // data length includes 1+2+2+2 header for rot speed and angles
            // each data point has 1+2 bytes for strength(1) and distance(2)
            uint16_t n_measurements = (data_len_buf.val() - 7) / 3;
            assert(n_measurements != 0);  // should not have zero data
            delta_angle = (end_angle - start_angle) / n_measurements;
            data_state = DataState::SIG_STRENGTH;

            // debug angle data
            DBG("---\n");
            DBG("Offset angle: %f\n", angle_buf.val() * 0.01f);
            DBG("Start angle: %f\n", start_angle);
            DBG("End angle: %f\n", end_angle);
            DBG("DL: %d\n", data_len_buf.val());
            DBG("DP float: %f\n", (data_len_buf.val() - 7.0f) / 3.0f);
            DBG("Data points: %d\n", n_measurements);
            DBG("Delta angle: %f\n", delta_angle);
            DBG("---\n");

            break;
        }
        // parts below loop
        case (DataState::SIG_STRENGTH):
            temp_point.sig_strength = byte;
            data_state = DataState::DIST;
            break;
        case (DataState::DIST): {
            if (dist_buf.insert(byte)) break;
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

            data_state = DataState::SIG_STRENGTH;
            break;
        }
    }
    if (data_byte_count == data_len_buf.val()) {
        if (data_state == DataState::SIG_STRENGTH) {
            DBG("Broken at sig\n");
        } else if (data_state == DataState::DIST) {
            DBG("Broken at dist\n");
        } else {
            DBG("Broken at invalid\n");
        }
        return true;
    }
    return false;
}

void LidarParser::reset_state() {
    state = State::START;

#if DEBUG_LENGTH
    DBG("CS > tot: %d, exp: %d\n", checksum_total, checksum_buf.val());
    DBG("FL > tot: %d, exp: %d\n", frame_byte_count, frame_len_buf.val());
    DBG("DL > tot: %d, exp: %d\n", data_byte_count, data_len_buf.val());
#endif
    // ensure data is correct
    // assert(checksum_total == checksum_buf.val());
    // assert(frame_byte_count == frame_len_buf.val());

    // reset values
    frame_byte_count = 0;
    checksum_total = 0;
    return;
}

void LidarParser::print_state() {
    switch (state) {
        case (State::START):
            DBG("START\n");
            break;
        case (State::FRAME_LENGTH):
            DBG("FRAME LENGTH\n");
            break;
        case (State::VERSION):
            DBG("VERSION\n");
            break;
        case (State::TYPE):
            DBG("TYPE\n");
            break;
        case (State::COMMAND):
            DBG("COMMAND\n");
            break;
        case (State::DATA_LENGTH):
            DBG("DATA LENGTH\n");
            break;
        case (State::DATA):
            switch (data_state) {
                case (DataState::ROT_SPEED):
                    DBG("DATA: ROT SPEED\n");
                    break;
                case (DataState::ANGLE):
                    DBG("DATA: ANGLE\n");
                    break;
                case (DataState::START_ANGLE):
                    DBG("DATA: START ANGLE\n");
                    break;
                case (DataState::END_ANGLE):
                    DBG("DATA: END ANGLE\n");
                    break;
                // parts below loop
                case (DataState::SIG_STRENGTH):
                    DBG("DATA: SIGNAL STRENGTH\n");
                    break;
                case (DataState::DIST):
                    DBG("DATA: DISTANCE\n");
                    break;
            }
            break;
        case (State::HEALTH):
            DBG("HEALTH\n");
            break;
        case (State::CHECKSUM):
            DBG("CHECKSUM\n");
            break;
    }
}