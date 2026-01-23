#pragma once
#define LIDAR_RX_PIN 5

#include "pico/stdlib.h"  // used for type definitions uint16 etc
#include <vector>

// code for the delta-2D lidar

// data comes from UART TX pin from the lidar
// data is sent as packets / frames

/* FRAME LAYOUT */
// header (1 byte): always 0xAA
// frame length (2 bytes): length of frame excluding confirmation bytes
// version (1 byte): default of 0x10
// type (1 byte): always 0x61
// command (1 byte): specifies data type
// data length (2 bytes): length of data
// data (variable length)
// confirmation bytes (2 bytes): 16-bit sum of all previous bytes

/* COMMANDS */
// measurements (0xAD) (3N+5 bytes)
// health information (0xAE) (1 byte) <- rotational speed

/* MEASUREMENT LAYOUT */
// byte 0: rotational speed as unsigned integer in 0.05rad/s
// byte 1~2: angle of rotation as signed 16-bit integer in 0.01°
// byte 3~4: frame start angle
// byte 5~6: frame end angle
// pairs of 3 bytes
// 1 byte unsigned signal strength
// 2 byte unsigned distance in 0.25mm

void init_lidar_rx();

// helper class to buffer 2 byte / 16-bit data
class TwoByteBuffer {
   public:
    // inserts byte into buffer
    // returns true if buffer is full
    // RESETS count if full
    bool insert(uint8_t byte) {
        if (is_first_byte) {
            data_ = byte << 8;  // MSB
            is_first_byte = false;
            return false;
        }

        // otherwise on second byte
        data_ += byte;
        is_first_byte = true;  // reset for next cycle

        return true;
    }

    uint16_t val() { return data_; }

    void reset() { is_first_byte = true; }

   private:
    bool is_first_byte = true;
    uint16_t data_;
};

struct DataPoint {
    uint8_t sig_strength;
    float distance;
    float angle;

    DataPoint() {}
    DataPoint(float dist, float angle, uint8_t sig_strength) {
        this->sig_strength = sig_strength;
        this->distance = dist;
        this->angle = angle;
    }
};

class LidarParser {
   public:
    void parse_byte(uint8_t byte);

   private:
    enum class State {
        START = 0,
        FRAME_LENGTH,
        VERSION,
        TYPE,
        COMMAND,
        DATA_LENGTH,
        DATA,
        CHECKSUM,
        HEALTH,
    };

    void print_state();

    enum class DataState {
        ROT_SPEED = 0,
        ANGLE,
        START_ANGLE,
        END_ANGLE,
        SIG_STRENGTH,
        DIST
    };

    // handles moving to next state
    void advance_state();
    // advance data state
    void advance_d_state();

    // returns true once all data bytes have been parsed
    bool parse_data(uint8_t byte);

    // state vars
    State state = State::START;
    DataState data_state = DataState::ROT_SPEED;
    bool is_valid_data = false;
    uint16_t checksum_total = 0;
    uint16_t frame_byte_count = 0;
    uint16_t data_byte_count = 0;

    // outer parser buffers
    TwoByteBuffer checksum_buf;
    TwoByteBuffer frame_len_buf;
    TwoByteBuffer data_len_buf;

    // data parser buffers
    TwoByteBuffer angle_buf;
    TwoByteBuffer start_angle_buf;
    TwoByteBuffer end_angle_buf;
    TwoByteBuffer dist_buf;

    // data vars
    float rotation_speed = 0.0f;                // in 0.05rad/s increments
    float start_angle, end_angle, delta_angle;  // calc from buffer
    DataPoint temp_point;                       // temp
    std::vector<DataPoint> data_vec;
};