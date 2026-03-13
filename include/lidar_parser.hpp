#pragma once

#include "pico/stdlib.h"  // used for type definitions uint16 etc
#include <vector>

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
        OFFSET_ANGLE,
        START_ANGLE,
        SIG_STRENGTH,
        DIST
    };

    // handles resetting vars
    void reset_state();

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
    TwoByteBuffer offset_angle_buf;
    TwoByteBuffer start_angle_buf;
    TwoByteBuffer dist_buf;

    // data vars
    float rotation_speed = 0.0f;  // in 0.05 rotation/s increments
    float delta_angle;            // calc from parser, in degrees
    DataPoint temp_point;         // temporary cache
    std::vector<DataPoint> data_vec;
};