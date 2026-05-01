#pragma once

#include "pico/stdlib.h"  // used for type definitions uint16 etc

#include "lidar_objects.hpp"

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
// byte 1~2: offset as signed 16-bit integer in 0.01°
// byte 3~4: frame start angle in same format as above
// pairs of 3 bytes
// 1 byte unsigned signal strength
// 2 byte unsigned distance in 0.25mm

// total measurement count = N = (data length - 5)/3
// angle = start angle + 22.5 * (n - 1)/N
// where n is measurement index from 1 -> N
// delta angle = 22.5 * 1/N

inline void wrap_angle(float& angle) {
    // wrap if above 360°
    if (angle > 360.0f) angle -= 360.0f;
}

class LidarParser {
   public:
    LidarParser() {}

    void parse_byte(uint8_t byte, RotationBuffer& rot_buf);

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
    bool parse_data(uint8_t byte, RotationBuffer& rot_buf);

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
};