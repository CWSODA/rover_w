#pragma once

#include "pico/stdlib.h"  // used for type definitions uint16 etc
#include <array>
#include <queue>

#include "settings.hpp"

// output struct of LiDAR
struct DataPoint {
    uint8_t sig_strength;  // strength from 0 to 255
    float distance;        // distance in meters
    float angle;           // angle in degrees

    DataPoint() {}
    DataPoint(float dist, float angle, uint8_t sig_str)
        : distance(dist), angle(angle), sig_strength(sig_str) {}
};

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

// contains two rotating buffers
struct DataBuffer {
    std::array<DataPoint, LIDAR_ROTATION_BUFFER_MAX_POINTS> buf;
    size_t count = 0;

    void push(DataPoint p) {
        if (count == sizeof(buf)) return;  // ignore if buffer full
        buf.at(count) = p;
        count++;
    }

    void clear() { count = 0; }  // no need to clear data
};
class RotationBuffer {
   public:
    void push(DataPoint p) { data_.at(buf_idx_).push(p); }

    void swap_buffer() {
        buf_idx_ ^= 1;
        has_new_buf_ = true;         // set flag so algorithm will take buffer
        data_.at(buf_idx_).clear();  // clear current buffer
    }

    bool has_new_buf() { return has_new_buf_; }

    // returns "complete" buffer
    // which is opposite of the buffer being pushed into
    DataBuffer& get_complete_buffer() {
        // probably should check for flag validity but oh well
        has_new_buf_ = false;  // reset flag
        return data_.at(1 ^ buf_idx_);
    }

   private:
    std::array<DataBuffer, 2> data_;
    uint8_t buf_idx_ = 0;  // picker for which buffer to fill into
    bool has_new_buf_ = false;
};