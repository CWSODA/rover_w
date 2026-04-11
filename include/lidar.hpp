#pragma once

#include "pico/stdlib.h"  // used for type definitions uint16 etc
#include <vector>

#include "lidar_parser.hpp"

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

class Lidar {
   public:
    Lidar();
    void update_lidar();

   private:
    LidarParser parser_;
};