#pragma once

#include "pico/stdlib.h"  // used for type definitions uint16 etc
#include <queue>

#include "lidar_parser.hpp"
#include "lidar_objects.hpp"

class Lidar {
   public:
    Lidar();
    void update_lidar();

    RotationBuffer& get_rotation_buffer() { return rot_buf_; }

   private:
    LidarParser parser_;

    RotationBuffer rot_buf_;
};