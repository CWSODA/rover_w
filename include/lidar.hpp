#pragma once

#include "pico/stdlib.h"  // used for type definitions uint16 etc
#include <queue>

#include "lidar_parser.hpp"

class Lidar {
   public:
    Lidar();
    void update_lidar();

    std::queue<DataPoint>& get_data_queue() { return data_queue_; }

   private:
    std::queue<DataPoint> data_queue_;
    LidarParser parser_ = LidarParser(&data_queue_);
};