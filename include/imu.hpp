// code for interfacing with LSM6DS3 accelerometer+gyro MCU
#pragma once

#include "pico/stdlib.h"
#include <cstring>

#include "i2c.hpp"
#include "timer.hpp"

#define IMU_I2C i2c1
constexpr uint8_t IMU_ADDR = 0x6A;

// registers
constexpr uint8_t FIFO_CTRL3 = 0x08;
constexpr uint8_t CTRL1_XL = 0x10;
constexpr uint8_t CTRL2_G = 0x11;
constexpr uint8_t OUTX_L_G = 0x22;
constexpr uint8_t OUTX_L_XL = 0x28;

// Output data rate (ODR)
constexpr uint8_t F_13HZ = 1;
constexpr uint8_t F_26HZ = 2;
constexpr uint8_t F_52HZ = 3;
constexpr uint8_t F_104HZ = 4;
constexpr uint8_t F_208HZ = 5;
constexpr uint8_t F_416HZ = 6;
constexpr uint8_t F_833HZ = 7;
constexpr uint8_t F_1660HZ = 8;

constexpr uint8_t IMU_FREQ = F_13HZ;
constexpr uint8_t XL_LOWER = 0b00'00;  // 2g fullscale
constexpr uint8_t G_LOWER = 0b00'00;   // 245dps fullscale
constexpr float XL_FULLSCALE = 2;
constexpr float G_FULLSCALE = 245;
constexpr uint8_t XL_CFG = (IMU_FREQ << 4) | XL_LOWER;
constexpr uint8_t G_CFG = (IMU_FREQ << 4) | G_LOWER;

// helpers
constexpr float RAD2DEG = 180 / 3.1415f;
struct Vec3 {
    float x, y, z;

    void operator+=(const Vec3& other) {
        this->x += other.x;
        this->y += other.y;
        this->z += other.z;
    }
    void operator/=(const float divisor) {
        this->x /= divisor;
        this->y /= divisor;
        this->z /= divisor;
    }

    void clear() {
        x = 0;
        y = 0;
        z = 0;
    }
};

class IMU {
   public:
    IMU();
    void calibrate_gyro();

    void update();

   private:
    void read_imu_data();
    void calc_rot();

    Vec3 accel_;
    Vec3 gyro_;
    Vec3 gyro_offset_;

    float pitch, roll, yaw;

    Timer timer_;
    CooldownTimer imu_cd_timer_ = CooldownTimer(IMU_SEND_CD_MS);
};