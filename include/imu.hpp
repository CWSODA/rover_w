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
};

// forward declarations
void calc_rot(Vec3& accel, Vec3& gyro);

bool init_imu() {
    // enable and config ODR
    uint8_t src[] = {CTRL1_XL, XL_CFG, G_CFG};
    i2c_write_blocking(IMU_I2C, IMU_ADDR, src, sizeof(src), false);

    return true;
}

void enable_int() {
    // (INT1_CTRL) 0x0D = 0b10; enable gyro ready
    // (INT2_CTRL) 0x0E = 0b01; enable accel ready
}

void setup_fifo() {
    uint8_t src[2] = {FIFO_CTRL3, 0};

    // add to fifo
    src[1] = 0x001 << 3;  // gyro fifo
    i2c_write_blocking(IMU_I2C, IMU_ADDR, src, sizeof(src), false);
    src[1] = 0x001;  // accel fifo
    i2c_write_blocking(IMU_I2C, IMU_ADDR, src, sizeof(src), false);
}

void read_xl() {
    uint8_t accel_data[3 * 2];

    // read
    i2c_write_blocking(IMU_I2C, IMU_ADDR, &OUTX_L_XL, 1, false);
    i2c_read_blocking(IMU_I2C, IMU_ADDR, accel_data, sizeof(accel_data), false);

    // extract values from raw data
    // values are LSB, MSB with order XYZ
    int16_t accel_x = (int16_t)(accel_data[0] | (accel_data[1] << 8));
    int16_t accel_y = (int16_t)(accel_data[2] | (accel_data[3] << 8));
    int16_t accel_z = (int16_t)(accel_data[4] | (accel_data[5] << 8));
    Vec3 accel;
    accel.x = accel_x * (XL_FULLSCALE / INT16_MAX);
    accel.y = accel_y * (XL_FULLSCALE / INT16_MAX);
    accel.z = accel_z * (XL_FULLSCALE / INT16_MAX);

    DBG("raw accel(%d, %d, %d)\n", accel_x, accel_y, accel_z);
    DBG("accel(%f, %f, %f)[g]\n", accel.x, accel.y, accel.z);

    calc_rot(accel, accel);
}

void read_imu_data() {
    uint8_t gyro_data[3 * 2];
    uint8_t accel_data[3 * 2];

    // read
    i2c_write_blocking(IMU_I2C, IMU_ADDR, &OUTX_L_G, 1, false);
    i2c_read_blocking(IMU_I2C, IMU_ADDR, gyro_data, sizeof(gyro_data), true);
    i2c_read_blocking(IMU_I2C, IMU_ADDR, accel_data, sizeof(accel_data), false);

    // extract values from raw data
    // values are LSB, MSB with order XYZ
    int16_t gyro_x = (int16_t)(gyro_data[0] | (gyro_data[1] << 8));
    int16_t gyro_y = (int16_t)(gyro_data[2] | (gyro_data[3] << 8));
    int16_t gyro_z = (int16_t)(gyro_data[4] | (gyro_data[5] << 8));
    Vec3 gyro;  // values in °/s
    gyro.x = gyro_x * (G_FULLSCALE / INT16_MAX);
    gyro.y = gyro_y * (G_FULLSCALE / INT16_MAX);
    gyro.z = gyro_z * (G_FULLSCALE / INT16_MAX);

    int16_t accel_x = (int16_t)(accel_data[0] | (accel_data[1] << 8));
    int16_t accel_y = (int16_t)(accel_data[2] | (accel_data[3] << 8));
    int16_t accel_z = (int16_t)(accel_data[4] | (accel_data[5] << 8));
    Vec3 accel;  // values in g
    accel.x = accel_x * (XL_FULLSCALE / INT16_MAX);
    accel.y = accel_y * (XL_FULLSCALE / INT16_MAX);
    accel.z = accel_z * (XL_FULLSCALE / INT16_MAX);

    // DBG("raw gyro(%d, %d, %d)\n", gyro_x, gyro_y, gyro_z);
    // DBG("gyro(%d, %d, %d)[°/s]\n", gyro.x, gyro.y, gyro.z);

    // DBG("raw accel(%d, %d, %d)\n", accel_x, accel_y, accel_z);
    // DBG("accel(%d, %d, %d)[g]\n", accel.x, accel.y, accel.z);

    calc_rot(accel, gyro);
}

void calc_rot(Vec3& accel, Vec3& gyro) {
    // use accelerometer to estimate pitch and roll
    float x2 = accel.x * accel.x;
    float y2 = accel.y * accel.y;
    float z2 = accel.z * accel.z;

    float pitch = atan2(-accel.x, sqrtf(y2 + z2)) * RAD2DEG;
    float roll = atan2(accel.y, accel.z) * RAD2DEG;
    float length = sqrt(x2 + y2 + z2);

    Vec3 tilt;
    tilt.x = acosf(accel.x / length) * RAD2DEG;
    tilt.y = acosf(accel.y / length) * RAD2DEG;
    tilt.z = acosf(accel.z / length) * RAD2DEG;  // tilt from ground plane

    // DBG("pitch: %f\n", pitch);
    // DBG("roll: %f\n", roll);
    // DBG("x tilt: %f\n", tilt.x);
    // DBG("y tilt: %f\n", tilt.y);
    // DBG("z tilt: %f\n", tilt.z);

    // use gyroscope to estimate yaw
    static Timer timer;
    static float g_pitch = 0, g_roll = 0, g_yaw = 0;
    auto delta_time_us = timer.clock_us();
    if (delta_time_us != None) {
        float delta_time = delta_time_us.value() * 1e-6;  // convert to s

        g_pitch += gyro.x * delta_time;
        g_roll += gyro.y * delta_time;
        g_yaw += gyro.z * delta_time;
        // DBG("delta time: %f\n", delta_time);
        // DBG("pitch: %f\n", g_pitch);
        // DBG("roll: %f\n", g_roll);
        // DBG("yaw: %f\n", g_yaw);
    }

#if SEND_IMU_DATA
    static CooldownTimer imu_cd(IMU_SEND_CD_MS);
    if (imu_cd.check()) {
        uint8_t msg[1 + 1 + 4 + 4 + 4];
        msg[0] = '$';
        msg[1] = 'R';
        memcpy(&msg[2], &g_pitch, 4);
        memcpy(&msg[2 + 4], &g_roll, 4);
        memcpy(&msg[2 + 4 + 4], &g_yaw, 4);
        send_bytes(msg, sizeof(msg));
    }
#endif
}