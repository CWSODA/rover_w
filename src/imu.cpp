// code for interfacing with LSM6DS3 accelerometer+gyro MCU
#include "imu.hpp"

#include <math.h>

#include "settings.hpp"

#define IMU_I2C i2c1

IMU::IMU() {
    // enable and config ODR
    uint8_t src[] = {CTRL1_XL, XL_CFG, G_CFG};
    i2c_write_blocking(IMU_I2C, IMU_ADDR, src, sizeof(src), false);
}

// updates IMU and determines if calibration is needed
void IMU::update(MotorControl& motor_ctrl, LED& led) {
    read_imu_data();  // always get data

    // calibrating data
    if (is_calib_) {
        gyro_offset_ += gyro_;  // sum
        calib_count_++;         // increase count for average

        if (calib_timeout_.check_expired()) {  // check timeout
            // WDBG("IMU calib count: %zu\n", calib_count_);  // debug count
            gyro_offset_ /= calib_count_;  // calculate average
            is_calib_ = false;             // reset
            motor_ctrl.enable();           // re-enable motors
        }
        return;  // dont update rotation
    }

    // else normal function
    calc_rot();

    // check if another calibration is needed
    if (calib_interval_timer_.check()) {
        WDBG("Starting calibration\n");
        led.set_indicator(LED_INDICATOR::GYRO_CALIBRATION);
        is_calib_ = true;        // trigger calibration on next loop
        gyro_offset_.clear();    // reset offset
        calib_count_ = 0;        // reset count
        motor_ctrl.disable();    // disable motors
        calib_timeout_.reset();  // reset timeout
    }
}

// reads raw IMU data
// converts raw 16 byte data into RAW float data (not offset yet!)
void IMU::read_imu_data() {
    // 16 bit = 2 bytes for all three axes
    uint8_t gyro_data[3 * 2];
    uint8_t accel_data[3 * 2];

    // read
    i2c_write_blocking(IMU_I2C, IMU_ADDR, &OUTX_L_G, 1, false);
    i2c_read_blocking(IMU_I2C, IMU_ADDR, gyro_data, sizeof(gyro_data), true);
    i2c_read_blocking(IMU_I2C, IMU_ADDR, accel_data, sizeof(accel_data), false);

    // extract values from raw data
    // values are LSB, MSB with order XYZ
    int16_t int_gyro_x = (int16_t)(gyro_data[0] | (gyro_data[1] << 8));
    int16_t int_gyro_y = (int16_t)(gyro_data[2] | (gyro_data[3] << 8));
    int16_t int_gyro_z = (int16_t)(gyro_data[4] | (gyro_data[5] << 8));
    int16_t int_accel_x = (int16_t)(accel_data[0] | (accel_data[1] << 8));
    int16_t int_accel_y = (int16_t)(accel_data[2] | (accel_data[3] << 8));
    int16_t int_accel_z = (int16_t)(accel_data[4] | (accel_data[5] << 8));

    // convert values to °/s
    gyro_.x = int_gyro_x * (G_FULLSCALE / INT16_MAX);
    gyro_.y = int_gyro_y * (G_FULLSCALE / INT16_MAX);
    gyro_.z = int_gyro_z * (G_FULLSCALE / INT16_MAX);

    // convert values to g
    accel_.x = int_accel_x * (XL_FULLSCALE / INT16_MAX);
    accel_.y = int_accel_y * (XL_FULLSCALE / INT16_MAX);
    accel_.z = int_accel_z * (XL_FULLSCALE / INT16_MAX);

#if DEBUG_IMU
// DBG("raw gyro(%d, %d, %d)\n", gyro_x, gyro_y, gyro_z);
// DBG("gyro(%d, %d, %d)[°/s]\n", gyro.x, gyro.y, gyro.z);

// DBG("raw accel(%d, %d, %d)\n", accel_x, accel_y, accel_z);
// DBG("accel(%d, %d, %d)[g]\n", accel.x, accel.y, accel.z);
#endif
}

// calculates rotation with IMU data
// applies offset to gyro_ data
void IMU::calc_rot() {
    // use accelerometer to estimate pitch and roll
    // float x2 = accel_.x * accel_.x;
    // float y2 = accel_.y * accel_.y;
    // float z2 = accel_.z * accel_.z;

    // float pitch = atan2(-accel_.x, sqrtf(y2 + z2)) * RAD2DEG;
    // float roll = atan2(accel_.y, accel_.z) * RAD2DEG;
    // float length = sqrtf(x2 + y2 + z2);

    // Vec3 tilt;
    // tilt.x = acosf(accel_.x / length) * RAD2DEG;
    // tilt.y = acosf(accel_.y / length) * RAD2DEG;
    // tilt.z = acosf(accel_.z / length) * RAD2DEG;  // tilt from ground plane

    // remove offset
    gyro_ -= gyro_offset_;

    // use gyroscope to estimate pitch, roll, yaw
    auto delta_time_us = timer_.clock_us();
    if (delta_time_us != None) {
        float delta_time = delta_time_us.value() * 1e-6;  // convert to s

        pitch_ += gyro_.x * delta_time;
        roll_ += gyro_.y * delta_time;
        yaw_ += gyro_.z * delta_time;
    }

#if SEND_IMU_DATA
    if (imu_cd_timer_.check()) {
        uint8_t msg[1 + 1 + 4 + 4 + 4];
        msg[0] = '$';
        msg[1] = 'R';
        memcpy(&msg[2], &pitch_, 4);
        memcpy(&msg[2 + 4], &roll_, 4);
        memcpy(&msg[2 + 4 + 4], &yaw_, 4);
        send_bytes(msg, sizeof(msg));
        // WDBG("IMU: (%f) (%f) (%f)\n", pitch_, roll_, yaw_);
    }
#endif
}

// void enable_interrupt() {
//     // (INT1_CTRL) 0x0D = 0b10; enable gyro ready
//     // (INT2_CTRL) 0x0E = 0b01; enable accel ready
// }

// void setup_fifo() {
//     uint8_t src[2] = {FIFO_CTRL3, 0};

//     // add to fifo
//     src[1] = 0x001 << 3;  // gyro fifo
//     i2c_write_blocking(IMU_I2C, IMU_ADDR, src, sizeof(src), false);
//     src[1] = 0x001;  // accel fifo
//     i2c_write_blocking(IMU_I2C, IMU_ADDR, src, sizeof(src), false);
// }