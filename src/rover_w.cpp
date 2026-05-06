#include <stdio.h>
#include "pico/stdlib.h"

/* -------------------- self headers -------------------- */
#include "settings.hpp"
#include "global_vars.hpp"

// components
#include "motor.hpp"
#include "lidar.hpp"
#include "imu.hpp"
#include "algo.hpp"
#include "led.hpp"
#include "tcp.hpp"

// UART debug
// screen /dev/tty.usbserial-FTU7C2WR 115200
// screen /dev/tty.usbserial-FTU7C2WR 460800

// USB debug
// screen /dev/tty.usbmodem11101
// screen /dev/tty.usbmodem1101

/* --------------- global extern variables -------------- */
bool is_motor_fine = true;
TCP_Buffer tcp_buffer;  // init TCP buffer here since it is global

/* --------------------- INIT ORDER --------------------- */
// LED for indication
// Data sending for messages
// I2C1 used for IMU
// MotorControl, Lidar, IMU in any order
// Recommended to calibrate IMU gyroscope after init

/* ---------------------- MAIN LOOP --------------------- */
int main() {
    LED led;

    bool has_wifi = false;
    init_data_sending(has_wifi);
    // sleep_ms(1000);  // allow time for connection
    DBG("Rover Starting...\n");

    init_i2c1();  // I2C1 for IMU and encoders

    MotorControl motor_control;
    Lidar lidar;
    IMU imu;
    Algo algo;

    /* ------- things that have seperate update loops: ------ */
    // - spinning lidar
    // - motors (encoders included)
    // - imu (xl + gyro)
    // - tcp parser / buffer
    // - LED flash control

    DBG("Starting loop...\n");
    led.has_wifi = has_wifi;
    algo.is_algo_on = !has_wifi;
    led.set_default();
    while (true) {
        tcp_buffer.parse_tcp_buffer(motor_control, algo, imu, led);

        imu.update(motor_control, led);
        float imu_yaw = imu.get_yaw();

        lidar.update_lidar();
        motor_control.update_motors(imu_yaw);
        led.update();
        algo.update(lidar.get_rotation_buffer(), imu_yaw, motor_control);

        flush_tcp_write_buffer();

        sleep_ms(1);  // set as low as possible
    }
}
