#include <stdio.h>
#include "pico/stdlib.h"

// self headers
#include "settings.hpp"
#include "global_vars.hpp"
#include "sender.hpp"
#include "motor.hpp"
#include "lidar.hpp"
#include "encoder.hpp"
#include "imu.hpp"

// UART debug
// screen /dev/tty.usbserial-FTU7C2WR 115200
// screen /dev/tty.usbserial-FTU7C2WR 460800

// USB debug
// screen /dev/tty.usbmodem11101
// screen /dev/tty.usbmodem1101

// global extern variables
bool is_motor_fine = true;
TCP_Buffer tcp_buffer;

int main() {
    init_data_sending();
    sleep_ms(1000);  // allow time for connection
    DBG("Rover Starting...\n");

    // init i2c
    init_i2c1();

    // INIT MOTORS
    // MotorControl motor_control;

    // INIT LIDAR
    // Lidar lidar;

    // init accelerometer + gyro
    // IMU imu;

    /*
    things that have seperate update loops:
    - spinning lidar
    - motors (encoders included)
    - imu (xl + gyro)
    - tcp parser
    */
    DBG("Starting loop...\n");
    while (true) {
        // motor_control.update_motors();
        sleep_ms(1000);
    }
}
