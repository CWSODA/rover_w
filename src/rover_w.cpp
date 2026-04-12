#include <stdio.h>
#include "pico/stdlib.h"

// self headers
#include "global.hpp"
#include "sender.hpp"
#include "motors.hpp"
#include "lidar.hpp"
#include "encoder.hpp"
#include "imu.hpp"

// UART debug
// screen /dev/tty.usbserial-FTU7C2WR 115200
// screen /dev/tty.usbserial-FTU7C2WR 460800

// USB debug
// screen /dev/tty.usbmodem11101
// screen /dev/tty.usbmodem1101

int main() {
    init_data_sending();
    sleep_ms(1000);
    DBG("Rover Starting...\n");

    // INIT MOTORS
    MotorControl motor_control;
    init_encoder();

    // INIT LIDAR
    Lidar lidar;

    // init i2c
    init_i2c1();

    // init accelerometer + gyro
    init_imu();

    /*
    things that have update loops:
    - lidar
    - motors
    - encoders
    - imu (xl + gyro)
    */
    DBG("Starting loop...\n");
    while (true) {
        // lidar.update_lidar();
        // motor_control.update_motors();
        // update_encoders();
        // read_xl();

        // sleep_ms(1.0f / 13.0f);

        static uint64_t n = 0;
        send_byte('k');
        send_byte('\n');
        sleep_ms(2000);
    }
}
