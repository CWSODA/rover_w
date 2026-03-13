#include <stdio.h>
#include "pico/stdlib.h"

// self headers
#include "sender.hpp"
#include "motors.hpp"
#include "lidar.hpp"
#include "magnetic_encoder.hpp"
#include "global.hpp"

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

    // INIT LIDAR
    Lidar lidar;

    while (true) {
        // lidar.update_lidar();
        // motor_control.update_motors();
    }
}
