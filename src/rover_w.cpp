#include <stdio.h>
#include "pico/stdlib.h"

// self headers
#include "sender.hpp"
// #include "motors.hpp"
#include "lidar.hpp"
#include "global.h"

// UART debug
// screen /dev/tty.usbserial-FTU7C2WR 115200

// USB debug
// screen /dev/tty.usbmodem11101

int main() {
    init_data_sending();
    sleep_ms(1000);
    DBG("Rover Starting...\n");

    // INIT MOTORS
    // MotorControl motors;

    // INIT LIDAR
    init_lidar_rx();

    while (true) {
        // char c = uart_getc(uart1);
        // printf(" %02X ", c);
        // motors.update_motors();
        // sleep_ms(500);
        // DBG("ALIVE\n");
        lidar_update();
    }
}
