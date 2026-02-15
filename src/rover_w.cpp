#include <stdio.h>
#include "pico/stdlib.h"

// self headers
#include "sender.hpp"
// #include "motors.hpp"
#include "lidar.hpp"
#include "global.h"

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
    // MotorControl motors;

    // INIT LIDAR
    init_lidar_rx();

    absolute_time_t time = get_absolute_time();
    int64_t timer_sum = 0;
    while (true) {
        // char c = uart_getc(uart1);
        // printf(" %02X ", c);
        // motors.update_motors();
        // sleep_ms(500);
        // DBG("ALIVE\n");
        lidar_update();
    }
}
