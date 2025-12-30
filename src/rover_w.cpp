#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "hardware/uart.h"

// self headers
#include "motors.hpp"
#include "lidar.hpp"

// #include "tcp.hpp"
#include "help.hpp"

// UART debug
// screen /dev/tty.usbserial-FTU7C2WR 115200

// USB debug
// screen /dev/tty.usbmodem11101

int main() {
    stdio_init_all();

    // wait for usb to connect
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }

    // INIT MOTORS
    // MotorControl motors;

    // INIT LIDAR
    // init_lidar_rx(LIDAR_RX_PIN);

    test();

    // init_server_chip();
    // if (init_tcp() != 0) {
    //     return -1;
    // }

    while (true) {
        // char c = uart_getc(uart1);
        // printf(" %02X ", c);
        // motors.update_motors();
        sleep_ms(10000);
        printf("ALIVE\n");
    }
}
