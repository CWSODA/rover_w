#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "hardware/uart.h"

// self headers
#include "wifi.hpp"
#include "motors.hpp"
#include "lidar.hpp"

// UART debug
// screen /dev/tty.usbserial-FTU7C2WR 115200

// USB debug
// screen /dev/tty.usbmodem11101

int main() {
    stdio_init_all();

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi chip init failed\n");
        return -1;
    }

    // turn on LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);

    // INIT MOTORS
    MotorControl motors;

    // INIT LIDAR
    init_lidar_rx(LIDAR_RX_PIN);

    while (true) {
        char c = uart_getc(uart1);
        printf(" %02X ", c);
        // motors.update_motors();
        // sleep_ms(4000);
    }
}
