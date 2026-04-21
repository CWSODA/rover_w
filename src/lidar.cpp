#include "lidar.hpp"

#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#include "settings.hpp"
#include "sender.hpp"
#include "uart1_ringbuf.h"  // has uart1_rx_handler

constexpr int LIDAR_RX_BAUDRATE = 115200;

// Lidar update loop
void Lidar::update_lidar() {
    static uint16_t n = 0;

    uint8_t byte;
    while (ringbuf_pop(&byte)) {
        // send_hex(byte);
        parser_.parse_byte(byte);
    }

    uint16_t overflow_count = ringbuf_get_overflow_count();
    if (n != overflow_count) {
        DBG("OVERFLOWED %d TIMES!\n", overflow_count);
        n = overflow_count;
    }
}

// initiate UART RX ONLY
Lidar::Lidar() {
    gpio_set_function(LIDAR_RX_PIN, GPIO_FUNC_UART);
    uart_init(uart1, LIDAR_RX_BAUDRATE);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);

    uart_set_translate_crlf(uart1, false);  // disable translations

    irq_set_exclusive_handler(UART1_IRQ, uart1_rx_handler);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irqs_enabled(uart1, true, false);  // enable rx interrupts
}