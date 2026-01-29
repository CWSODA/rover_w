#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "serial_debug.hpp"
#include "hardware/irq.h"

constexpr uint UART_DEBUG_BAUDRATE = 115200;

void init_debug_uart() {
    gpio_set_function(DEBUG_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DEBUG_RX_PIN, GPIO_FUNC_UART);
    uart_init(uart0, UART_DEBUG_BAUDRATE);

    uart_set_translate_crlf(uart0, false);  // disable translations

    // irq_set_exclusive_handler(UART0_IRQ, debug_uart_handler);
    // irq_set_enabled(UART0_IRQ, true);
    // uart_set_irqs_enabled(uart0, true, false);  // enable rx interrupts
}

void debug_uart_handler() {
    // do nothing
}