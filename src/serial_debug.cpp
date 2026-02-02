#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "serial_debug.hpp"
#include "hardware/irq.h"

// 921600 baudrate
constexpr uint UART_DEBUG_BAUDRATE = 115200 * 8;

void init_debug_uart() {
    gpio_set_function(DEBUG_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DEBUG_RX_PIN, GPIO_FUNC_UART);

    uart_set_translate_crlf(uart0, false);  // disable translations

    // enable uart0 debug with only TX
    stdio_uart_init_full(uart0, UART_DEBUG_BAUDRATE, DEBUG_TX_PIN, -1);
}