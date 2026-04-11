// file to seperate sending data through serial and/or network
#include <stdio.h>
#include <cstring>
#include "pico/stdlib.h"

#include "sender.hpp"
#include "global.hpp"
#include "tcp.hpp"

static void init_debug_uart() {
    gpio_set_function(DEBUG_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DEBUG_RX_PIN, GPIO_FUNC_UART);

    uart_set_translate_crlf(uart0, false);  // disable translations

    // enable uart0 debug with only TX
    stdio_uart_init_full(uart0, UART_DEBUG_BAUDRATE, DEBUG_TX_PIN, -1);
}

void init_data_sending() {
// common setup for serial debug
#if USB_SERIAL
    stdio_init_all();

    // wait for usb to connect
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }
    DBG("Connected to USB\n");
#endif

#if UART_SERIAL
    // ensure correct uart settings
    init_debug_uart();
#endif

#if NET_DATA
    // add in network initializer
    // add in wait for connection if required
    init_wifi();
#endif
}

void send_byte(const uint8_t byte) {
#if UART_SERIAL | USB_SERIAL
    stdio_putchar_raw(byte);
#endif
#if NET_DATA
    // send through net
    tcp_write_data(NULL, NULL, &byte, 1);
#endif
}

void send_float(const float val) {
    uint8_t bytes[4];
    memcpy(bytes, &val, 4);
    send_byte(bytes[0]);
    send_byte(bytes[1]);
    send_byte(bytes[2]);
    send_byte(bytes[3]);
}

static const char hex_table[] = "0123456789ABCDEF";
void send_hex(const uint8_t byte) {
    send_byte(hex_table[byte >> 4]);
    send_byte(hex_table[byte & 0x0F]);
    send_byte('\n');
}