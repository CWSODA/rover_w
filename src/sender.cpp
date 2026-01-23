// file to seperate sending data through serial and/or network
#include <stdio.h>
#include <cstring>
#include "pico/stdlib.h"

#include "sender.hpp"
#include "serial_debug.hpp"

void init_data_sending() {
// common setup for serial debug
#if USB_SERIAL
    stdio_init_all();

    // wait for usb to connect
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }
    printf("Connected to USB\n");
#endif

#if UART_SERIAL
    stdio_init_all();
    // ensure correct uart settings
    init_debug_uart();
#endif

#if NET_DATA
    // add in network initializer
    // add in wait for connection if required
#endif
}

void send_byte(const char c) {
#if UART_SERIAL | USB_SERIAL
    serial_send_byte(c);
#endif
#if NET_DATA
    // send through net
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