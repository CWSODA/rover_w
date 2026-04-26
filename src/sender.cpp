// file to seperate sending data through serial and/or network
#include <stdio.h>
#include <cstring>
#include "pico/stdlib.h"

#include "settings.hpp"
#include "sender.hpp"
#include "tcp.hpp"

static void init_debug_uart();

void init_data_sending(bool& has_wifi) {
// common setup for serial debug
#if USB_DEBUG | USB_SEND
    if (!stdio_usb_init()) {
        DBG("Failed to initialize USB debugging\n");
    } else {
#if USB_WAIT
        // wait for usb to connect
        while (!stdio_usb_connected()) {
            sleep_ms(10);
        }
        sleep_ms(10);
        DBG("Connected to USB\n");
#endif
    }
#endif

#if UART_DEBUG
    // ensure correct uart settings
    init_debug_uart();
#endif

#if NET_DEBUG
    // add in network initializer
    // add in wait for connection if required
    init_wifi(has_wifi);
#endif
}

void send_byte(const uint8_t byte) {
#if UART_DEBUG | USB_SEND
    stdio_putchar_raw(byte);
#endif
#if NET_DEBUG
    // send through net
    tcp_write_data(&byte, 1);
#endif
}

void send_bytes(const uint8_t* c, size_t len) {
#if UART_DEBUG | USB_SEND
    fwrite(c, 1, len, stdout);
#endif
#if NET_DEBUG
    tcp_write_data(c, len);
#endif
}

void send_float(const float val) {
    uint8_t bytes[4];
    memcpy(bytes, &val, 4);
    send_bytes(bytes, sizeof(bytes));
}

static const char hex_table[] = "0123456789ABCDEF";
void send_hex(const uint8_t byte) {
    uint8_t msg[] = {hex_table[byte >> 4], hex_table[byte & 0x0F], '\n'};
    send_bytes(msg, sizeof(msg));
}

static void init_debug_uart() {
    gpio_set_function(DEBUG_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DEBUG_RX_PIN, GPIO_FUNC_UART);

    uart_set_translate_crlf(uart0, false);  // disable translations

    // enable uart0 debug with only TX
    stdio_uart_init_full(uart0, UART_DEBUG_BAUDRATE, DEBUG_TX_PIN, -1);
}

void dbg_log(const char* fmt, ...) {
    static char buf[2048];

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0) return;
    if (len > sizeof(buf)) len = sizeof(buf);

#if UART_DEBUG | USB_DEBUG | USB_SEND
    fwrite(buf, 1, len, stdout);
#endif

#ifdef NET_DEBUG
    tcp_write_data((const uint8_t*)buf, len);
#endif
}