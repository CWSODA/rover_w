// functions to send debug data through USB / uart

#define DEBUG_TX_PIN 0
#define DEBUG_RX_PIN 1

void init_debug_uart();

void debug_uart_handler();

inline void serial_send_byte(const char c) { stdio_putchar_raw(c); }
void serial_send_float(const float val);