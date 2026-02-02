// functions to send debug data through USB / uart

constexpr int DEBUG_TX_PIN = 0;
constexpr int DEBUG_RX_PIN = 1;

void init_debug_uart();

inline void serial_send_byte(const char c) { stdio_putchar_raw(c); }