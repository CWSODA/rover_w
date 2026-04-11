// functions to send debug data through USB / uart
void init_debug_uart();

inline void serial_send_byte(const char c) { stdio_putchar_raw(c); }