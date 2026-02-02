// functions to streamline sending data
// through serial and/or network protocols
#define UART_SERIAL true
#define USB_SERIAL false
#define NET_DATA false

// initializes any enabled streams
void init_data_sending();

// sends byte through ALL enabled streams
void send_byte(const char c);

// sends float as 4 raw bytes
void send_float(const float val);

// sends byte as HEX converted string
void send_hex(const uint8_t byte);