// functions to streamline sending data
// through serial and/or network protocols
#define UART_SERIAL true
#define USB_SERIAL false
#define NET_DATA false

void init_data_sending();

void send_byte(const char c);

void send_float(const float val);