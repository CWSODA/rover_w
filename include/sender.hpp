// functions to streamline sending data
// through serial and/or network protocols
#include "global.hpp"

// initializes any enabled streams
void init_data_sending();

// sends byte through ALL enabled streams
void send_byte(const char c);

// sends float as 4 raw bytes
void send_float(const float val);

// sends byte as HEX converted string
void send_hex(const uint8_t byte);

// toggle for debug printf
void dbg_log(const char* fmt, ...);
#if DEBUG_LOG
#include <stdio.h>
#define DBG(...) printf(__VA_ARGS__)
#else
#define DBG(...)
#endif