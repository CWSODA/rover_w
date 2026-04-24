#pragma once

// functions to streamline sending data
// through serial and/or network protocols
#include "settings.hpp"

// initializes any enabled streams
void init_data_sending();

// sends byte through ALL enabled streams
// sending through TCP does not flush!
// send as bytes to flush
void send_byte(const uint8_t c);

// sends bytes through ALL enabled streams
void send_bytes(const uint8_t* c, size_t len);

// sends float as 4 raw bytes
void send_float(const float val);

// sends byte as HEX converted string
void send_hex(const uint8_t byte);

// toggle for debug printf
void dbg_log(const char* fmt, ...);
#if DEBUG_LOG
#include <stdio.h>
#define DBG(...) dbg_log(__VA_ARGS__)  // prints to net + wired
#define WDBG(...) printf(__VA_ARGS__)  // only prints to wired (USB/UART)
#else
#define DBG(...)
#define WDBG(...)
#endif