#pragma once

#include "pico/stdlib.h"

void init_wifi(bool& has_wifi);

void tcp_buffer_data(const uint8_t* buf, uint16_t len);
void tcp_write_data(const uint8_t* buf, uint16_t len);
void flush_tcp_write_buffer();  // flushes the tcp output in buffer
