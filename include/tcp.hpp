#pragma once

void init_wifi(bool& has_wifi);

void tcp_write_data(const uint8_t* buf, uint16_t len);

void flush_tcp();  // flushes the tcp output in buffer