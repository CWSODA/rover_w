#pragma once

#include "pico/stdlib.h"

void uart1_rx_handler();

// popped value goes to pointer, returns false if empty
bool ringbuf_pop(uint8_t* byte);

// returns overflow count
uint16_t ringbuf_get_overflow_count();

// for debugging only
// void test_ringbuf();