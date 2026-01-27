#include "uart1_ringbuf.h"
#include "global.h"

constexpr uint16_t RB_MASK = 0b11'1111'1111;
constexpr uint16_t RB_SIZE = RB_MASK + 1;
// mask is all ones if buf_size is power of 2

static volatile uint8_t rx_buf[RB_SIZE];
static volatile uint16_t rb_head = 0;
static volatile uint16_t rb_tail = 0;
static uint16_t rb_overflow_count = 0;

static inline bool ringbuf_push(const uint8_t byte) {
    // 1110 & 1111 = 1110
    // 1111 & 1111 = 1111
    // 10000 & 01111 = 00000 wrap around
    uint16_t next = (rb_head + 1) & RB_MASK;

    if (next == rb_tail) {
        ++rb_overflow_count;
        return false;  // ringbuffer is full
    }

    rx_buf[rb_head] = byte;  // insert byte
    rb_head = next;          // update index
    return true;
}

void uart1_rx_handler() {
    while (uart_is_readable(uart1)) {
        uint8_t ch = uart_getc(uart1);
        ringbuf_push(ch);
    }
}

// popped value goes to pointer, returns false if empty
bool ringbuf_pop(uint8_t* byte) {
    if (rb_tail == rb_head) {
        return false;  // empty
    }

    *byte = rx_buf[rb_tail];
    // update tail
    rb_tail = (rb_tail + 1) & RB_MASK;
    return true;
}

uint16_t ringbuf_get_overflow_count() { return rb_overflow_count; }

// void test_ringbuf() {
//     uint8_t rx_buf[RB_SIZE];
//     uint16_t rb_head = 0;
//     uint16_t rb_tail = 0;
//     uint16_t rb_overflow_count = 0;

//     while (true) {
//         // 1110 & 1111 = 1110
//         // 1111 & 1111 = 1111
//         // 10000 & 01111 = 00000 wrap around
//         uint16_t next = (rb_head + 1) & RB_MASK;

//         if (next == rb_tail) {
//             DBG("OVERFLOW!\n");
//             return;
//         }

//         DBG("INSERTED AT %zu\n", rb_head);
//         rx_buf[rb_head] = 1;
//         rb_head = next;
//     }
// }