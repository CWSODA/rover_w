void init_wifi();

void tcp_write_data(void* arg, struct tcp_pcb* tpcb, const uint8_t* buf,
                    uint16_t len);
