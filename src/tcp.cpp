//  Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
//  SPDX-License-Identifier: BSD-3-Clause

#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include "settings.hpp"
#include "global_vars.hpp"

typedef struct TCP_SERVER_T_ {
    struct tcp_pcb* server_pcb;
    struct tcp_pcb* client_pcb;
    bool complete;
} TCP_SERVER_T;

static TCP_SERVER_T state = {NULL, NULL, false};

static err_t close_client() {
    err_t err = ERR_OK;

    // no client to close
    if (state.client_pcb == NULL) {
        return err;
    }

    // closes client and removes all callback functions
    tcp_arg(state.client_pcb, NULL);
    tcp_poll(state.client_pcb, NULL, 0);
    tcp_sent(state.client_pcb, NULL);
    tcp_recv(state.client_pcb, NULL);
    tcp_err(state.client_pcb, NULL);
    err = tcp_close(state.client_pcb);
    if (err != ERR_OK) {
        DBG("close failed %d, calling abort\n", err);
        tcp_abort(state.client_pcb);
        err = ERR_ABRT;
    }
    state.client_pcb = NULL;

    return err;
}

static err_t close_tcp_server() {
    // TCP_SERVER_T* state = (TCP_SERVER_T*)arg;
    err_t err = ERR_OK;

    // closes client if exists
    err = close_client();

    // closes server
    if (state.server_pcb) {
        tcp_arg(state.server_pcb, NULL);
        tcp_close(state.server_pcb);
        state.server_pcb = NULL;
    }
    cyw43_arch_deinit();
    return err;
}

// ends the server on error
static err_t tcp_server_result(int status) {
    DBG("closing server: code %d\n", status);
    state.complete = true;
    return close_tcp_server();
}

// on pico sent data
static err_t tcp_server_sent(void* arg, struct tcp_pcb* tpcb, u16_t len) {
    return ERR_OK;
}

// write to tcp client, flushes output
void tcp_write_data(const uint8_t* buf, uint16_t len) {
    if (state.client_pcb == NULL) return;  // no connection yet
    cyw43_arch_lwip_begin();  // lock lwip state since it is not thread safe.
                              // MUST USE!!!
    uint16_t free = tcp_sndbuf(state.client_pcb);
    uint16_t queued = tcp_sndqueuelen(state.client_pcb);
    // use WDBG here so it does not trigger more TCP outputs
#if DEBUG_TCP_WRITE
    WDBG("Free %zu, Len %zu, Queue: %zu/%zu\n", free, len, queued,
         TCP_SND_QUEUELEN);
#endif
    if (len > free) {
        // drop or skip send
        WDBG("Dropped data, mem overflow\n");
        cyw43_arch_lwip_end();
        return;
    }
    if (queued >= TCP_SND_QUEUELEN) {
        // drop or skip send
        WDBG("Dropped data, queue length overflow\n");
        cyw43_arch_lwip_end();
        return;
    }
    err_t err = tcp_write(state.client_pcb, buf, len, TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) {
        WDBG("Failed to queue data %d\n", err);
        // tcp_server_result(-1);
    }
    tcp_output(state.client_pcb);  // flush
    cyw43_arch_lwip_end();         // release lock
}

// on pico receive data from client
static err_t tcp_server_recv(void* arg, struct tcp_pcb* tpcb, struct pbuf* p,
                             err_t err) {
    if (p == NULL) {
        // client has closed the connection
        DBG("client has closed the connection\n");
        return close_client();
    }
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not
    // required, however you can use this method to cause an assertion in debug
    // mode, if this method is called when cyw43_arch_lwip_begin IS needed
    // cyw43_arch_lwip_check();

    // tot_len is length of packet, len is length of this buffer
    if (p->tot_len > 0) {
        // WDBG("tcp_server_recv %d err %d\n", p->len, err);

        // copy data to buffer
        for (size_t idx = 0; idx < p->tot_len; idx++) {
            uint8_t byte = ((uint8_t*)(p->payload))[idx];

            tcp_buffer.push_byte(byte);
        }

        // lets lwip know that the data has been processed
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);
    return ERR_OK;
}

// should not be called since there should be something sent within 5 seconds
static err_t tcp_server_poll(void* arg, struct tcp_pcb* tpcb) {
    WDBG("Pinging server\n");
    uint8_t buffer[] = {'$', 'H'};
    tcp_write_data(buffer, sizeof(buffer));
    // return tcp_server_result(arg, -1);  // no response is an error?
    return ERR_OK;
}

static void tcp_server_err(void* arg, err_t err) {
    if (err != ERR_ABRT) {
        DBG("tcp_client_err_fn %d\n", err);
        if (err == -14) return;  // do nothing on reconnect (code -14)
        tcp_server_result(err);
    }
}

static err_t tcp_server_accept(void* arg, struct tcp_pcb* client_pcb,
                               err_t err) {
    // TCP_SERVER_T* state = (TCP_SERVER_T*)arg;
    if (err != ERR_OK || client_pcb == NULL) {
        DBG("Failure in accept\n");
        tcp_server_result(err);
        return ERR_VAL;
    }
    DBG("Client connected\n");

    if (state.client_pcb != NULL) {
        DBG("Additional client connecting. Old client will be disconnected!\n");
        // probably close connection here
        // code below will lose the pcb handle to old client
    }

    state.client_pcb = client_pcb;
    tcp_arg(client_pcb, &state);
    tcp_sent(client_pcb, tcp_server_sent);
    tcp_recv(client_pcb, tcp_server_recv);
    tcp_poll(client_pcb, tcp_server_poll, TCP_POLL_TIME_S * 2);
    tcp_err(client_pcb, tcp_server_err);

    // return tcp_server_send_data(arg, state.client_pcb);
    return ERR_OK;
}

static bool tcp_server_open() {
    // TCP_SERVER_T* state = (TCP_SERVER_T*)arg;
    DBG("Starting server at %s on port %u\n",
        ip4addr_ntoa(netif_ip4_addr(netif_list)), TCP_PORT);

    // protocol control block
    struct tcp_pcb* pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        DBG("failed to create pcb\n");
        return false;
    }

    err_t err = tcp_bind(pcb, NULL, TCP_PORT);
    if (err) {
        DBG("failed to bind to port %u\n", TCP_PORT);
        return false;
    }

    state.server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state.server_pcb) {
        DBG("failed to listen\n");
        if (pcb) {
            tcp_close(pcb);
        }
        return false;
    }

    tcp_arg(state.server_pcb, &state);
    tcp_accept(state.server_pcb, tcp_server_accept);
    // other tcp functions are bound after accepting connection

    return true;
}

static void run_tcp_server() {
    if (!tcp_server_open()) {
        tcp_server_result(-1);
        DBG("failed to open tcp server\n");
        return;
    }
}

void init_wifi() {
    if (cyw43_arch_init()) {
        DBG("failed to initialise cyw43 chip!\n");
    }

    cyw43_arch_enable_sta_mode();

    DBG("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                           CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        WDBG("Failed to connect to wifi!\n");
    } else {
        WDBG("Connected.\n");
        // init tcp buffer before starting server
        run_tcp_server();
    }
}