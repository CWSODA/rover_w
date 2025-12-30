#pragma once
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "lwip/pbuf.h"
#include <string>

// interface = access point
#define IFACE CYW43_ITF_AP
#define WIFI_SSID "rover_pico"
#define WIFI_PASS "hello_world"
#define TCP_PORT 4242

#define BUF_SIZE 2048

struct TCP_ServerData {
    struct tcp_pcb* server_pcb;
    struct tcp_pcb* client_pcb;
    bool complete;
    uint8_t buffer_sent[BUF_SIZE];
    uint8_t buffer_recv[BUF_SIZE];
    int sent_len;
    int recv_len;
    int run_count;
};

void write_tcp_data(tcp_pcb* pcb, const std::string& msg) {
    // write to tcp connection with no flags
    // flags being: expect more data, don't send immediately
    if (!tcp_write(pcb, msg.data(), msg.size(), 0)) {
        printf("Error writing TCP data, trying again!\n");
        write_tcp_data(pcb, msg);
    }

    // immediately send data
    tcp_output(pcb);
}

static err_t tcp_recv_callback(void* arg, struct tcp_pcb* client_pcb,
                               struct pbuf* p, err_t err) {
    // received data (p) is null pointer if connection is closed
    if (!p) {
        tcp_close(client_pcb);
        printf("TCP Connection Disconnected!\n");
        return ERR_OK;
    }

    // Print received data
    std::string msg((char*)p->payload, p->len);
    printf("Received: %s\n", msg);

    // Send data back
    write_tcp_data(client_pcb, msg);

    tcp_recved(client_pcb, p->len);  // gpt
    pbuf_free(p);
    return ERR_OK;
}

static err_t tcp_sent_callback(void* arg, struct tcp_pcb* client_pcb,
                               uint16_t len) {
    printf("Data Sent!\n");

    return ERR_OK;
}

static err_t tcp_accept_callback(void* arg, struct tcp_pcb* client_pcb,
                                 err_t err) {
    printf("Client connected!\n");
    tcp_recv(client_pcb, tcp_recv_callback);
    tcp_sent(client_pcb, tcp_sent_callback);

    const std::string msg = "Hello from Pico W!\n";
    write_tcp_data(client_pcb, msg);

    return ERR_OK;
}

static err_t tcp_server_accept(void* arg, struct tcp_pcb* client_pcb,
                               err_t err) {
    TCP_ServerData* server_data = (TCP_ServerData*)arg;
    if (err != ERR_OK || client_pcb == NULL) {
        printf("Failure in accept\n");
        // tcp_server_result(arg, err);
        return ERR_VAL;
    }
    printf("Client connected\n");

    server_data->client_pcb = client_pcb;
    tcp_arg(client_pcb, server_data);
    // tcp_sent(client_pcb, tcp_server_sent);
    // tcp_recv(client_pcb, tcp_server_recv);
    //  tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2);
    // tcp_err(client_pcb, tcp_server_err);

    // return tcp_server_send_data(arg, server_data->client_pcb);

    return ERR_OK;
}

bool open_tcp_server(TCP_ServerData& arg) {
    TCP_ServerData* server_data = &arg;
    printf("Starting server at %s on port %u\n",
           ip4addr_ntoa(netif_ip4_addr(netif_list)), TCP_PORT);

    struct tcp_pcb* pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        printf("failed to create pcb\n");
        return false;
    }

    err_t err = tcp_bind(pcb, NULL, TCP_PORT);
    if (err) {
        printf("failed to bind to port %u\n", TCP_PORT);
        return false;
    }

    server_data->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!server_data->server_pcb) {
        printf("failed to listen\n");
        if (pcb) {
            tcp_close(pcb);
        }
        return false;
    }

    tcp_arg(server_data->server_pcb, &server_data);
    tcp_accept(server_data->server_pcb, tcp_server_accept);

    return true;
}

int init_tcp() {
    if (cyw43_arch_init()) {
        printf("WiFi init failed\n");
        return -1;
    }

    // turn on LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);

    printf("Establishing AP Station...\n");
    cyw43_arch_enable_ap_mode(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK);
    printf("Established Access Point with SSID: %s, Password: %s\n", WIFI_SSID,
           WIFI_PASS);

    // // Create a TCP Protocol Control Block
    // struct tcp_pcb* pcb = tcp_new();
    // tcp_bind(pcb, IP_ADDR_ANY, SERVER_PORT);

    // pcb = tcp_listen(pcb);
    // tcp_accept(pcb, tcp_accept_callback);

    // wait for ip address to be given
    while (cyw43_state.netif[IFACE].ip_addr.addr == 0) {
        sleep_ms(100);
    }

    uint8_t* ip_address = (uint8_t*)&(cyw43_state.netif[IFACE].ip_addr.addr);
    printf("IP address: %d.%d.%d.%d\n", ip_address[0], ip_address[1],
           ip_address[2], ip_address[3]);
    printf("TCP server listening on port %d...\n", TCP_PORT);

    TCP_ServerData tcp_server_data;
    open_tcp_server(tcp_server_data);

    return 0;
}