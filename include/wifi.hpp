#pragma once
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#define WIFI_SSID "rover_pico_AP"
#define WIFI_PASS "i_love_networks"

#define SERVER_IP
#define PORT 6767

// returns true if fail
bool init_wifi_chip() {
    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi chip init failed\n");
        return -1;
    }

    // turn on LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);

    // Enable wifi access point
    cyw43_arch_enable_ap_mode(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK);
    printf("Established Access Point: %s, Pass: %s\n", WIFI_SSID, WIFI_PASS);

    // display IP address
    // netif takes an ITF enum, STA = 0, AP = 1
    uint8_t* ip_address = (uint8_t*)&(cyw43_state.netif[1].ip_addr.addr);
    printf("IP address %d.%d.%d.%d\n", ip_address[0], ip_address[1],
           ip_address[2], ip_address[3]);

    // create socket for a PC to connect to

    return 0;
}