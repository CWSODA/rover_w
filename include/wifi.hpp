#pragma once
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#define WIFI_SSID "VM24B898"
#define WIFI_PASS "xVqtbjdqwyy4"

// returns true if fail
bool start_wifi() {
    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi chip init failed\n");
        return 1;
    }

    // turn on LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);

    // Enable wifi station
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS,
                                           CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
        // Read the ip address in a human readable way

        uint8_t* ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
        printf("IP address %d.%d.%d.%d\n", ip_address[0], ip_address[1],
               ip_address[2], ip_address[3]);
    }

    return 0;
}