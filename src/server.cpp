#include "server.hpp"
#include "server_internal.hpp"

#define WIFI_SSID "rover_pico_AP"
#define WIFI_PASS "rover_pass"

// returns true if fail
bool init_server_chip() {
    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi chip init failed\n");
        return true;
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

    // init web server
    httpd_init();
    printf("Initialized HTTP server!\n");

    // configure SSI and CGI
    init_ssi();
    init_cgi();
    printf("Configured SSI and CGI!\n");

    return false;
}

void init_ssi() {
    http_set_ssi_handler(ssi_handler, ssi_tags, LWIP_ARRAYSIZE(ssi_tags));
}

void init_cgi() {
    http_set_cgi_handlers(cgi_handlers, LWIP_ARRAYSIZE(cgi_handlers));
}

uint16_t ssi_handler(int index, char* pc_insert, int insert_len) {
    size_t printed_count = 0;  // number of values stored in buffer

    switch (index) {
        case (0): {  // one
            float data = 6.7;
            printed_count = snprintf(pc_insert, insert_len, "%f", data);
            break;
        }
        case (1): {  // two
            float data = 2.0;
            printed_count = snprintf(pc_insert, insert_len, "%f", data);
            break;
        }
        case (2): {  // led
            bool led_status = cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN);
            if (led_status) {
                printed_count = snprintf(pc_insert, insert_len, "ON");
            } else {
                printed_count = snprintf(pc_insert, insert_len, "OFF");
            }
            break;
        }
        default: {
            // dont do anything
        }
    }

    return printed_count;
}