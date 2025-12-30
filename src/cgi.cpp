#include "cgi.hpp"

const char* led_cgi_handler(int index, int n_params, char* pc_param[],
                            char* pc_value[]) {
    // check for requests: /<name>.cgi?<name>=<variable>
    // ex: /led.cgi?led=x
    if (strcmp(pc_param[0], "led") == 0) {
        if (strcmp(pc_value[0], "0") == 0)
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
        if (strcmp(pc_value[0], "1") == 0)
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
    }

    return "/index.shtml";
}