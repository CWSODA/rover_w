#pragma once

#include "lwipopts.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/httpd.h"

const char* led_cgi_handler(int index, int n_params, char* pc_param[],
                            char* pc_value[]);

// handlers corresponding to each id
const tCGI cgi_handlers[] = {
    {"/led.cgi", led_cgi_handler},
};