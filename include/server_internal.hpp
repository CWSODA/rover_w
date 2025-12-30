#pragma once

#include "lwipopts.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/httpd.h"

#include "cgi.hpp"

void init_ssi();
void init_cgi();

// these tags must be under 8 bytes
const char* ssi_tags[] = {"one", "two", "led"};

// index is the index of the tag in ssi_tags
// pc_insert is pointer to char buffer that stores the data
// insert_len is the size of the buffer in bytes
uint16_t ssi_handler(int index, char* pc_insert, int insert_len);