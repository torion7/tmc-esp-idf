// Host-Test-Stub: minimaler Ersatz fuer den ESP-IDF-Header, damit die
// IDF-freien Quellen (tmc_motion_decode.c, tmc_motion_units.c) mit dem
// Host-gcc compilieren. Nur was die eingebundenen Header brauchen.
#pragma once

typedef int esp_err_t;

#define ESP_OK                 0
#define ESP_FAIL              -1
#define ESP_ERR_NO_MEM         0x101
#define ESP_ERR_INVALID_ARG    0x102
#define ESP_ERR_INVALID_STATE  0x103
#define ESP_ERR_NOT_FOUND      0x105
#define ESP_ERR_NOT_SUPPORTED  0x106
#define ESP_ERR_TIMEOUT        0x107
