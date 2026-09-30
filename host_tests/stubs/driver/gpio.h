// Host-Test-Stub fuer driver/gpio.h: nur die Typen und Funktionen, die die
// TMC-Registertreiber benutzen. Implementierung in fake_spi.c.
#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef enum {
    GPIO_NUM_NC = -1,
    GPIO_NUM_0  = 0,
    GPIO_NUM_1,
    GPIO_NUM_2,
    GPIO_NUM_3,
    GPIO_NUM_4,
    GPIO_NUM_5,
} gpio_num_t;

typedef enum {
    GPIO_MODE_DISABLE = 0,
    GPIO_MODE_INPUT   = 1,
    GPIO_MODE_OUTPUT  = 2,
} gpio_mode_t;

typedef struct {
    uint64_t    pin_bit_mask;
    gpio_mode_t mode;
} gpio_config_t;

esp_err_t gpio_config(const gpio_config_t *cfg);
esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level);
