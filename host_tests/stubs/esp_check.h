// Host-Test-Stub fuer esp_check.h: dieselbe Semantik wie im IDF (Fehler
// loggen, dann zurueckgeben bzw. `ret` setzen und springen), nur ohne das
// Funktions-/Zeilenpraefix in der Meldung.
#pragma once

#include "esp_err.h"
#include "esp_log.h"

#define ESP_RETURN_ON_ERROR(x, log_tag, ...) do {       \
        esp_err_t err_rc_ = (x);                        \
        if (err_rc_ != ESP_OK) {                        \
            ESP_LOGE(log_tag, __VA_ARGS__);             \
            return err_rc_;                             \
        }                                               \
    } while (0)

#define ESP_GOTO_ON_ERROR(x, goto_tag, log_tag, ...) do { \
        esp_err_t err_rc_ = (x);                        \
        if (err_rc_ != ESP_OK) {                        \
            ESP_LOGE(log_tag, __VA_ARGS__);             \
            ret = err_rc_;                              \
            goto goto_tag;                              \
        }                                               \
    } while (0)

#define ESP_RETURN_ON_FALSE(a, err_code, log_tag, ...) do { \
        if (!(a)) {                                     \
            ESP_LOGE(log_tag, __VA_ARGS__);             \
            return err_code;                            \
        }                                               \
    } while (0)
