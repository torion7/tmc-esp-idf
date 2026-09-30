// Host-Test-Stub fuer esp_log.h: ESP_LOGx landen in host_log() (fake_spi.c),
// das die Meldungen zaehlt und nur mit HOST_LOG=1 ausgibt. Das format-
// Attribut laesst gcc die Formatstrings pruefen wie beim IDF-Build.
#pragma once

void host_log(char level, const char *tag, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

#define ESP_LOGE(tag, ...) host_log('E', tag, __VA_ARGS__)
#define ESP_LOGW(tag, ...) host_log('W', tag, __VA_ARGS__)
#define ESP_LOGI(tag, ...) host_log('I', tag, __VA_ARGS__)
#define ESP_LOGD(tag, ...) host_log('D', tag, __VA_ARGS__)
#define ESP_LOGV(tag, ...) host_log('V', tag, __VA_ARGS__)
