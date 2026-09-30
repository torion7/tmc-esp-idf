// Host-Ersatz fuer die ESP-IDF-Funktionen, die die TMC-Registertreiber
// aufrufen (SPI, GPIO, Log). Ein 40-Bit-Datagramm mit gesetztem Bit 7 im
// Adressbyte ist ein Schreibzugriff: der Wert landet in s_regs[] und im
// Protokoll. Die Antwort traegt wie beim echten Chip die Daten des
// VORHERIGEN Datagramms (Lesepipeline des TMC-SPI-Protokolls).

#include "fake_spi.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#define FAKE_LOG_MAX 256

struct spi_device_t { int unused; };
static struct spi_device_t s_dev;

static uint32_t     s_regs[128];
static uint8_t      s_prev_addr;
static fake_write_t s_log[FAKE_LOG_MAX];
static size_t       s_log_len;
static int          s_warnings;
static int          s_errors;

void fake_reset(void)
{
    memset(s_regs, 0, sizeof(s_regs));
    s_prev_addr = 0;
    s_log_len   = 0;
    s_warnings  = 0;
    s_errors    = 0;
}

void fake_clear_log(void)
{
    s_log_len = 0;
}

size_t fake_write_count(void)
{
    return s_log_len;
}

const fake_write_t *fake_write_at(size_t i)
{
    return i < s_log_len ? &s_log[i] : NULL;
}

int fake_writes_to(uint8_t reg)
{
    int n = 0;
    for (size_t i = 0; i < s_log_len; ++i) {
        if (s_log[i].reg == reg) n++;
    }
    return n;
}

int fake_last_write_index(uint8_t reg)
{
    for (size_t i = s_log_len; i-- > 0;) {
        if (s_log[i].reg == reg) return (int)i;
    }
    return -1;
}

uint32_t fake_reg(uint8_t reg)
{
    return s_regs[reg & 0x7F];
}

int fake_warnings(void)
{
    return s_warnings;
}

int fake_errors(void)
{
    return s_errors;
}

// ---------------------------------------------------------------------------
// esp_log.h
// ---------------------------------------------------------------------------
void host_log(char level, const char *tag, const char *fmt, ...)
{
    if (level == 'W') s_warnings++;
    if (level == 'E') s_errors++;
    if (getenv("HOST_LOG") == NULL) return;

    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "%c (%s) ", level, tag);
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
}

// ---------------------------------------------------------------------------
// driver/gpio.h
// ---------------------------------------------------------------------------
esp_err_t gpio_config(const gpio_config_t *cfg)
{
    (void)cfg;
    return ESP_OK;
}

esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level)
{
    (void)gpio_num;
    (void)level;
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// driver/spi_master.h
// ---------------------------------------------------------------------------
esp_err_t spi_bus_initialize(spi_host_device_t host, const spi_bus_config_t *bus_config,
                             spi_common_dma_t dma_chan)
{
    (void)host;
    (void)bus_config;
    (void)dma_chan;
    return ESP_OK;
}

esp_err_t spi_bus_free(spi_host_device_t host)
{
    (void)host;
    return ESP_OK;
}

esp_err_t spi_bus_add_device(spi_host_device_t host, const spi_device_interface_config_t *dev_config,
                             spi_device_handle_t *handle)
{
    (void)host;
    (void)dev_config;
    *handle = &s_dev;
    return ESP_OK;
}

esp_err_t spi_bus_remove_device(spi_device_handle_t handle)
{
    (void)handle;
    return ESP_OK;
}

esp_err_t spi_device_polling_transmit(spi_device_handle_t handle, spi_transaction_t *trans)
{
    if (handle != &s_dev || trans->length != 40 || trans->tx_buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t *tx   = trans->tx_buffer;
    uint8_t        addr = tx[0] & 0x7F;
    uint32_t       value = ((uint32_t)tx[1] << 24) | ((uint32_t)tx[2] << 16) |
                           ((uint32_t)tx[3] <<  8) |  (uint32_t)tx[4];

    if (trans->rx_buffer != NULL) {
        uint8_t *rx = trans->rx_buffer;
        uint32_t prev = s_regs[s_prev_addr];
        rx[0] = 0;                               // SPI_STATUS: nichts gesetzt
        rx[1] = (uint8_t)(prev >> 24);
        rx[2] = (uint8_t)(prev >> 16);
        rx[3] = (uint8_t)(prev >>  8);
        rx[4] = (uint8_t)prev;
    }

    if (tx[0] & 0x80) {
        s_regs[addr] = value;
        if (s_log_len < FAKE_LOG_MAX) {
            s_log[s_log_len++] = (fake_write_t){ .reg = addr, .value = value };
        } else {
            fprintf(stderr, "fake_spi: Schreibprotokoll voll (fake_clear_log vergessen?)\n");
            abort();
        }
    }
    s_prev_addr = addr;
    return ESP_OK;
}
