// Host-Test-Stub fuer driver/spi_master.h: nur die Typen und Funktionen, die
// die TMC-Registertreiber benutzen. Implementierung in fake_spi.c.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    SPI1_HOST = 0,
    SPI2_HOST = 1,
    SPI3_HOST = 2,
} spi_host_device_t;

typedef enum {
    SPI_DMA_DISABLED = 0,
    SPI_DMA_CH_AUTO  = 3,
} spi_common_dma_t;

typedef struct {
    int mosi_io_num;
    int miso_io_num;
    int sclk_io_num;
    int quadwp_io_num;
    int quadhd_io_num;
    int max_transfer_sz;
} spi_bus_config_t;

typedef struct {
    uint8_t mode;
    int     clock_speed_hz;
    int     spics_io_num;
    int     queue_size;
} spi_device_interface_config_t;

typedef struct {
    size_t      length;       // Bits
    size_t      rxlength;     // Bits
    const void *tx_buffer;
    void       *rx_buffer;
} spi_transaction_t;

typedef struct spi_device_t *spi_device_handle_t;

esp_err_t spi_bus_initialize(spi_host_device_t host, const spi_bus_config_t *bus_config,
                             spi_common_dma_t dma_chan);
esp_err_t spi_bus_free(spi_host_device_t host);
esp_err_t spi_bus_add_device(spi_host_device_t host, const spi_device_interface_config_t *dev_config,
                             spi_device_handle_t *handle);
esp_err_t spi_bus_remove_device(spi_device_handle_t handle);
esp_err_t spi_device_polling_transmit(spi_device_handle_t handle, spi_transaction_t *trans);
