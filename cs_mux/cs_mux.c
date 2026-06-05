#include "cs_mux.h"

#include <stdlib.h>

#include "esp_check.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "cs_mux";

#define CS_MUX_ADDRESS_SETUP_US 2U
#define CS_MUX_ENABLE_SETUP_US  2U
#define CS_MUX_ENABLE_HOLD_US   2U
#define CS_MUX_INTER_FRAME_US   2U

// Mehrere TMC-Chips teilen sich EINEN 74HC138 (Adress-GPIOs) UND den SPI-Bus.
// Eine Transaktion ist erst sicher, wenn Kanal-Umschaltung + SPI-Transfer +
// Abwahl als Einheit laufen — sonst kann auf einem Dual-Core-Target ein zweiter
// Task den MUX-Kanal mitten in einer fremden Transaktion umschalten und ein
// Datagramm landet auf dem falschen Chip (z. B. ein Gate-Velocity-Befehl auf dem
// Foerder-Motor -> Scheibe dreht ungewollt). Der bus_lock (im gemeinsamen
// cs_mux-Handle) serialisiert daher select_cb..deselect_cb ueber ALLE Ports.
#define CS_MUX_BUS_LOCK_TIMEOUT_MS 1000U

struct cs_mux_dev_t {
    cs_mux_config_t   cfg;
    uint8_t           channel;
    SemaphoreHandle_t bus_lock;   // serialisiert select..deselect ueber alle Ports
};

static int level_for_state(bool selected, bool active_low)
{
    if (active_low) {
        return selected ? 0 : 1;
    }
    return selected ? 1 : 0;
}

static esp_err_t apply_address(cs_mux_handle_t h, uint8_t channel)
{
    ESP_RETURN_ON_FALSE(channel < 8U, ESP_ERR_INVALID_ARG, TAG, "channel out of range");
    ESP_RETURN_ON_ERROR(gpio_set_level(h->cfg.pin_a0, (channel & 0x01U) != 0U), TAG, "A0 failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(h->cfg.pin_a1, (channel & 0x02U) != 0U), TAG, "A1 failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(h->cfg.pin_a2, (channel & 0x04U) != 0U), TAG, "A2 failed");
    esp_rom_delay_us(CS_MUX_ADDRESS_SETUP_US);
    return ESP_OK;
}

esp_err_t cs_mux_init(const cs_mux_config_t *cfg, cs_mux_handle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "cfg null");
    ESP_RETURN_ON_FALSE(out_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "out null");
    ESP_RETURN_ON_FALSE(cfg->pin_a0 != GPIO_NUM_NC &&
                        cfg->pin_a1 != GPIO_NUM_NC &&
                        cfg->pin_a2 != GPIO_NUM_NC &&
                        cfg->pin_enable != GPIO_NUM_NC,
                        ESP_ERR_INVALID_ARG, TAG, "address/enable pins required");
    ESP_RETURN_ON_FALSE(cfg->default_channel < 8U, ESP_ERR_INVALID_ARG, TAG, "default channel invalid");

    cs_mux_handle_t h = calloc(1, sizeof(*h));
    ESP_RETURN_ON_FALSE(h != NULL, ESP_ERR_NO_MEM, TAG, "alloc failed");
    h->cfg = *cfg;
    h->channel = cfg->default_channel;

    h->bus_lock = xSemaphoreCreateMutex();
    if (h->bus_lock == NULL) {
        free(h);
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t io = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << cfg->pin_a0) |
                        (1ULL << cfg->pin_a1) |
                        (1ULL << cfg->pin_a2) |
                        (1ULL << cfg->pin_enable),
    };

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        vSemaphoreDelete(h->bus_lock);
        free(h);
        return err;
    }

    int idle = level_for_state(false, cfg->enable_active_low);
    err = gpio_set_level(cfg->pin_enable, idle);
    if (err == ESP_OK) {
        err = apply_address(h, h->channel);
    }
    if (err != ESP_OK) {
        vSemaphoreDelete(h->bus_lock);
        free(h);
        return err;
    }

    *out_handle = h;
    return ESP_OK;
}

void cs_mux_deinit(cs_mux_handle_t handle)
{
    if (handle == NULL) {
        return;
    }
    int idle = level_for_state(false, handle->cfg.enable_active_low);
    (void)gpio_set_level(handle->cfg.pin_enable, idle);
    if (handle->bus_lock != NULL) {
        vSemaphoreDelete(handle->bus_lock);
    }
    free(handle);
}

esp_err_t cs_mux_set_channel(cs_mux_handle_t handle, uint8_t channel)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");
    ESP_RETURN_ON_ERROR(apply_address(handle, channel), TAG, "set address failed");
    handle->channel = channel;
    return ESP_OK;
}

uint8_t cs_mux_get_channel(cs_mux_handle_t handle)
{
    return handle ? handle->channel : 0U;
}

esp_err_t cs_mux_assert(cs_mux_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");
    ESP_RETURN_ON_ERROR(apply_address(handle, handle->channel), TAG, "address failed");
    int level = level_for_state(true, handle->cfg.enable_active_low);
    ESP_RETURN_ON_ERROR(gpio_set_level(handle->cfg.pin_enable, level), TAG, "assert failed");
    esp_rom_delay_us(CS_MUX_ENABLE_SETUP_US);
    return ESP_OK;
}

esp_err_t cs_mux_deassert(cs_mux_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");
    esp_rom_delay_us(CS_MUX_ENABLE_HOLD_US);
    int level = level_for_state(false, handle->cfg.enable_active_low);
    ESP_RETURN_ON_ERROR(gpio_set_level(handle->cfg.pin_enable, level), TAG, "deassert failed");
    esp_rom_delay_us(CS_MUX_INTER_FRAME_US);
    return ESP_OK;
}

esp_err_t cs_mux_select_cb(void *handle_as_voidp)
{
    return cs_mux_assert((cs_mux_handle_t)handle_as_voidp);
}

esp_err_t cs_mux_deselect_cb(void *handle_as_voidp)
{
    return cs_mux_deassert((cs_mux_handle_t)handle_as_voidp);
}

// ---------------------------------------------------------------------------
// cs_mux_port: per-driver channel binding
// ---------------------------------------------------------------------------
struct cs_mux_port_t {
    cs_mux_handle_t mux;
    uint8_t         channel;
};

esp_err_t cs_mux_port_create(cs_mux_handle_t mux, uint8_t channel, cs_mux_port_handle_t *out_port)
{
    ESP_RETURN_ON_FALSE(mux      != NULL, ESP_ERR_INVALID_ARG, TAG, "mux null");
    ESP_RETURN_ON_FALSE(out_port != NULL, ESP_ERR_INVALID_ARG, TAG, "out null");
    ESP_RETURN_ON_FALSE(channel < 8U,     ESP_ERR_INVALID_ARG, TAG, "channel out of range");

    cs_mux_port_handle_t port = calloc(1, sizeof(*port));
    ESP_RETURN_ON_FALSE(port != NULL, ESP_ERR_NO_MEM, TAG, "alloc failed");
    port->mux     = mux;
    port->channel = channel;
    *out_port = port;
    return ESP_OK;
}

void cs_mux_port_destroy(cs_mux_port_handle_t port)
{
    free(port);
}

uint8_t cs_mux_port_get_channel(cs_mux_port_handle_t port)
{
    return port ? port->channel : 0U;
}

esp_err_t cs_mux_port_set_channel(cs_mux_port_handle_t port, uint8_t channel)
{
    ESP_RETURN_ON_FALSE(port != NULL, ESP_ERR_INVALID_ARG, TAG, "port null");
    ESP_RETURN_ON_FALSE(channel < 8U, ESP_ERR_INVALID_ARG, TAG, "channel out of range");
    port->channel = channel;
    return ESP_OK;
}

// Bus-Lock VOR Kanal-Umschaltung nehmen und erst in deselect_cb wieder freigeben:
// so sind Adress-GPIOs + SPI-Transfer (laeuft beim Aufrufer zwischen select_cb
// und deselect_cb) + Abwahl eine atomare Einheit ueber alle Ports desselben MUX.
// Schlaegt das Setup nach genommenem Lock fehl, wird der Lock hier wieder
// freigegeben (deselect_cb wird vom Treiber nur nach erfolgreichem select_cb
// aufgerufen).
esp_err_t cs_mux_port_select_cb(void *port_as_voidp)
{
    cs_mux_port_handle_t port = (cs_mux_port_handle_t)port_as_voidp;
    ESP_RETURN_ON_FALSE(port != NULL, ESP_ERR_INVALID_ARG, TAG, "port null");

    if (port->mux->bus_lock != NULL &&
        xSemaphoreTake(port->mux->bus_lock, pdMS_TO_TICKS(CS_MUX_BUS_LOCK_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "bus lock timeout");
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = cs_mux_set_channel(port->mux, port->channel);
    if (err == ESP_OK) {
        err = cs_mux_assert(port->mux);
    }
    if (err != ESP_OK && port->mux->bus_lock != NULL) {
        xSemaphoreGive(port->mux->bus_lock);   // Setup gescheitert -> Lock nicht halten
    }
    return err;
}

esp_err_t cs_mux_port_deselect_cb(void *port_as_voidp)
{
    cs_mux_port_handle_t port = (cs_mux_port_handle_t)port_as_voidp;
    ESP_RETURN_ON_FALSE(port != NULL, ESP_ERR_INVALID_ARG, TAG, "port null");
    esp_err_t err = cs_mux_deassert(port->mux);
    if (port->mux->bus_lock != NULL) {
        xSemaphoreGive(port->mux->bus_lock);
    }
    return err;
}
