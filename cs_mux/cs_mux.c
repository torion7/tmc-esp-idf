#include "cs_mux.h"

#include <stdlib.h>

#include "esp_check.h"
#include "esp_rom_sys.h"

static const char *TAG = "cs_mux";

#define CS_MUX_ADDRESS_SETUP_US 2U
#define CS_MUX_ENABLE_SETUP_US  2U
#define CS_MUX_ENABLE_HOLD_US   2U
#define CS_MUX_INTER_FRAME_US   2U

struct cs_mux_dev_t {
    cs_mux_config_t cfg;
    uint8_t         channel;
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

    gpio_config_t io = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << cfg->pin_a0) |
                        (1ULL << cfg->pin_a1) |
                        (1ULL << cfg->pin_a2) |
                        (1ULL << cfg->pin_enable),
    };

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        free(h);
        return err;
    }

    int idle = level_for_state(false, cfg->enable_active_low);
    err = gpio_set_level(cfg->pin_enable, idle);
    if (err == ESP_OK) {
        err = apply_address(h, h->channel);
    }
    if (err != ESP_OK) {
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

esp_err_t cs_mux_port_select_cb(void *port_as_voidp)
{
    cs_mux_port_handle_t port = (cs_mux_port_handle_t)port_as_voidp;
    ESP_RETURN_ON_FALSE(port != NULL, ESP_ERR_INVALID_ARG, TAG, "port null");
    ESP_RETURN_ON_ERROR(cs_mux_set_channel(port->mux, port->channel), TAG, "set channel");
    return cs_mux_assert(port->mux);
}

esp_err_t cs_mux_port_deselect_cb(void *port_as_voidp)
{
    cs_mux_port_handle_t port = (cs_mux_port_handle_t)port_as_voidp;
    ESP_RETURN_ON_FALSE(port != NULL, ESP_ERR_INVALID_ARG, TAG, "port null");
    return cs_mux_deassert(port->mux);
}
