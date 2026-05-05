// TMC5240 hardware ABN encoder adapter for tmc_motion_closedloop.
//
// X_ENC on the 5240 is already accumulated in microsteps when ENC_CONST
// is set to "microsteps per encoder count", so the read_position
// callback simply returns X_ENC widened to int64_t.

#include "tmc_motion_closedloop_5240.h"
#include "tmc5240.h"

#include <stdlib.h>

#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "tmc_cl_5240";

typedef struct {
    tmc_motion_handle_t motor_owning_lock;
    tmc5240_handle_t    chip;
    uint32_t            cached_status;  // last status read; cleared on chip
} enc5240_state_t;

static esp_err_t e5240_read_position_microsteps(void *user, int64_t *out)
{
    enc5240_state_t *st = (enc5240_state_t *)user;
    int32_t  x_enc = 0;
    uint32_t status = 0;
    esp_err_t err = tmc5240_read_encoder(st->chip, &x_enc, &status);
    if (err != ESP_OK) return err;
    if (status != 0U) (void)tmc5240_clear_encoder_status(st->chip, status);
    st->cached_status = status;     // remember for the next read_status call
    *out = (int64_t)x_enc;
    return ESP_OK;
}

static esp_err_t e5240_set_position_microsteps(void *user, int64_t microsteps)
{
    enc5240_state_t *st = (enc5240_state_t *)user;
    int32_t v;
    if      (microsteps > INT32_MAX) v = INT32_MAX;
    else if (microsteps < INT32_MIN) v = INT32_MIN;
    else                              v = (int32_t)microsteps;
    return tmc5240_set_encoder_position(st->chip, v);
}

static esp_err_t e5240_read_status(void *user, uint32_t *out_flags)
{
    enc5240_state_t *st = (enc5240_state_t *)user;
    *out_flags = st->cached_status;
    return ESP_OK;
}

esp_err_t tmc_closedloop_5240_make_encoder(
    tmc_motion_handle_t  motor_owning_lock,
    tmc5240_handle_t     chip,
    const tmc_closedloop_5240_encoder_init_t *init,
    tmc_closedloop_encoder_ops_t *out_ops)
{
    ESP_RETURN_ON_FALSE(motor_owning_lock && chip && init && out_ops,
                        ESP_ERR_INVALID_ARG, TAG, "null arg");

    // Configure the encoder block under the motor lock so we don't race
    // with motion commands using the same SPI bus.
    ESP_RETURN_ON_ERROR(tmc_motion_lock(motor_owning_lock, pdMS_TO_TICKS(1000)),
                        TAG, "motor lock");

    tmc5240_encoder_config_t cfg = {
        .encmode       = init->encmode,
        .enc_const     = init->enc_const,
        .enc_deviation = init->enc_deviation,
    };
    esp_err_t err = tmc5240_configure_encoder(chip, &cfg);
    tmc_motion_unlock(motor_owning_lock);
    if (err != ESP_OK) return err;

    enc5240_state_t *state = calloc(1, sizeof(*state));
    ESP_RETURN_ON_FALSE(state, ESP_ERR_NO_MEM, TAG, "alloc state");
    state->motor_owning_lock = motor_owning_lock;
    state->chip              = chip;

    out_ops->read_position_microsteps = e5240_read_position_microsteps;
    out_ops->set_position_microsteps  = e5240_set_position_microsteps;
    out_ops->read_status              = e5240_read_status;
    out_ops->user                     = state;
    return ESP_OK;
}

void tmc_closedloop_5240_release_encoder(tmc_closedloop_encoder_ops_t *ops)
{
    if (!ops) return;
    if (ops->user) {
        free(ops->user);
        ops->user = NULL;
    }
}
