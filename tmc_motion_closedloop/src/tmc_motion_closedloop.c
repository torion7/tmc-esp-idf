// Closed-loop regulator core. Handle-based, encoder-source-agnostic.

#include "tmc_motion_closedloop.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "tmc_cl";

#define CL_TASK_STACK_BYTES   4096
#define CL_TASK_PRIORITY      4
#define CL_DEFAULT_PERIOD_MS  20U
#define CL_MIN_PERIOD_MS       5U
#define CL_MAX_PERIOD_MS    1000U

struct tmc_closedloop_s {
    tmc_motion_handle_t          motor;
    tmc_closedloop_encoder_ops_t encoder;
    SemaphoreHandle_t            cfg_lock;

    bool                         active_correction;
    uint32_t                     period_ms;
    uint32_t                     deviation_warn_microsteps;
    uint32_t                     deadband_steps;
    uint32_t                     max_correction_steps;
    uint16_t                     kp_per_mille;

    int32_t                      intended_xtarget;
    tmc_closedloop_status_t      status;

    TaskHandle_t                 task;
    bool                         shutdown;
};

// ===========================================================================
// Helpers
// ===========================================================================
static int32_t clamp_i64_to_i32(int64_t v)
{
    if (v > INT32_MAX) return INT32_MAX;
    if (v < INT32_MIN) return INT32_MIN;
    return (int32_t)v;
}

static uint32_t clamp_period(uint32_t v)
{
    if (v == 0)                  return CL_DEFAULT_PERIOD_MS;
    if (v < CL_MIN_PERIOD_MS)    return CL_MIN_PERIOD_MS;
    if (v > CL_MAX_PERIOD_MS)    return CL_MAX_PERIOD_MS;
    return v;
}

// ===========================================================================
// Tick (the regulator). Holds motor lock for the duration.
// ===========================================================================
static void cl_tick(tmc_closedloop_handle_t h)
{
    if (tmc_motion_lock(h->motor, pdMS_TO_TICKS(200)) != ESP_OK) return;

    const tmc_motion_driver_ops_t *ops  = tmc_motion_get_ops(h->motor);
    void                          *chip = tmc_motion_get_chip_handle(h->motor);

    int64_t  enc_microsteps = 0;
    uint32_t enc_status     = 0;
    if (h->encoder.read_position_microsteps(h->encoder.user, &enc_microsteps) != ESP_OK) {
        tmc_motion_unlock(h->motor);
        return;
    }
    if (h->encoder.read_status) {
        (void)h->encoder.read_status(h->encoder.user, &enc_status);
    }

    uint32_t raw_xactual = 0, raw_rampmode = 0;
    if (ops->read_register(chip, ops->reg_xactual,  &raw_xactual)  != ESP_OK ||
        ops->read_register(chip, ops->reg_rampmode, &raw_rampmode) != ESP_OK) {
        tmc_motion_unlock(h->motor);
        return;
    }
    int32_t xactual  = (int32_t)raw_xactual;
    uint8_t rampmode = (uint8_t)(raw_rampmode & 0x3U);

    int64_t err64       = (int64_t)h->intended_xtarget - enc_microsteps;
    int32_t error_steps = clamp_i64_to_i32(err64);

    // Refresh status snapshot.
    h->status.enable                 = true;
    h->status.active_correction      = h->active_correction;
    h->status.x_enc_microsteps       = enc_microsteps;
    h->status.x_enc                  = (int32_t)enc_microsteps;  // raw view (truncated for display)
    h->status.xactual                = xactual;
    h->status.intended_xtarget       = h->intended_xtarget;
    h->status.position_error         = error_steps;
    h->status.enc_status             = enc_status;
    h->status.warn_deviation         = (enc_status & 0x2U) != 0U;
    h->status.n_event                = (enc_status & 0x1U) != 0U;
    h->status.last_correction        = clamp_i64_to_i32((int64_t)xactual - enc_microsteps);
    h->status.update_count++;
    h->status.last_update_ms         = (uint64_t)(esp_timer_get_time() / 1000ULL);

    // Active correction: direct encoder-position mapping (no software gain).
    if (h->active_correction && rampmode == 0U) {
        int32_t chip_target = clamp_i64_to_i32((int64_t)xactual + err64);
        esp_err_t err = ops->write_register(chip, ops->reg_xtarget, (uint32_t)chip_target);
        if (err != ESP_OK) ESP_LOGW(TAG, "XTARGET write failed: %s", esp_err_to_name(err));
    }

    tmc_motion_unlock(h->motor);
}

static void cl_task(void *arg)
{
    tmc_closedloop_handle_t h = (tmc_closedloop_handle_t)arg;
    ESP_LOGI(TAG, "[%s] closed-loop task started", tmc_motion_get_name(h->motor));

    while (!h->shutdown) {
        uint32_t period;
        if (xSemaphoreTake(h->cfg_lock, portMAX_DELAY) == pdTRUE) {
            period = h->period_ms;
            xSemaphoreGive(h->cfg_lock);
        } else {
            period = CL_DEFAULT_PERIOD_MS;
        }

        cl_tick(h);
        vTaskDelay(pdMS_TO_TICKS(period));
    }
    h->task = NULL;
    vTaskDelete(NULL);
}

// ===========================================================================
// Public API
// ===========================================================================
esp_err_t tmc_closedloop_create(const tmc_closedloop_config_t *cfg, tmc_closedloop_handle_t *out)
{
    ESP_RETURN_ON_FALSE(cfg && out, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_FALSE(cfg->motor,                              ESP_ERR_INVALID_ARG, TAG, "no motor");
    ESP_RETURN_ON_FALSE(cfg->encoder.read_position_microsteps,    ESP_ERR_INVALID_ARG, TAG, "no enc.read");
    ESP_RETURN_ON_FALSE(cfg->encoder.set_position_microsteps,     ESP_ERR_INVALID_ARG, TAG, "no enc.set");

    tmc_closedloop_handle_t h = calloc(1, sizeof(*h));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "alloc");

    h->cfg_lock = xSemaphoreCreateMutex();
    if (!h->cfg_lock) { free(h); return ESP_ERR_NO_MEM; }

    h->motor                     = cfg->motor;
    h->encoder                   = cfg->encoder;
    h->active_correction         = cfg->active_correction;
    h->period_ms                 = clamp_period(cfg->period_ms);
    h->deviation_warn_microsteps = cfg->deviation_warn_microsteps;
    h->deadband_steps            = cfg->deadband_steps;
    h->max_correction_steps      = cfg->max_correction_steps;
    h->kp_per_mille              = (cfg->kp_per_mille > 5000) ? 5000 : cfg->kp_per_mille;
    h->shutdown                  = false;

    BaseType_t ok = xTaskCreate(cl_task, "tmc_cl", CL_TASK_STACK_BYTES,
                                h, CL_TASK_PRIORITY, &h->task);
    if (ok != pdPASS) {
        vSemaphoreDelete(h->cfg_lock);
        free(h);
        return ESP_ERR_NO_MEM;
    }

    *out = h;
    return ESP_OK;
}

void tmc_closedloop_destroy(tmc_closedloop_handle_t h)
{
    if (!h) return;
    h->shutdown = true;
    for (int i = 0; i < 50 && h->task; ++i) vTaskDelay(pdMS_TO_TICKS(20));
    if (h->cfg_lock) vSemaphoreDelete(h->cfg_lock);
    free(h);
}

esp_err_t tmc_closedloop_set_active_correction(tmc_closedloop_handle_t h, bool on)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null");
    if (xSemaphoreTake(h->cfg_lock, portMAX_DELAY) == pdTRUE) {
        h->active_correction = on;
        xSemaphoreGive(h->cfg_lock);
    }
    return ESP_OK;
}

esp_err_t tmc_closedloop_set_period_ms(tmc_closedloop_handle_t h, uint32_t period_ms)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null");
    if (xSemaphoreTake(h->cfg_lock, portMAX_DELAY) == pdTRUE) {
        h->period_ms = clamp_period(period_ms);
        xSemaphoreGive(h->cfg_lock);
    }
    return ESP_OK;
}

esp_err_t tmc_closedloop_set_deviation_warn(tmc_closedloop_handle_t h, uint32_t microsteps)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null");
    if (xSemaphoreTake(h->cfg_lock, portMAX_DELAY) == pdTRUE) {
        h->deviation_warn_microsteps = microsteps;
        xSemaphoreGive(h->cfg_lock);
    }
    return ESP_OK;
}

esp_err_t tmc_closedloop_align_to_motor(tmc_closedloop_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null");
    ESP_RETURN_ON_ERROR(tmc_motion_lock(h->motor, pdMS_TO_TICKS(1000)), TAG, "motor lock");

    const tmc_motion_driver_ops_t *ops  = tmc_motion_get_ops(h->motor);
    void                          *chip = tmc_motion_get_chip_handle(h->motor);

    uint32_t raw_xactual = 0;
    esp_err_t err = ops->read_register(chip, ops->reg_xactual, &raw_xactual);
    if (err == ESP_OK) {
        int32_t xactual = (int32_t)raw_xactual;
        err = h->encoder.set_position_microsteps(h->encoder.user, (int64_t)xactual);
        if (err == ESP_OK) {
            h->intended_xtarget          = xactual;
            h->status.intended_xtarget   = xactual;
        }
    }
    tmc_motion_unlock(h->motor);
    return err;
}

void tmc_closedloop_set_intended_xtarget(tmc_closedloop_handle_t h, int32_t microsteps)
{
    if (!h) return;
    h->intended_xtarget = microsteps;
    h->status.intended_xtarget = microsteps;
}

esp_err_t tmc_closedloop_get_status(tmc_closedloop_handle_t h, tmc_closedloop_status_t *out)
{
    ESP_RETURN_ON_FALSE(h && out, ESP_ERR_INVALID_ARG, TAG, "null");
    *out = h->status;
    return ESP_OK;
}
