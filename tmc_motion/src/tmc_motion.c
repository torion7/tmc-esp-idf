// tmc_motion core: lifecycle, locking, diag refresh / decode, snapshot.
// Motion-command implementations live in tmc_motion_motion.c; stallguard
// in tmc_motion_stallguard.c.

#include "tmc_motion.h"
#include "tmc_motion_internal.h"

#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "tmc_motion";

// ===========================================================================
// Locking
// ===========================================================================
esp_err_t tmc_motion_lock_internal(tmc_motion_handle_t h)
{
    if (h == NULL || h->lock == NULL) return ESP_ERR_INVALID_STATE;
    return (xSemaphoreTake(h->lock, pdMS_TO_TICKS(1000)) == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
}

void tmc_motion_unlock_internal(tmc_motion_handle_t h)
{
    if (h && h->lock) xSemaphoreGive(h->lock);
}

esp_err_t tmc_motion_lock(tmc_motion_handle_t h, uint32_t timeout_ticks)
{
    if (h == NULL || h->lock == NULL) return ESP_ERR_INVALID_STATE;
    return (xSemaphoreTake(h->lock, timeout_ticks) == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
}

void tmc_motion_unlock(tmc_motion_handle_t h)
{
    if (h && h->lock) xSemaphoreGive(h->lock);
}

// ===========================================================================
// Last-error tracking (caller holds lock)
// ===========================================================================
void tmc_motion_set_last_error_locked(tmc_motion_handle_t h, esp_err_t err, const char *msg)
{
    h->last_error_code = err;
    snprintf(h->last_error, sizeof(h->last_error), "%s", msg ? msg : "");
}

void tmc_motion_clear_last_error_locked(tmc_motion_handle_t h)
{
    h->last_error_code = ESP_OK;
    h->last_error[0]   = '\0';
}

// ===========================================================================
// Diag refresh / decode
// ===========================================================================
static bool drv_status_flag(uint32_t drv_status, uint8_t bit)
{
    return (drv_status & (1UL << bit)) != 0U;
}

void tmc_motion_decode_diag(const tmc_motion_diag_t *diag, tmc_motion_decoded_diag_t *out)
{
    memset(out, 0, sizeof(*out));

    out->drv_enn          = ((diag->ioin >> 4) & 0x1U) != 0;
    out->sd_mode          = ((diag->ioin >> 6) & 0x1U) != 0;
    out->swcomp           = ((diag->ioin >> 7) & 0x1U) != 0;

    out->stst             = drv_status_flag(diag->drv_status, 31);
    out->ola              = drv_status_flag(diag->drv_status, 29);
    out->olb              = drv_status_flag(diag->drv_status, 30);
    out->s2ga             = drv_status_flag(diag->drv_status, 27);
    out->s2gb             = drv_status_flag(diag->drv_status, 28);
    out->s2vsa            = drv_status_flag(diag->drv_status, 12);
    out->s2vsb            = drv_status_flag(diag->drv_status, 13);
    out->otpw             = drv_status_flag(diag->drv_status, 26);
    out->ot               = drv_status_flag(diag->drv_status, 25);
    out->stall            = drv_status_flag(diag->drv_status, 24);
    out->fsactive         = drv_status_flag(diag->drv_status, 15);
    out->stealth          = drv_status_flag(diag->drv_status, 14);

    out->status_sg        = ((diag->ramp_stat >> 13) & 0x1U) != 0;
    out->event_stop_sg    = ((diag->ramp_stat >>  6) & 0x1U) != 0;
    out->position_reached = ((diag->ramp_stat >>  9) & 0x1U) != 0;
    out->velocity_reached = ((diag->ramp_stat >>  8) & 0x1U) != 0;

    out->cs_actual        = (diag->drv_status >> 16) & 0x1FU;
    out->sg_result        =  diag->drv_status        & 0x3FFU;
}

esp_err_t tmc_motion_refresh_diag_locked(tmc_motion_handle_t h)
{
    esp_err_t err = h->ops->collect_diag(h->chip_handle, &h->last_diag);
    if (err != ESP_OK) {
        tmc_motion_set_last_error_locked(h, err, "collect_diag failed");
        return err;
    }
    h->last_diag_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
    return ESP_OK;
}

esp_err_t tmc_motion_clear_ramp_status_events_locked(tmc_motion_handle_t h)
{
    uint32_t ramp_stat = 0;
    ESP_RETURN_ON_ERROR(h->ops->read_register(h->chip_handle, h->ops->reg_ramp_stat, &ramp_stat),
                        TAG, "read RAMP_STAT");
    return h->ops->write_register(h->chip_handle, h->ops->reg_ramp_stat, ramp_stat);
}

bool tmc_motion_busy_locked(tmc_motion_handle_t h)
{
    return h->sg_test.active || h->sg_cal.active;
}

// ===========================================================================
// Lifecycle
// ===========================================================================
esp_err_t tmc_motion_create(const tmc_motion_config_t *cfg, tmc_motion_handle_t *out)
{
    ESP_RETURN_ON_FALSE(cfg != NULL && out != NULL,            ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_FALSE(cfg->chip_handle && cfg->driver_ops,    ESP_ERR_INVALID_ARG, TAG, "chip/ops missing");
    ESP_RETURN_ON_FALSE(cfg->fullsteps_per_rev > 0,             ESP_ERR_INVALID_ARG, TAG, "fullsteps_per_rev=0");
    ESP_RETURN_ON_FALSE(cfg->microsteps_per_fullstep > 0,       ESP_ERR_INVALID_ARG, TAG, "microsteps_per_fullstep=0");
    ESP_RETURN_ON_FALSE(cfg->fclk_hz > 0,                       ESP_ERR_INVALID_ARG, TAG, "fclk_hz=0");

    tmc_motion_handle_t h = calloc(1, sizeof(*h));
    ESP_RETURN_ON_FALSE(h != NULL, ESP_ERR_NO_MEM, TAG, "alloc handle");

    h->lock = xSemaphoreCreateMutex();
    if (h->lock == NULL) {
        free(h);
        return ESP_ERR_NO_MEM;
    }

    h->chip_handle        = cfg->chip_handle;
    h->ops                = cfg->driver_ops;
    h->fullsteps_per_rev  = cfg->fullsteps_per_rev;
    h->microsteps_per_rev = cfg->fullsteps_per_rev * cfg->microsteps_per_fullstep;
    h->fclk_hz            = cfg->fclk_hz;
    h->motor_inverted     = cfg->initial_motor_inverted;
    h->motion_phys        = cfg->initial_motion;
    snprintf(h->name, sizeof(h->name), "%s", cfg->name ? cfg->name : "motor");

    if (cfg->initial_stallguard)  h->stallguard  = *cfg->initial_stallguard;
    if (cfg->initial_coolstep)    h->coolstep    = *cfg->initial_coolstep;
    if (cfg->initial_stealthchop) h->stealthchop = *cfg->initial_stealthchop;

    // Keep the chip's shaft bit neutral. Direction inversion is handled in
    // the motion layer so position, velocity, and UI units stay consistent.
    {
        uint32_t gconf = 0;
        if (h->ops->read_register(h->chip_handle, h->ops->reg_gconf, &gconf) == ESP_OK) {
            gconf &= ~(1UL << 4);
            (void)h->ops->write_register(h->chip_handle, h->ops->reg_gconf, gconf);
        }
    }

    // Apply initial profile.
    if (xSemaphoreTake(h->lock, portMAX_DELAY) == pdTRUE) {
        esp_err_t err = h->ops->set_rms_current_ma(h->chip_handle,
                                                   h->motion_phys.run_current_ma,
                                                   h->motion_phys.hold_percent);
        if (err == ESP_OK) err = tmc_motion_apply_profile_locked(h, 0);
        if (err == ESP_OK) err = tmc_motion_engage_position_hold_locked(h);

        // Apply initial features when configured.
        if (err == ESP_OK && cfg->initial_stallguard) {
            err = h->ops->configure_stallguard(h->chip_handle,
                                               h->stallguard.sgt,
                                               h->stallguard.sfilt,
                                               h->stallguard.tcoolthrs,
                                               h->stallguard.sg_stop);
        }
        if (err == ESP_OK && cfg->initial_coolstep) {
            err = h->ops->configure_coolstep(h->chip_handle,
                                             h->coolstep.semin, h->coolstep.semax,
                                             h->coolstep.seup,  h->coolstep.sedn,
                                             h->coolstep.seimin);
        }
        if (err == ESP_OK && cfg->initial_stealthchop) {
            err = h->ops->configure_stealthchop(h->chip_handle,
                                                h->stealthchop.enable,
                                                h->stealthchop.tpwmthrs);
        }

        if (err == ESP_OK) tmc_motion_clear_last_error_locked(h);
        else               tmc_motion_set_last_error_locked  (h, err, "init failed");
        (void)tmc_motion_refresh_diag_locked(h);
        xSemaphoreGive(h->lock);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "[%s] init failed: %s", h->name, esp_err_to_name(err));
            tmc_motion_destroy(h);
            return err;
        }
    }

    *out = h;
    ESP_LOGI(TAG, "[%s] tmc_motion ready (%s, %u µsteps/rev, %u Hz)",
             h->name, h->ops->name,
             (unsigned)h->microsteps_per_rev, (unsigned)h->fclk_hz);
    return ESP_OK;
}

void tmc_motion_destroy(tmc_motion_handle_t h)
{
    if (h == NULL) return;

    // Cancel running tasks.
    if (h->sg_test_task || h->sg_cal_task) {
        if (xSemaphoreTake(h->lock, pdMS_TO_TICKS(500)) == pdTRUE) {
            h->sg_test.cancel_requested = true;
            h->sg_cal.cancel_requested  = true;
            xSemaphoreGive(h->lock);
        }
        // Tasks self-delete; give them a moment.
        for (int i = 0; i < 50 && (h->sg_test_task || h->sg_cal_task); ++i) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    if (h->lock) vSemaphoreDelete(h->lock);
    free(h);
}

// ===========================================================================
// Snapshot
// ===========================================================================
esp_err_t tmc_motion_get_snapshot(tmc_motion_handle_t h, tmc_motion_snapshot_t *out)
{
    ESP_RETURN_ON_FALSE(h && out, ESP_ERR_INVALID_ARG, TAG, "null arg");

    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");
    esp_err_t err = tmc_motion_refresh_diag_locked(h);
    if (err == ESP_OK) {
        memset(out, 0, sizeof(*out));
        out->initialized        = true;
        out->name               = h->name;
        out->driver_name        = h->ops->name;
        out->last_error_code    = h->last_error_code;
        snprintf(out->last_error, sizeof(out->last_error), "%s", h->last_error);
        out->microsteps_per_rev = h->microsteps_per_rev;
        out->fclk_hz            = h->fclk_hz;
        out->motor_inverted     = h->motor_inverted;
        out->last_diag_ms       = h->last_diag_ms;
        out->diag               = h->last_diag;
        tmc_motion_decode_diag(&h->last_diag, &out->decoded);
        out->motion.commanded_velocity = h->commanded_velocity;
        out->motion.xtarget            = h->xtarget;
        out->motion.profile            = h->motion_phys;
        out->stallguard         = h->stallguard;
        out->coolstep           = h->coolstep;
        out->stealthchop        = h->stealthchop;
        out->sg_test            = h->sg_test;
        out->sg_calibration     = h->sg_cal;
    }
    tmc_motion_unlock_internal(h);
    return err;
}

// ===========================================================================
// Raw register access (locked)
// ===========================================================================
esp_err_t tmc_motion_write_register(tmc_motion_handle_t h, uint8_t reg, uint32_t value)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");
    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");
    esp_err_t err = h->ops->write_register(h->chip_handle, reg, value);
    tmc_motion_unlock_internal(h);
    return err;
}

esp_err_t tmc_motion_read_register(tmc_motion_handle_t h, uint8_t reg, uint32_t *out)
{
    ESP_RETURN_ON_FALSE(h && out, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");
    esp_err_t err = h->ops->read_register(h->chip_handle, reg, out);
    tmc_motion_unlock_internal(h);
    return err;
}

// ===========================================================================
// Accessors
// ===========================================================================
const tmc_motion_driver_ops_t *tmc_motion_get_ops(tmc_motion_handle_t h) {
    return h ? h->ops : NULL;
}
void *tmc_motion_get_chip_handle(tmc_motion_handle_t h) {
    return h ? h->chip_handle : NULL;
}
uint32_t tmc_motion_get_microsteps_per_rev(tmc_motion_handle_t h) {
    return h ? h->microsteps_per_rev : 0;
}
uint32_t tmc_motion_get_fclk_hz(tmc_motion_handle_t h) {
    return h ? h->fclk_hz : 0;
}
const char *tmc_motion_get_name(tmc_motion_handle_t h) {
    return h ? h->name : "";
}

// ===========================================================================
// Handle-bound unit conversions
// ===========================================================================
uint32_t tmc_motion_rpm_to_vmax(tmc_motion_handle_t h, double rpm) {
    if (!h) return 0;
    return tmc_motion_units_rpm_to_vmax(rpm, h->microsteps_per_rev, h->fclk_hz);
}
double tmc_motion_vmax_to_rpm(tmc_motion_handle_t h, uint32_t vmax) {
    if (!h) return 0.0;
    return tmc_motion_units_vmax_to_rpm(vmax, h->microsteps_per_rev, h->fclk_hz);
}
uint32_t tmc_motion_rps2_to_amax(tmc_motion_handle_t h, double rps2) {
    if (!h) return 0;
    return tmc_motion_units_rps2_to_amax(rps2, h->microsteps_per_rev, h->fclk_hz);
}
double tmc_motion_amax_to_rps2(tmc_motion_handle_t h, uint32_t amax) {
    if (!h) return 0.0;
    return tmc_motion_units_amax_to_rps2(amax, h->microsteps_per_rev, h->fclk_hz);
}
int32_t tmc_motion_degrees_to_steps(tmc_motion_handle_t h, double deg) {
    if (!h) return 0;
    if (h && h->motor_inverted) deg = -deg;
    return tmc_motion_units_degrees_to_steps(deg, h->microsteps_per_rev);
}
double tmc_motion_steps_to_degrees(tmc_motion_handle_t h, int32_t steps) {
    if (!h) return 0.0;
    double deg = tmc_motion_units_steps_to_degrees(steps, h->microsteps_per_rev);
    return (h && h->motor_inverted) ? -deg : deg;
}
