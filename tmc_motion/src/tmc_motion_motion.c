// Motion-profile / velocity / position commands.

#include "tmc_motion.h"
#include "tmc_motion_internal.h"

#include <stdint.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "tmc_motion_motion";

// ===========================================================================
// Stallguard arming timing constants (used by velocity bring-up below)
// ===========================================================================
#define STALLGUARD_SG_STOP_ARM_DELAY_MS 300U
#define STALLGUARD_ARM_POLL_MS           50U
#define STALLGUARD_ARM_TIMEOUT_MS      2000U
#define STALLGUARD_ARM_STABLE_MS        200U
#define STALLGUARD_ARM_VACTUAL_PERCENT   80U

// ===========================================================================
// Profile sanitization
// ===========================================================================
static double clamp_d(double v, double lo, double hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static tmc_motion_config_phys_t sanitize_profile(const tmc_motion_config_phys_t *in)
{
    tmc_motion_config_phys_t s = *in;
    if (s.default_rpm <= 0.0) s.default_rpm = 15.0;
    if (s.hold_rpm    <= 0.0) s.hold_rpm    = s.default_rpm;
    if (s.amax_rps2   <= 0.0) s.amax_rps2   = 3.2;
    if (s.dmax_rps2   <= 0.0) s.dmax_rps2   = s.amax_rps2;
    if (s.d1_rps2     <= 0.0) s.d1_rps2     = s.dmax_rps2;
    if (s.vstop_rpm   <= 0.0) s.vstop_rpm   = 0.012;
    s.hold_percent    = (uint8_t)clamp_d(s.hold_percent, 0.0, 100.0);
    return s;
}

static bool microsteps_to_mres(uint16_t microsteps, uint8_t *out_mres)
{
    switch (microsteps) {
    case 256: *out_mres = 0; return true;
    case 128: *out_mres = 1; return true;
    case  64: *out_mres = 2; return true;
    case  32: *out_mres = 3; return true;
    case  16: *out_mres = 4; return true;
    case   8: *out_mres = 5; return true;
    case   4: *out_mres = 6; return true;
    case   2: *out_mres = 7; return true;
    case   1: *out_mres = 8; return true;
    default: return false;
    }
}

// ===========================================================================
// Apply current motion_phys profile to chip registers (caller holds lock)
// ===========================================================================
esp_err_t tmc_motion_apply_profile_locked(tmc_motion_handle_t h, uint32_t vmax_override)
{
    const tmc_motion_config_phys_t *p = &h->motion_phys;
    const tmc_motion_driver_ops_t  *o = h->ops;
    void                           *c = h->chip_handle;

    uint32_t vmax  = (vmax_override != 0) ? vmax_override
                                          : tmc_motion_units_rpm_to_vmax(p->default_rpm,
                                                                         h->microsteps_per_rev,
                                                                         h->fclk_hz);
    uint32_t limit = (p->velocity_limit_rpm > 0.0)
                     ? tmc_motion_units_rpm_to_vmax(p->velocity_limit_rpm,
                                                    h->microsteps_per_rev, h->fclk_hz)
                     : TMC_MOTION_VMAX_MAX;
    if (vmax > limit) vmax = limit;
    if (vmax == 0)    vmax = 1;

    uint32_t vstart = tmc_motion_units_rpm_to_vmax (p->vstart_rpm, h->microsteps_per_rev, h->fclk_hz);
    uint32_t a1     = tmc_motion_units_rps2_to_amax(p->a1_rps2,    h->microsteps_per_rev, h->fclk_hz);
    uint32_t v1     = tmc_motion_units_rpm_to_vmax (p->v1_rpm,     h->microsteps_per_rev, h->fclk_hz);
    uint32_t amax   = tmc_motion_units_rps2_to_amax(p->amax_rps2,  h->microsteps_per_rev, h->fclk_hz);
    uint32_t dmax   = tmc_motion_units_rps2_to_amax(p->dmax_rps2,  h->microsteps_per_rev, h->fclk_hz);
    uint32_t d1     = tmc_motion_units_rps2_to_amax(p->d1_rps2,    h->microsteps_per_rev, h->fclk_hz);
    uint32_t vstop  = tmc_motion_units_rpm_to_vmax (p->vstop_rpm,  h->microsteps_per_rev, h->fclk_hz);
    if (vstop == 0) vstop = 1;  // chip rejects vstop=0

    esp_err_t err = o->write_register(c, o->reg_vstart,  vstart);
    if (err == ESP_OK) err = o->write_register(c, o->reg_a1,    a1);
    if (err == ESP_OK) err = o->write_register(c, o->reg_v1,    v1);
    if (err == ESP_OK) err = o->set_amax     (c,                amax);
    if (err == ESP_OK) err = o->write_register(c, o->reg_vmax,  vmax);
    if (err == ESP_OK) err = o->write_register(c, o->reg_dmax,  dmax);
    if (err == ESP_OK) err = o->write_register(c, o->reg_d1,    d1);
    if (err == ESP_OK) err = o->write_register(c, o->reg_vstop, vstop);
    if (err == ESP_OK) err = o->write_register(c, o->reg_tzerowait, p->tzerowait);
    return err;
}

// ===========================================================================
// Position-hold helper
// ===========================================================================
esp_err_t tmc_motion_engage_position_hold_locked(tmc_motion_handle_t h)
{
    uint32_t raw_xactual = 0;
    esp_err_t err = h->ops->read_register(h->chip_handle, h->ops->reg_xactual, &raw_xactual);
    if (err != ESP_OK) return err;
    int32_t xactual = (int32_t)raw_xactual;

    uint32_t hold_vmax = tmc_motion_units_rpm_to_vmax(h->motion_phys.hold_rpm,
                                                      h->microsteps_per_rev, h->fclk_hz);
    if (hold_vmax == 0) hold_vmax = 1;
    err = tmc_motion_apply_profile_locked(h, hold_vmax);
    if (err != ESP_OK) return err;
    err = h->ops->write_register(h->chip_handle, h->ops->reg_rampmode, 0);
    if (err != ESP_OK) return err;
    err = h->ops->write_register(h->chip_handle, h->ops->reg_xtarget, (uint32_t)xactual);
    if (err != ESP_OK) return err;

    h->xtarget            = xactual;
    h->commanded_velocity = 0;
    return ESP_OK;
}

// ===========================================================================
// Velocity bring-up with optional sg_stop arming
// ===========================================================================
esp_err_t tmc_motion_start_velocity_with_stallguard_arm_locked(tmc_motion_handle_t h,
                                                               int32_t velocity,
                                                               bool    sg_stop_requested)
{
    esp_err_t err = h->ops->configure_stallguard(h->chip_handle,
                                                 h->stallguard.sgt,
                                                 h->stallguard.sfilt,
                                                 h->stallguard.tcoolthrs,
                                                 false);
    if (err != ESP_OK) return err;

    err = tmc_motion_clear_ramp_status_events_locked(h);
    if (err != ESP_OK) return err;

    uint32_t target_abs = (velocity < 0) ? (uint32_t)(-(int64_t)velocity) : (uint32_t)velocity;
    uint32_t limit      = (h->motion_phys.velocity_limit_rpm > 0.0)
                          ? tmc_motion_units_rpm_to_vmax(h->motion_phys.velocity_limit_rpm,
                                                         h->microsteps_per_rev, h->fclk_hz)
                          : TMC_MOTION_VMAX_MAX;
    if (target_abs > limit) {
        target_abs = limit;
        velocity   = (velocity < 0) ? -(int32_t)target_abs : (int32_t)target_abs;
    }
    err = tmc_motion_apply_profile_locked(h, target_abs);
    if (err != ESP_OK) return err;

    err = h->ops->set_velocity_mode(h->chip_handle, velocity);
    if (err != ESP_OK) return err;

    if (!sg_stop_requested || velocity == 0) return ESP_OK;

    // Wait for VACTUAL to ramp up before arming sg_stop.
    uint32_t stable_ms   = 0;
    uint64_t deadline_ms = (uint64_t)(esp_timer_get_time() / 1000ULL) + STALLGUARD_ARM_TIMEOUT_MS;

    while ((uint64_t)(esp_timer_get_time() / 1000ULL) < deadline_ms) {
        vTaskDelay(pdMS_TO_TICKS(STALLGUARD_ARM_POLL_MS));

        err = tmc_motion_refresh_diag_locked(h);
        if (err != ESP_OK) return err;

        uint32_t actual_abs    = (h->last_diag.vactual < 0)
                                 ? (uint32_t)(-h->last_diag.vactual)
                                 : (uint32_t)h->last_diag.vactual;
        uint32_t arm_threshold = (target_abs * STALLGUARD_ARM_VACTUAL_PERCENT) / 100U;

        if (actual_abs >= arm_threshold) {
            stable_ms += STALLGUARD_ARM_POLL_MS;
            if (stable_ms >= STALLGUARD_ARM_STABLE_MS) break;
        } else {
            stable_ms = 0;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(STALLGUARD_SG_STOP_ARM_DELAY_MS));
    err = tmc_motion_clear_ramp_status_events_locked(h);
    if (err != ESP_OK) return err;

    return h->ops->configure_stallguard(h->chip_handle,
                                        h->stallguard.sgt,
                                        h->stallguard.sfilt,
                                        h->stallguard.tcoolthrs,
                                        true);
}

// ===========================================================================
// Public motion commands
// ===========================================================================
esp_err_t tmc_motion_set_velocity_rpm(tmc_motion_handle_t h, double rpm)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");
    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");
    if (tmc_motion_busy_locked(h)) {
        tmc_motion_unlock_internal(h);
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t vmax     = tmc_motion_rpm_to_vmax(h, rpm);
    int32_t  signed_v = (rpm < 0.0) ? -(int32_t)vmax : (int32_t)vmax;
    if (h->motor_inverted) signed_v = -signed_v;

    esp_err_t err = tmc_motion_start_velocity_with_stallguard_arm_locked(h, signed_v,
                                                                         h->stallguard.sg_stop);
    if (err == ESP_OK) {
        h->commanded_velocity = signed_v;
        tmc_motion_clear_last_error_locked(h);
        (void)tmc_motion_refresh_diag_locked(h);
    } else {
        tmc_motion_set_last_error_locked(h, err, "set velocity failed");
    }
    tmc_motion_unlock_internal(h);
    return err;
}

esp_err_t tmc_motion_set_position_degrees(tmc_motion_handle_t h, double degrees, double max_rpm)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");
    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");
    if (tmc_motion_busy_locked(h)) {
        tmc_motion_unlock_internal(h);
        return ESP_ERR_INVALID_STATE;
    }

    int32_t target_steps = tmc_motion_degrees_to_steps(h, degrees);
    uint32_t vmax        = (max_rpm > 0.0) ? tmc_motion_rpm_to_vmax(h, max_rpm) : 0;

    // Disable sg_stop in position mode to avoid spurious stops during ramp.
    esp_err_t err = h->ops->configure_stallguard(h->chip_handle,
                                                 h->stallguard.sgt,
                                                 h->stallguard.sfilt,
                                                 h->stallguard.tcoolthrs,
                                                 false);
    if (err == ESP_OK) err = tmc_motion_clear_ramp_status_events_locked(h);
    if (err == ESP_OK) err = tmc_motion_apply_profile_locked(h, vmax);
    if (err == ESP_OK) err = h->ops->write_register(h->chip_handle, h->ops->reg_rampmode, 0);
    if (err == ESP_OK) err = h->ops->write_register(h->chip_handle, h->ops->reg_xtarget, (uint32_t)target_steps);

    if (err == ESP_OK) {
        h->commanded_velocity = 0;
        h->xtarget            = target_steps;
        tmc_motion_clear_last_error_locked(h);
        (void)tmc_motion_refresh_diag_locked(h);
    } else {
        tmc_motion_set_last_error_locked(h, err, "set position failed");
    }
    tmc_motion_unlock_internal(h);
    return err;
}

esp_err_t tmc_motion_stop(tmc_motion_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");
    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");

    h->sg_test.cancel_requested = true;
    h->sg_cal.cancel_requested  = true;

    esp_err_t err = h->ops->stop(h->chip_handle);
    if (err == ESP_OK) err = tmc_motion_engage_position_hold_locked(h);
    if (err == ESP_OK) {
        h->commanded_velocity = 0;
        tmc_motion_clear_last_error_locked(h);
        (void)tmc_motion_refresh_diag_locked(h);
    } else {
        tmc_motion_set_last_error_locked(h, err, "stop failed");
    }
    tmc_motion_unlock_internal(h);
    return err;
}

esp_err_t tmc_motion_set_position_zero(tmc_motion_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");
    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");
    if (tmc_motion_busy_locked(h)) {
        tmc_motion_unlock_internal(h);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = h->ops->write_register(h->chip_handle, h->ops->reg_xactual, 0);
    if (err == ESP_OK) err = tmc_motion_engage_position_hold_locked(h);
    if (err == ESP_OK) {
        tmc_motion_clear_last_error_locked(h);
        (void)tmc_motion_refresh_diag_locked(h);
    } else {
        tmc_motion_set_last_error_locked(h, err, "zero failed");
    }
    tmc_motion_unlock_internal(h);
    return err;
}

esp_err_t tmc_motion_set_current(tmc_motion_handle_t h, uint16_t run_ma, uint8_t hold_pct)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");
    ESP_RETURN_ON_FALSE(hold_pct <= 100, ESP_ERR_INVALID_ARG, TAG, "hold>100");
    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");
    if (tmc_motion_busy_locked(h)) {
        tmc_motion_unlock_internal(h);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = h->ops->set_rms_current_ma(h->chip_handle, run_ma, hold_pct);
    if (err == ESP_OK) {
        h->motion_phys.run_current_ma = run_ma;
        h->motion_phys.hold_percent   = hold_pct;
        tmc_motion_clear_last_error_locked(h);
        (void)tmc_motion_refresh_diag_locked(h);
    } else {
        tmc_motion_set_last_error_locked(h, err, "set current failed");
    }
    tmc_motion_unlock_internal(h);
    return err;
}

esp_err_t tmc_motion_set_motor_inverted(tmc_motion_handle_t h, bool inverted)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");
    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");
    if (tmc_motion_busy_locked(h)) {
        tmc_motion_unlock_internal(h);
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t gconf = 0;
    esp_err_t err = h->ops->read_register(h->chip_handle, h->ops->reg_gconf, &gconf);
    if (err == ESP_OK) {
        gconf &= ~(1UL << 4);
        err = h->ops->write_register(h->chip_handle, h->ops->reg_gconf, gconf);
    }
    if (err == ESP_OK) {
        uint32_t raw_xactual = 0;
        err = h->ops->read_register(h->chip_handle, h->ops->reg_xactual, &raw_xactual);
        if (err == ESP_OK) {
            int32_t old_xactual = (int32_t)raw_xactual;
            int32_t old_xtarget = h->xtarget;
            bool old_inverted = h->motor_inverted;

            double logical_xactual = tmc_motion_units_steps_to_degrees(old_xactual, h->microsteps_per_rev);
            double logical_xtarget = tmc_motion_units_steps_to_degrees(old_xtarget, h->microsteps_per_rev);
            if (old_inverted) {
                logical_xactual = -logical_xactual;
                logical_xtarget = -logical_xtarget;
            }

            h->motor_inverted = inverted;
            int32_t new_xactual = tmc_motion_units_degrees_to_steps(inverted ? -logical_xactual : logical_xactual,
                                                                    h->microsteps_per_rev);
            int32_t new_xtarget = tmc_motion_units_degrees_to_steps(inverted ? -logical_xtarget : logical_xtarget,
                                                                    h->microsteps_per_rev);
            err = h->ops->write_register(h->chip_handle, h->ops->reg_xactual, (uint32_t)new_xactual);
            if (err == ESP_OK) err = h->ops->write_register(h->chip_handle, h->ops->reg_xtarget, (uint32_t)new_xtarget);
            if (err == ESP_OK) {
                h->xtarget = new_xtarget;
                err = tmc_motion_engage_position_hold_locked(h);
            }
            if (err != ESP_OK) {
                h->motor_inverted = old_inverted;
            }
        }
    }
    if (err == ESP_OK) {
        tmc_motion_clear_last_error_locked(h);
        (void)tmc_motion_refresh_diag_locked(h);
    } else {
        tmc_motion_set_last_error_locked(h, err, "set direction failed");
    }
    tmc_motion_unlock_internal(h);
    return err;
}

esp_err_t tmc_motion_set_microsteps_per_fullstep(tmc_motion_handle_t h, uint16_t microsteps)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");

    uint8_t mres = 0;
    ESP_RETURN_ON_FALSE(microsteps_to_mres(microsteps, &mres), ESP_ERR_INVALID_ARG, TAG, "invalid microsteps");

    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");
    if (tmc_motion_busy_locked(h)) {
        tmc_motion_unlock_internal(h);
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t old_microsteps_per_rev = h->microsteps_per_rev;
    uint32_t new_microsteps_per_rev = h->fullsteps_per_rev * (uint32_t)microsteps;
    if (old_microsteps_per_rev == new_microsteps_per_rev) {
        tmc_motion_unlock_internal(h);
        return ESP_OK;
    }

    uint32_t raw_xactual = 0;
    esp_err_t err = h->ops->read_register(h->chip_handle, h->ops->reg_xactual, &raw_xactual);
    int32_t old_xactual = (int32_t)raw_xactual;
    int32_t old_xtarget = h->xtarget;

    uint32_t chopconf = 0;
    if (err == ESP_OK) err = h->ops->read_register(h->chip_handle, h->ops->reg_chopconf, &chopconf);
    if (err == ESP_OK) {
        chopconf &= ~(0xFUL << 24);
        chopconf |= ((uint32_t)mres & 0xFU) << 24;
        err = h->ops->write_register(h->chip_handle, h->ops->reg_chopconf, chopconf);
    }

    if (err == ESP_OK) {
        int64_t scaled_xactual = ((int64_t)old_xactual * (int64_t)new_microsteps_per_rev) /
                                 (int64_t)old_microsteps_per_rev;
        int64_t scaled_xtarget = ((int64_t)old_xtarget * (int64_t)new_microsteps_per_rev) /
                                 (int64_t)old_microsteps_per_rev;
        h->microsteps_per_rev = new_microsteps_per_rev;
        err = h->ops->write_register(h->chip_handle, h->ops->reg_xactual, (uint32_t)(int32_t)scaled_xactual);
        if (err == ESP_OK) err = h->ops->write_register(h->chip_handle, h->ops->reg_xtarget, (uint32_t)(int32_t)scaled_xtarget);
        if (err == ESP_OK) {
            h->xtarget = (int32_t)scaled_xtarget;
            err = tmc_motion_engage_position_hold_locked(h);
        }
        if (err != ESP_OK) {
            h->microsteps_per_rev = old_microsteps_per_rev;
        }
    }

    if (err == ESP_OK) {
        tmc_motion_clear_last_error_locked(h);
        (void)tmc_motion_refresh_diag_locked(h);
    } else {
        tmc_motion_set_last_error_locked(h, err, "set microsteps failed");
    }
    tmc_motion_unlock_internal(h);
    return err;
}

esp_err_t tmc_motion_set_fullsteps_per_rev(tmc_motion_handle_t h, uint16_t fullsteps_per_rev)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");
    ESP_RETURN_ON_FALSE(fullsteps_per_rev > 0, ESP_ERR_INVALID_ARG, TAG, "invalid fullsteps");

    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");
    if (tmc_motion_busy_locked(h)) {
        tmc_motion_unlock_internal(h);
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t old_microsteps_per_rev = h->microsteps_per_rev;
    uint32_t old_fullsteps_per_rev = h->fullsteps_per_rev;
    uint32_t microsteps_per_fullstep = old_microsteps_per_rev / old_fullsteps_per_rev;
    uint32_t new_microsteps_per_rev = (uint32_t)fullsteps_per_rev * microsteps_per_fullstep;
    if (new_microsteps_per_rev == 0) {
        tmc_motion_unlock_internal(h);
        return ESP_ERR_INVALID_ARG;
    }
    if (old_microsteps_per_rev == new_microsteps_per_rev) {
        h->fullsteps_per_rev = fullsteps_per_rev;
        tmc_motion_unlock_internal(h);
        return ESP_OK;
    }

    uint32_t raw_xactual = 0;
    esp_err_t err = h->ops->read_register(h->chip_handle, h->ops->reg_xactual, &raw_xactual);
    int32_t old_xactual = (int32_t)raw_xactual;
    int32_t old_xtarget = h->xtarget;

    if (err == ESP_OK) {
        int64_t scaled_xactual = ((int64_t)old_xactual * (int64_t)new_microsteps_per_rev) /
                                 (int64_t)old_microsteps_per_rev;
        int64_t scaled_xtarget = ((int64_t)old_xtarget * (int64_t)new_microsteps_per_rev) /
                                 (int64_t)old_microsteps_per_rev;
        h->fullsteps_per_rev = fullsteps_per_rev;
        h->microsteps_per_rev = new_microsteps_per_rev;
        err = h->ops->write_register(h->chip_handle, h->ops->reg_xactual, (uint32_t)(int32_t)scaled_xactual);
        if (err == ESP_OK) err = h->ops->write_register(h->chip_handle, h->ops->reg_xtarget, (uint32_t)(int32_t)scaled_xtarget);
        if (err == ESP_OK) {
            h->xtarget = (int32_t)scaled_xtarget;
            err = tmc_motion_engage_position_hold_locked(h);
        }
        if (err != ESP_OK) {
            h->fullsteps_per_rev = old_fullsteps_per_rev;
            h->microsteps_per_rev = old_microsteps_per_rev;
        }
    }

    if (err == ESP_OK) {
        tmc_motion_clear_last_error_locked(h);
        (void)tmc_motion_refresh_diag_locked(h);
    } else {
        tmc_motion_set_last_error_locked(h, err, "set fullsteps failed");
    }
    tmc_motion_unlock_internal(h);
    return err;
}

esp_err_t tmc_motion_configure_motion_phys(tmc_motion_handle_t h, const tmc_motion_config_phys_t *cfg)
{
    ESP_RETURN_ON_FALSE(h && cfg, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");
    if (tmc_motion_busy_locked(h)) {
        tmc_motion_unlock_internal(h);
        return ESP_ERR_INVALID_STATE;
    }

    tmc_motion_config_phys_t sanitized = sanitize_profile(cfg);

    esp_err_t err = h->ops->set_rms_current_ma(h->chip_handle,
                                               sanitized.run_current_ma,
                                               sanitized.hold_percent);
    tmc_motion_config_phys_t previous = h->motion_phys;
    if (err == ESP_OK) {
        h->motion_phys = sanitized;
        err = tmc_motion_apply_profile_locked(h, 0);
    }
    if (err == ESP_OK) {
        tmc_motion_clear_last_error_locked(h);
        (void)tmc_motion_refresh_diag_locked(h);
    } else {
        h->motion_phys = previous;
        tmc_motion_set_last_error_locked(h, err, "configure motion failed");
    }
    tmc_motion_unlock_internal(h);
    return err;
}

esp_err_t tmc_motion_configure_stallguard(tmc_motion_handle_t h, const tmc_motion_stallguard_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(h && cfg, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");
    if (tmc_motion_busy_locked(h)) {
        tmc_motion_unlock_internal(h);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = h->ops->configure_stallguard(h->chip_handle,
                                                 cfg->sgt, cfg->sfilt,
                                                 cfg->tcoolthrs, cfg->sg_stop);
    if (err == ESP_OK) {
        h->stallguard = *cfg;
        tmc_motion_clear_last_error_locked(h);
        (void)tmc_motion_refresh_diag_locked(h);
    } else {
        tmc_motion_set_last_error_locked(h, err, "configure stallguard failed");
    }
    tmc_motion_unlock_internal(h);
    return err;
}

esp_err_t tmc_motion_configure_coolstep(tmc_motion_handle_t h, const tmc_motion_coolstep_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(h && cfg, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");
    if (tmc_motion_busy_locked(h)) {
        tmc_motion_unlock_internal(h);
        return ESP_ERR_INVALID_STATE;
    }

    // CoolStep needs SpreadCycle. If user enables CoolStep while StealthChop
    // is on, drop StealthChop first.
    if (cfg->semin > 0 && h->stealthchop.enable) {
        esp_err_t e = h->ops->configure_stealthchop(h->chip_handle, false, h->stealthchop.tpwmthrs);
        if (e != ESP_OK) {
            tmc_motion_set_last_error_locked(h, e, "disable stealthchop before coolstep");
            tmc_motion_unlock_internal(h);
            return e;
        }
        h->stealthchop.enable = false;
    }

    esp_err_t err = h->ops->configure_coolstep(h->chip_handle,
                                               cfg->semin, cfg->semax,
                                               cfg->seup, cfg->sedn, cfg->seimin);
    if (err == ESP_OK) {
        h->coolstep = *cfg;
        tmc_motion_clear_last_error_locked(h);
        (void)tmc_motion_refresh_diag_locked(h);
    } else {
        tmc_motion_set_last_error_locked(h, err, "configure coolstep failed");
    }
    tmc_motion_unlock_internal(h);
    return err;
}

esp_err_t tmc_motion_configure_stealthchop(tmc_motion_handle_t h, const tmc_motion_stealthchop_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(h && cfg, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");
    if (tmc_motion_busy_locked(h)) {
        tmc_motion_unlock_internal(h);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = h->ops->configure_stealthchop(h->chip_handle, cfg->enable, cfg->tpwmthrs);
    if (err == ESP_OK) {
        h->stealthchop = *cfg;
        tmc_motion_clear_last_error_locked(h);
        (void)tmc_motion_refresh_diag_locked(h);
    } else {
        tmc_motion_set_last_error_locked(h, err, "configure stealthchop failed");
    }
    tmc_motion_unlock_internal(h);
    return err;
}
