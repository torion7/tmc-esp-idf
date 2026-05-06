// TMC5160 → tmc_motion driver-ops adapter. Mirror of adapter_5240.c.

#include "tmc_motion_5160.h"

#include "tmc5160.h"
#include "tmc5160_regs.h"

#include <string.h>

static esp_err_t a_write_register(void *c, uint8_t reg, uint32_t v) {
    return tmc5160_write_register((tmc5160_handle_t)c, reg, v);
}
static esp_err_t a_read_register(void *c, uint8_t reg, uint32_t *out) {
    return tmc5160_read_register((tmc5160_handle_t)c, reg, out);
}
static esp_err_t a_set_enabled(void *c, bool enabled) {
    return tmc5160_set_enabled((tmc5160_handle_t)c, enabled);
}
static esp_err_t a_set_rms_current_ma(void *c, uint16_t run_ma, uint8_t hold_pct) {
    return tmc5160_set_rms_current_ma((tmc5160_handle_t)c, run_ma, hold_pct);
}
static esp_err_t a_set_amax(void *c, uint32_t amax) {
    return tmc5160_set_amax((tmc5160_handle_t)c, amax);
}
static esp_err_t a_set_velocity_mode(void *c, int32_t vmax) {
    return tmc5160_set_velocity_mode((tmc5160_handle_t)c, vmax);
}
static esp_err_t a_set_position_mode(void *c, int32_t xtarget) {
    return tmc5160_set_position_mode((tmc5160_handle_t)c, xtarget);
}
static esp_err_t a_stop(void *c) {
    return tmc5160_stop((tmc5160_handle_t)c);
}
static esp_err_t a_configure_stallguard(void *c, int8_t sgt, bool sfilt,
                                        uint32_t tcoolthrs, bool sg_stop) {
    return tmc5160_configure_stallguard((tmc5160_handle_t)c, sgt, sfilt, tcoolthrs, sg_stop);
}
static esp_err_t a_configure_coolstep(void *c, uint8_t semin, uint8_t semax,
                                      uint8_t seup, uint8_t sedn, bool seimin) {
    return tmc5160_configure_coolstep((tmc5160_handle_t)c, semin, semax, seup, sedn, seimin);
}
static esp_err_t a_configure_stealthchop(void *c, bool enable, uint32_t tpwmthrs) {
    return tmc5160_configure_stealthchop((tmc5160_handle_t)c, enable, tpwmthrs);
}
static esp_err_t a_collect_diag(void *c, tmc_motion_diag_t *out) {
    tmc5160_diag_t d;
    esp_err_t err = tmc5160_collect_diag((tmc5160_handle_t)c, &d);
    if (err != ESP_OK) return err;
    memset(out, 0, sizeof(*out));
    out->ioin           = d.ioin;
    out->gstat          = d.gstat;
    out->gconf          = d.gconf;
    out->chopconf       = d.chopconf;
    out->coolconf       = d.coolconf;
    out->sw_mode        = d.sw_mode;
    out->ramp_stat      = d.ramp_stat;
    out->ihold_irun     = d.ihold_irun;
    out->tpowerdown     = d.tpowerdown;
    out->global_scaler  = d.global_scaler;
    out->tcoolthrs      = d.tcoolthrs;
    out->tpwmthrs       = d.tpwmthrs;
    out->drv_status     = d.drv_status;
    out->tstep          = d.tstep;
    out->xactual        = d.xactual;
    out->vactual        = d.vactual;
    return ESP_OK;
}

const tmc_motion_driver_ops_t tmc_motion_ops_5160 = {
    .name                  = "TMC5160",
    .write_register        = a_write_register,
    .read_register         = a_read_register,
    .set_enabled           = a_set_enabled,
    .set_rms_current_ma    = a_set_rms_current_ma,
    .set_amax              = a_set_amax,
    .set_velocity_mode     = a_set_velocity_mode,
    .set_position_mode     = a_set_position_mode,
    .stop                  = a_stop,
    .configure_stallguard  = a_configure_stallguard,
    .configure_coolstep    = a_configure_coolstep,
    .configure_stealthchop = a_configure_stealthchop,
    .collect_diag          = a_collect_diag,

    .reg_xactual           = TMC5160_REG_XACTUAL,
    .reg_xtarget           = TMC5160_REG_XTARGET,
    .reg_vmax              = TMC5160_REG_VMAX,
    .reg_amax              = TMC5160_REG_AMAX,
    .reg_a1                = TMC5160_REG_A1,
    .reg_v1                = TMC5160_REG_V1,
    .reg_dmax              = TMC5160_REG_DMAX,
    .reg_d1                = TMC5160_REG_D1,
    .reg_vstart            = TMC5160_REG_VSTART,
    .reg_vstop             = TMC5160_REG_VSTOP,
    .reg_tzerowait         = TMC5160_REG_TZEROWAIT,
    .reg_rampmode          = TMC5160_REG_RAMPMODE,
    .reg_ramp_stat         = TMC5160_REG_RAMP_STAT,
    .reg_gconf             = TMC5160_REG_GCONF,
    .reg_chopconf          = TMC5160_REG_CHOPCONF,
};
