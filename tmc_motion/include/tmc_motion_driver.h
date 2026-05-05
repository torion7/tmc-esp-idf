#pragma once

// Driver-Ops vtable: chip-agnostic interface that tmc_motion calls into.
// Each supported TMC variant provides a `tmc_motion_driver_ops_t` instance
// in its adapter source file (see adapters/adapter_5240.c, adapter_5160.c).
//
// The `chip` argument is the chip-specific handle (tmc5240_handle_t /
// tmc5160_handle_t), opaqued through `void *`. The adapter casts back.

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Unified diagnostic snapshot. Adapters fill the fields their chip reports
// and leave others at 0. Field names match the TMC5240/5160 register names.
// ---------------------------------------------------------------------------
typedef struct {
    uint32_t ioin;
    uint32_t gstat;
    uint32_t gconf;
    uint32_t chopconf;
    uint32_t coolconf;
    uint32_t sw_mode;
    uint32_t ramp_stat;
    uint32_t ihold_irun;
    uint32_t tpowerdown;
    uint32_t global_scaler;
    uint32_t tcoolthrs;
    uint32_t tpwmthrs;
    uint32_t drv_status;
    uint32_t tstep;
    int32_t  xactual;
    int32_t  vactual;
} tmc_motion_diag_t;

// ---------------------------------------------------------------------------
// Driver ops — what tmc_motion needs to talk to a chip.
// ---------------------------------------------------------------------------
typedef struct tmc_motion_driver_ops_s {
    // Identification — used in logs.
    const char *name;

    // Raw register access (still needed for fields not abstracted below:
    // RAMP_STAT clear, GCONF.shaft toggle, XTARGET writes from closed-loop).
    esp_err_t (*write_register)(void *chip, uint8_t reg, uint32_t value);
    esp_err_t (*read_register )(void *chip, uint8_t reg, uint32_t *out);

    // Power / current
    esp_err_t (*set_enabled         )(void *chip, bool enabled);
    esp_err_t (*set_rms_current_ma  )(void *chip, uint16_t run_ma, uint8_t hold_pct);

    // Motion
    esp_err_t (*set_amax            )(void *chip, uint32_t amax);
    esp_err_t (*set_velocity_mode   )(void *chip, int32_t  vmax);
    esp_err_t (*set_position_mode   )(void *chip, int32_t  xtarget);
    esp_err_t (*stop                )(void *chip);

    // Features
    esp_err_t (*configure_stallguard )(void *chip, int8_t   sgt,
                                                   bool     sfilt,
                                                   uint32_t tcoolthrs,
                                                   bool     sg_stop);
    esp_err_t (*configure_coolstep   )(void *chip, uint8_t  semin,
                                                   uint8_t  semax,
                                                   uint8_t  seup,
                                                   uint8_t  sedn,
                                                   bool     seimin);
    esp_err_t (*configure_stealthchop)(void *chip, bool     enable,
                                                   uint32_t tpwmthrs);

    // Diagnostics
    esp_err_t (*collect_diag        )(void *chip, tmc_motion_diag_t *out);

    // Register addresses — both 5240 and 5160 share the addresses, but
    // exposing them here lets a future chip variant remap if needed.
    uint8_t reg_xactual;
    uint8_t reg_xtarget;
    uint8_t reg_vmax;
    uint8_t reg_amax;
    uint8_t reg_a1;
    uint8_t reg_v1;
    uint8_t reg_dmax;
    uint8_t reg_d1;
    uint8_t reg_vstart;
    uint8_t reg_vstop;
    uint8_t reg_tzerowait;
    uint8_t reg_rampmode;
    uint8_t reg_ramp_stat;
    uint8_t reg_gconf;
} tmc_motion_driver_ops_t;

#ifdef __cplusplus
}
#endif
