#pragma once

// tmc_motion — high-level, chip-agnostic motion service for Trinamic
// TMC5160 / TMC5240 stepper drivers (and any future chip with the same
// motion-controller layout).
//
// Design principles:
//   * The library does NOT touch SPI: the application initializes a chip
//     driver (tmc5240_init / tmc5160_init) and hands the resulting chip
//     handle plus a driver-ops adapter (`tmc_motion_ops_5240` etc.) to
//     tmc_motion_create().
//   * Multi-instance: every call takes a tmc_motion_handle_t. Multiple
//     motors run independently, each with its own mutex, motion profile,
//     stallguard test/calibration tasks, etc.
//   * Physical units: the public motion API speaks rpm / rev/s^2 / degrees;
//     internal conversion to chip register units uses the per-handle
//     mechanical (microsteps_per_rev) and clock (fclk_hz) parameters.
//   * Optional add-ons (closed-loop encoder regulation, web UI) are
//     separate components that consume the public API.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "tmc_motion_driver.h"
#include "tmc_motion_units.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tmc_motion_s *tmc_motion_handle_t;

// ---------------------------------------------------------------------------
// Configuration structs (all values in physical units)
// ---------------------------------------------------------------------------
typedef struct {
    double   default_rpm;        // VMAX used when a motion command does not specify a speed
    double   velocity_limit_rpm; // upper clamp; 0 -> hardware max
    double   hold_rpm;           // VMAX used by the position-hold engage path
    uint16_t run_current_ma;
    uint8_t  hold_percent;
    double   vstart_rpm;
    double   a1_rps2;            // first-stage acceleration
    double   v1_rpm;              // threshold velocity for a1 -> amax
    double   amax_rps2;
    double   dmax_rps2;
    double   d1_rps2;            // final-stage deceleration
    double   vstop_rpm;
    uint32_t tzerowait;          // raw chip ticks (time unit, not motion unit)
} tmc_motion_config_phys_t;

typedef struct {
    int16_t  sgt;
    bool     sfilt;
    uint32_t tcoolthrs;
    bool     sg_stop;
} tmc_motion_stallguard_config_t;

typedef struct {
    uint8_t  semin;   // 0 = CoolStep disabled, 1..15 = active
    uint8_t  semax;
    uint8_t  seup;
    uint8_t  sedn;
    bool     seimin;
} tmc_motion_coolstep_config_t;

typedef struct {
    bool     enable;
    uint32_t tpwmthrs;
} tmc_motion_stealthchop_config_t;

// ---------------------------------------------------------------------------
// Lifecycle config
// ---------------------------------------------------------------------------
typedef struct {
    // Opaque chip handle, already initialized by the caller.
    void                          *chip_handle;
    // Driver-ops adapter for this chip.
    const tmc_motion_driver_ops_t *driver_ops;

    // Mechanical / electrical parameters of THIS motor.
    uint32_t fullsteps_per_rev;        // typical 200 for NEMA17/23
    uint32_t microsteps_per_fullstep;  // typical 256 (chip default)
    uint32_t fclk_hz;                  // chip f_CLK; TMC internal default = 12 MHz

    // Initial motion profile (physical units). All derived registers are
    // pushed to the chip during create().
    tmc_motion_config_phys_t       initial_motion;

    // Initial feature configurations. NULL means "leave chip defaults".
    const tmc_motion_stallguard_config_t  *initial_stallguard;
    const tmc_motion_coolstep_config_t    *initial_coolstep;
    const tmc_motion_stealthchop_config_t *initial_stealthchop;

    // Logical direction default. The motion layer maps logical units to the
    // driver's raw direction so commands and displayed positions stay aligned.
    bool                            initial_motor_inverted;

    // Optional name (used in logs and the UI).
    const char                     *name;
} tmc_motion_config_t;

// ---------------------------------------------------------------------------
// Snapshot returned by tmc_motion_get_snapshot
// ---------------------------------------------------------------------------
typedef struct {
    bool     drv_enn;
    bool     sd_mode;
    bool     swcomp;
    bool     stst;
    bool     stealth;
    bool     ola, olb;
    bool     s2ga, s2gb;
    bool     s2vsa, s2vsb;
    bool     otpw, ot;
    bool     stall;
    bool     fsactive;
    bool     status_sg;
    bool     event_stop_sg;
    bool     position_reached;
    bool     velocity_reached;
    uint32_t cs_actual;
    uint32_t sg_result;
    uint32_t sg4_result;
    // stealthChop-PWM (aus PWM_SCALE): Duty-Summe [0..1023], Auto-Offset
    // (vorzeichenbehaftet) und die Summe als Prozent der Vollaussteuerung.
    uint32_t pwm_scale_sum;
    int32_t  pwm_scale_auto;
    uint32_t pwm_scale_pct;
    // Chip-Temperatur in Grad C (TMC5240-ADC, Datenblatt-Formel
    // (ADC_TEMP - 2038) / 7.7); 0.0 wenn der Chip keinen Temp-ADC hat.
    float    chip_temp_c;
} tmc_motion_decoded_diag_t;

typedef struct {
    int32_t  commanded_velocity;   // chip-units (sign = direction)
    int32_t  xtarget;              // chip-units (microsteps)
    tmc_motion_config_phys_t profile;  // current profile in physical units
} tmc_motion_motion_state_t;

typedef struct {
    uint32_t timestamp_ms;
    int32_t  xactual;
    int32_t  vactual;
    uint32_t sg_result;
    uint32_t tstep;
    uint32_t ramp_stat;
    uint32_t drv_status;
    bool     status_sg;
    bool     event_stop_sg;
} tmc_motion_stallguard_sample_t;

typedef struct {
    int32_t  velocity;
    uint32_t duration_ms;
    uint32_t sample_period_ms;
} tmc_motion_stallguard_test_request_t;

typedef struct {
    bool     active;
    bool     completed;
    bool     overflow;
    bool     cancel_requested;
    bool     event_stop_sg;
    uint32_t duration_ms;
    uint32_t sample_period_ms;
    uint32_t elapsed_ms;
    int32_t  velocity;
    uint32_t sample_count;
} tmc_motion_stallguard_test_status_t;

typedef struct {
    uint16_t run_current_ma;
    uint32_t amax;
    int32_t  velocity;
    uint32_t sg_min, sg_max, sg_avg;
    uint32_t tstep_avg;
    bool     event_stop_sg;
} tmc_motion_stallguard_calibration_point_t;

typedef struct {
    uint16_t current_min_ma, current_max_ma, current_step_ma;
    uint32_t amax_min, amax_max, amax_step;
    int32_t  velocity_min, velocity_max, velocity_step;
    uint32_t settle_ms;
    uint32_t point_duration_ms;
    uint32_t sample_period_ms;
} tmc_motion_stallguard_calibration_request_t;

typedef struct {
    bool     active;
    bool     completed;
    bool     cancel_requested;
    bool     has_recommendation;
    uint32_t total_points;
    uint32_t tested_points;
    uint32_t settle_ms;
    uint32_t point_duration_ms;
    uint32_t sample_period_ms;
    int8_t   suggested_sgt;
    uint32_t suggested_tcoolthrs;
    uint32_t worst_sg_min;
} tmc_motion_stallguard_calibration_status_t;

typedef struct {
    bool                                       initialized;
    const char                                *name;       // borrowed; do not free
    const char                                *driver_name;

    esp_err_t                                  last_error_code;
    char                                       last_error[96];

    // Motor parameters that the UI may want to display.
    uint32_t                                   microsteps_per_rev;
    uint32_t                                   fclk_hz;
    bool                                       motor_inverted;

    uint64_t                                   last_diag_ms;
    tmc_motion_diag_t                          diag;
    tmc_motion_decoded_diag_t                  decoded;
    tmc_motion_motion_state_t                  motion;
    tmc_motion_stallguard_config_t             stallguard;
    tmc_motion_coolstep_config_t               coolstep;
    tmc_motion_stealthchop_config_t            stealthchop;
    tmc_motion_stallguard_test_status_t        sg_test;
    tmc_motion_stallguard_calibration_status_t sg_calibration;
} tmc_motion_snapshot_t;

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
esp_err_t tmc_motion_create (const tmc_motion_config_t *cfg,
                             tmc_motion_handle_t *out);
void      tmc_motion_destroy(tmc_motion_handle_t h);

// ---------------------------------------------------------------------------
// Motion commands (physical units)
// ---------------------------------------------------------------------------
// Sign of `rpm` selects direction.
esp_err_t tmc_motion_set_velocity_rpm    (tmc_motion_handle_t h, double rpm);
// `degrees` is absolute relative to the current zero. `max_rpm <= 0` -> default.
esp_err_t tmc_motion_set_position_degrees(tmc_motion_handle_t h, double degrees, double max_rpm);
esp_err_t tmc_motion_stop                (tmc_motion_handle_t h);
esp_err_t tmc_motion_set_position_zero   (tmc_motion_handle_t h);
esp_err_t tmc_motion_set_current         (tmc_motion_handle_t h, uint16_t run_ma, uint8_t hold_pct);
esp_err_t tmc_motion_set_motor_inverted  (tmc_motion_handle_t h, bool inverted);
esp_err_t tmc_motion_set_fullsteps_per_rev(tmc_motion_handle_t h, uint16_t fullsteps_per_rev);
esp_err_t tmc_motion_set_microsteps_per_fullstep(tmc_motion_handle_t h, uint16_t microsteps);
esp_err_t tmc_motion_configure_motion_phys(tmc_motion_handle_t h,
                                           const tmc_motion_config_phys_t *cfg);

// ---------------------------------------------------------------------------
// Feature configuration
// ---------------------------------------------------------------------------
esp_err_t tmc_motion_configure_stallguard (tmc_motion_handle_t h,
                                           const tmc_motion_stallguard_config_t *cfg);
esp_err_t tmc_motion_configure_coolstep   (tmc_motion_handle_t h,
                                           const tmc_motion_coolstep_config_t *cfg);
esp_err_t tmc_motion_configure_stealthchop(tmc_motion_handle_t h,
                                           const tmc_motion_stealthchop_config_t *cfg);

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
esp_err_t tmc_motion_get_snapshot(tmc_motion_handle_t h, tmc_motion_snapshot_t *out);

// ---------------------------------------------------------------------------
// StallGuard test (constant-velocity sample window)
// ---------------------------------------------------------------------------
esp_err_t tmc_motion_start_stallguard_test  (tmc_motion_handle_t h,
                                             const tmc_motion_stallguard_test_request_t *request);
esp_err_t tmc_motion_get_stallguard_samples (tmc_motion_handle_t h,
                                             tmc_motion_stallguard_sample_t *samples,
                                             size_t max_samples,
                                             size_t *out_count);

// StallGuard calibration sweep (current x amax x velocity)
esp_err_t tmc_motion_start_stallguard_calibration(tmc_motion_handle_t h,
                                                  const tmc_motion_stallguard_calibration_request_t *request);
esp_err_t tmc_motion_get_stallguard_calibration_points(tmc_motion_handle_t h,
                                                       tmc_motion_stallguard_calibration_point_t *points,
                                                       size_t max_points,
                                                       size_t *out_count);

// ---------------------------------------------------------------------------
// Locking + raw register access
//
// External add-ons (closed-loop, manual SPI work) MUST hold the motion lock
// before they touch the underlying chip. tmc_motion_write_register and
// tmc_motion_read_register acquire/release the lock on each call. Use
// _lock/_unlock when you need to chain multiple register accesses
// atomically.
// ---------------------------------------------------------------------------
esp_err_t tmc_motion_lock  (tmc_motion_handle_t h, uint32_t timeout_ticks);
void      tmc_motion_unlock(tmc_motion_handle_t h);

esp_err_t tmc_motion_write_register(tmc_motion_handle_t h, uint8_t reg, uint32_t value);
esp_err_t tmc_motion_read_register (tmc_motion_handle_t h, uint8_t reg, uint32_t *out);

// Accessors used by add-ons.
const tmc_motion_driver_ops_t *tmc_motion_get_ops        (tmc_motion_handle_t h);
void                          *tmc_motion_get_chip_handle(tmc_motion_handle_t h);
uint32_t                       tmc_motion_get_microsteps_per_rev(tmc_motion_handle_t h);
uint32_t                       tmc_motion_get_fclk_hz   (tmc_motion_handle_t h);
const char                    *tmc_motion_get_name      (tmc_motion_handle_t h);

// Convenience unit conversions bound to a handle's parameters.
uint32_t tmc_motion_rpm_to_vmax (tmc_motion_handle_t h, double rpm);
double   tmc_motion_vmax_to_rpm (tmc_motion_handle_t h, uint32_t vmax);
uint32_t tmc_motion_rps2_to_amax(tmc_motion_handle_t h, double rps2);
double   tmc_motion_amax_to_rps2(tmc_motion_handle_t h, uint32_t amax);
int32_t  tmc_motion_degrees_to_steps(tmc_motion_handle_t h, double deg);
double   tmc_motion_steps_to_degrees(tmc_motion_handle_t h, int32_t steps);

#ifdef __cplusplus
}
#endif
