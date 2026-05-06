#pragma once

// Per-handle internal state shared between tmc_motion translation units.
// Not part of the public API.

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "tmc_motion.h"

#define TMC_MOTION_STALLGUARD_SAMPLE_CAPACITY    160
#define TMC_MOTION_STALLGUARD_CAL_POINT_CAPACITY 180

struct tmc_motion_s {
    SemaphoreHandle_t                lock;
    void                            *chip_handle;
    const tmc_motion_driver_ops_t   *ops;

    char                             name[32];

    // Mechanical / clock
    uint32_t                         fullsteps_per_rev;
    uint32_t                         microsteps_per_rev;
    uint32_t                         fclk_hz;
    bool                             motor_inverted;

    // Configured profiles (mirror of last-applied state).
    tmc_motion_config_phys_t         motion_phys;
    int32_t                          commanded_velocity;  // chip units
    int32_t                          xtarget;             // chip units
    tmc_motion_stallguard_config_t   stallguard;
    tmc_motion_coolstep_config_t     coolstep;
    tmc_motion_stealthchop_config_t  stealthchop;

    // Last-collected diag.
    tmc_motion_diag_t                last_diag;
    uint64_t                         last_diag_ms;

    esp_err_t                        last_error_code;
    char                             last_error[96];

    // StallGuard test
    TaskHandle_t                                  sg_test_task;
    tmc_motion_stallguard_test_status_t           sg_test;
    tmc_motion_stallguard_sample_t                sg_samples[TMC_MOTION_STALLGUARD_SAMPLE_CAPACITY];
    size_t                                        sg_sample_head;
    size_t                                        sg_sample_count;

    // StallGuard calibration sweep
    TaskHandle_t                                       sg_cal_task;
    tmc_motion_stallguard_calibration_status_t         sg_cal;
    tmc_motion_stallguard_calibration_point_t          sg_cal_points[TMC_MOTION_STALLGUARD_CAL_POINT_CAPACITY];
    size_t                                             sg_cal_head;
    size_t                                             sg_cal_count;
};

// Locking helpers (defined in tmc_motion.c).
esp_err_t tmc_motion_lock_internal  (tmc_motion_handle_t h);
void      tmc_motion_unlock_internal(tmc_motion_handle_t h);

// Last-error tracking (caller holds lock).
void      tmc_motion_set_last_error_locked  (tmc_motion_handle_t h, esp_err_t err, const char *msg);
void      tmc_motion_clear_last_error_locked(tmc_motion_handle_t h);

// Diag refresh (caller holds lock).
esp_err_t tmc_motion_refresh_diag_locked(tmc_motion_handle_t h);

// Decode the unified diag struct into bool flags / counters.
void      tmc_motion_decode_diag(const tmc_motion_diag_t *diag,
                                 tmc_motion_decoded_diag_t *out);

// Apply the current motion_phys profile (with given vmax_override) to the
// chip. Caller holds lock. vmax_override == 0 -> use motion_phys.default_rpm.
esp_err_t tmc_motion_apply_profile_locked(tmc_motion_handle_t h, uint32_t vmax_override);

// Clear RAMP_STAT events (write-1-to-clear).
esp_err_t tmc_motion_clear_ramp_status_events_locked(tmc_motion_handle_t h);

// Engage position-hold at current XACTUAL (used after stop / boot / zero
// so that an external closed-loop has a stable target).
esp_err_t tmc_motion_engage_position_hold_locked(tmc_motion_handle_t h);

// Velocity-mode bring-up with optional sg_stop arming. Used by
// set_velocity_rpm and the stallguard test task.
esp_err_t tmc_motion_start_velocity_with_stallguard_arm_locked(tmc_motion_handle_t h,
                                                               int32_t velocity,
                                                               bool    sg_stop_requested);

// Busy = stallguard test or sweep is currently running.
bool      tmc_motion_busy_locked(tmc_motion_handle_t h);

// StallGuard ring-buffer helpers.
void      tmc_motion_reset_stallguard_samples_locked        (tmc_motion_handle_t h);
void      tmc_motion_push_stallguard_sample_locked          (tmc_motion_handle_t h, const tmc_motion_diag_t *diag);
void      tmc_motion_reset_stallguard_calibration_points_locked(tmc_motion_handle_t h);
void      tmc_motion_push_stallguard_calibration_point_locked  (tmc_motion_handle_t h, const tmc_motion_stallguard_calibration_point_t *p);

// Recommendation derivation (after the sweep finished).
void      tmc_motion_derive_stallguard_recommendation_locked(tmc_motion_handle_t h);

// Mutex-acquire macro for stallguard tasks (portMAX_DELAY).
#define TMC_MOTION_BLOCKING_TAKE(h)  (xSemaphoreTake((h)->lock, portMAX_DELAY) == pdTRUE)
#define TMC_MOTION_GIVE(h)            xSemaphoreGive((h)->lock)
