#pragma once

// Closed-loop encoder regulation for tmc_motion.
//
// This is an OPTIONAL add-on: if a project doesn't need closed-loop
// control, simply do not include this component. The motion library
// itself has no dependency on it.
//
// The regulator is encoder-source-agnostic: callers provide an
// encoder-ops callback table that knows how to read/set the position.
// A ready-made adapter for the TMC5240's hardware ABN encoder ships
// with this component (encoder_5240.c).
//
// Threading model: the regulator runs as its own FreeRTOS task per
// closed-loop handle. Each tick it acquires the motor's tmc_motion
// lock, reads the encoder + chip state, and (if active correction is
// on AND the chip is in position ramp-mode) writes XTARGET so the
// next ramp covers exactly the measured physical error.

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "tmc_motion.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Encoder-ops callback table. Implementations may use any source: the
// chip's own ABN encoder block, an external PCNT-based encoder, etc.
//
// Position is reported in MICROSTEPS in the motor's local microstep
// frame. If the encoder is e.g. 4000 counts/rev and the motor is
// 51200 microsteps/rev, the implementation either pre-applies the
// ratio (1 count = 12.8 microsteps) or relies on chip features that
// already do it (TMC5240 ENC_CONST does this for you).
// ---------------------------------------------------------------------------
typedef struct {
    // Reads the current encoder position in microsteps.
    esp_err_t (*read_position_microsteps)(void *user, int64_t *out);
    // Forces the encoder accumulator to a given microstep value.
    esp_err_t (*set_position_microsteps )(void *user, int64_t microsteps);
    // Optional: reads any chip status flags (deviation warning, N event).
    // May be NULL. Bit layout is encoder-specific.
    esp_err_t (*read_status              )(void *user, uint32_t *out_flags);
    // Opaque pointer forwarded to every callback.
    void *user;
} tmc_closedloop_encoder_ops_t;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
typedef struct {
    tmc_motion_handle_t           motor;
    tmc_closedloop_encoder_ops_t  encoder;

    // Tick period in milliseconds. Clamped to [5, 1000].
    uint32_t period_ms;
    // When true, the regulator writes XTARGET on each tick to compensate
    // measured slip. When false, it only updates status (monitoring).
    bool     active_correction;
    // Microsteps; if encoder backend reports a deviation flag we mirror it.
    uint32_t deviation_warn_microsteps;

    // Legacy fields preserved at the user's request — they have no effect
    // on the direct-correction control law but are passed through to the
    // status snapshot for backwards compatibility with existing UIs.
    uint32_t deadband_steps;
    uint32_t max_correction_steps;
    uint16_t kp_per_mille;
} tmc_closedloop_config_t;

// ---------------------------------------------------------------------------
// Live status
// ---------------------------------------------------------------------------
typedef struct {
    bool     enable;
    bool     active_correction;
    bool     warn_deviation;
    bool     n_event;
    int32_t  x_enc;                 // raw encoder count (top 32 bits of read_position truncated for display)
    int64_t  x_enc_microsteps;      // encoder position in microsteps
    int32_t  xactual;               // chip XACTUAL
    int32_t  intended_xtarget;      // last user-issued target (microsteps)
    int32_t  position_error;        // intended - encoder (clamped to int32)
    int32_t  last_correction;       // measured slip (XACTUAL - encoder, clamped)
    uint32_t enc_status;            // last raw status flags
    uint32_t update_count;
    uint64_t last_update_ms;
} tmc_closedloop_status_t;

typedef struct tmc_closedloop_s *tmc_closedloop_handle_t;

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
esp_err_t tmc_closedloop_create (const tmc_closedloop_config_t *cfg,
                                 tmc_closedloop_handle_t *out);
void      tmc_closedloop_destroy(tmc_closedloop_handle_t h);

// ---------------------------------------------------------------------------
// Runtime control
// ---------------------------------------------------------------------------
esp_err_t tmc_closedloop_set_active_correction(tmc_closedloop_handle_t h, bool on);
esp_err_t tmc_closedloop_set_period_ms        (tmc_closedloop_handle_t h, uint32_t period_ms);
esp_err_t tmc_closedloop_set_deviation_warn   (tmc_closedloop_handle_t h, uint32_t microsteps);

// Aligns the encoder's internal accumulator to the chip's current XACTUAL,
// then sets the regulator's intended target to that XACTUAL too. Use after
// homing or set_position_zero so the next tick sees zero error.
esp_err_t tmc_closedloop_align_to_motor(tmc_closedloop_handle_t h);

// Tells the regulator the motor's intended position. Normally called by
// tmc_motion_set_position_degrees, but apps can call it directly when
// they bypass the public motion API. Microsteps argument matches
// motor's microstep frame.
void      tmc_closedloop_set_intended_xtarget(tmc_closedloop_handle_t h, int32_t microsteps);

// Snapshot of current status.
esp_err_t tmc_closedloop_get_status(tmc_closedloop_handle_t h, tmc_closedloop_status_t *out);

#ifdef __cplusplus
}
#endif
