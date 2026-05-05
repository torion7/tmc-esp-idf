#pragma once

// Convenience: builds a tmc_closedloop_encoder_ops_t backed by the
// TMC5240's hardware ABN encoder block. The 5240 may be either the
// motor's primary chip or a separate one wired up just for sensing.

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "tmc_motion_closedloop.h"
#include "tmc5240.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // ENCMODE register raw value (pol_A/B/N, sel_decimal, ...).
    uint32_t encmode;
    // ENC_CONST raw value (Q16.16 binary or Q16 decimal per encmode bit 10).
    int32_t  enc_const;
    // Microsteps; 0 disables the deviation warning.
    uint32_t enc_deviation;
} tmc_closedloop_5240_encoder_init_t;

// Configures the 5240's encoder block, registers an ops table that reads
// X_ENC + ENC_STATUS via the SAME tmc_motion handle's lock (so SPI
// transactions don't collide with motion commands).
//
// `motor_owning_lock` is the tmc_motion_handle_t whose lock will be
// acquired around every encoder read. If the encoder lives on the
// motor's own chip, pass that motor's handle. If it lives on a
// separate chip, the caller is responsible for routing locks
// appropriately (typically: a dedicated motor handle for that chip
// passed here as well).
//
// `chip` is the chip-driver handle whose encoder block is used.
//
// On success, fills `out_ops`. The internal state used by the ops is
// allocated and stays alive for the life of the closed-loop handle;
// it is freed when the closed-loop is destroyed.
esp_err_t tmc_closedloop_5240_make_encoder(
    tmc_motion_handle_t  motor_owning_lock,
    tmc5240_handle_t     chip,
    const tmc_closedloop_5240_encoder_init_t *init,
    tmc_closedloop_encoder_ops_t *out_ops);

// Releases the per-ops state. The closed-loop module calls this in
// destroy(); applications normally don't.
void tmc_closedloop_5240_release_encoder(tmc_closedloop_encoder_ops_t *ops);

#ifdef __cplusplus
}
#endif
