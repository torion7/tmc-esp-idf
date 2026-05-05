#pragma once

// Pure unit conversions between physical (rpm, rev/s^2, degrees) and the
// chip's native register units (VMAX, AMAX, microsteps).
//
// TMC5160/5240 datasheet:
//   v [microsteps/s]   = VMAX * f_CLK / 2^24
//   a [microsteps/s^2] = AMAX * f_CLK^2 / 2^41
//
// All helpers are stateless. The caller passes the per-motor parameters
// (microsteps_per_rev, fclk_hz). The tmc_motion handle-API exposes thin
// wrappers that bind these to the configured motor.

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Hard limits used for clamping. The TMC5160/5240 store VMAX/V1/VSTART/VSTOP
// in 23-bit unsigned registers (max 8388607); AMAX/DMAX/A1/D1 in 16-bit
// (max 65535).
// ---------------------------------------------------------------------------
#define TMC_MOTION_VMAX_MAX  8388607u
#define TMC_MOTION_AMAX_MAX  65535u

// Velocity:  rpm  <-> VMAX register units
uint32_t tmc_motion_units_rpm_to_vmax (double   rpm,  uint32_t microsteps_per_rev, uint32_t fclk_hz);
double   tmc_motion_units_vmax_to_rpm (uint32_t vmax, uint32_t microsteps_per_rev, uint32_t fclk_hz);

// Velocity:  rev/s <-> VMAX register units
uint32_t tmc_motion_units_rps_to_vmax (double   rps,  uint32_t microsteps_per_rev, uint32_t fclk_hz);
double   tmc_motion_units_vmax_to_rps (uint32_t vmax, uint32_t microsteps_per_rev, uint32_t fclk_hz);

// Acceleration:  rev/s^2 <-> AMAX register units
uint32_t tmc_motion_units_rps2_to_amax(double   rps2, uint32_t microsteps_per_rev, uint32_t fclk_hz);
double   tmc_motion_units_amax_to_rps2(uint32_t amax, uint32_t microsteps_per_rev, uint32_t fclk_hz);

// Position:  degrees <-> microsteps
int32_t  tmc_motion_units_degrees_to_steps(double  degrees, uint32_t microsteps_per_rev);
double   tmc_motion_units_steps_to_degrees(int32_t steps,   uint32_t microsteps_per_rev);

// Sign-preserving wrapper around vmax_to_rpm. Useful for VACTUAL or
// commanded_velocity which carry direction in their sign.
double   tmc_motion_units_vmax_signed_to_rpm(int32_t v_signed, uint32_t microsteps_per_rev, uint32_t fclk_hz);

#ifdef __cplusplus
}
#endif
