#include "tmc_motion_units.h"

// TMC5160/5240 datasheet conversions:
//   v [microsteps/s]   = VMAX * f_CLK / 2^24
//   a [microsteps/s^2] = AMAX * f_CLK^2 / 2^41
//
// 1 revolution = microsteps_per_rev microsteps, hence:
//   1 rev/s   == microsteps_per_rev microsteps/s
//   1 rev/min == microsteps_per_rev / 60 microsteps/s
//   1 deg     == microsteps_per_rev / 360 microsteps

#define TMC_VMAX_2POW   16777216.0       // 2^24
#define TMC_AMAX_2POW   2199023255552.0  // 2^41

static double clamp_d(double v, double lo, double hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int32_t round_to_int32(double v)
{
    if (v >  2147483647.0) return  2147483647;
    if (v < -2147483648.0) return (int32_t)(-2147483647 - 1);
    return (int32_t)((v >= 0.0) ? (v + 0.5) : (v - 0.5));
}

static uint32_t rpm_to_vmax_unsigned(double rpm, uint32_t microsteps_per_rev, uint32_t fclk_hz)
{
    if (rpm < 0.0) rpm = -rpm;
    double v_steps_per_s = rpm * ((double)microsteps_per_rev / 60.0);
    double vmax          = v_steps_per_s * TMC_VMAX_2POW / (double)fclk_hz;
    vmax                 = clamp_d(vmax, 0.0, (double)TMC_MOTION_VMAX_MAX);
    return (uint32_t)(vmax + 0.5);
}

uint32_t tmc_motion_units_rpm_to_vmax(double rpm, uint32_t microsteps_per_rev, uint32_t fclk_hz)
{
    return rpm_to_vmax_unsigned(rpm, microsteps_per_rev, fclk_hz);
}

double tmc_motion_units_vmax_to_rpm(uint32_t vmax, uint32_t microsteps_per_rev, uint32_t fclk_hz)
{
    if (microsteps_per_rev == 0 || fclk_hz == 0) return 0.0;
    double v_steps_per_s = (double)vmax * (double)fclk_hz / TMC_VMAX_2POW;
    return v_steps_per_s * (60.0 / (double)microsteps_per_rev);
}

uint32_t tmc_motion_units_rps_to_vmax(double rps, uint32_t microsteps_per_rev, uint32_t fclk_hz)
{
    return rpm_to_vmax_unsigned(rps * 60.0, microsteps_per_rev, fclk_hz);
}

double tmc_motion_units_vmax_to_rps(uint32_t vmax, uint32_t microsteps_per_rev, uint32_t fclk_hz)
{
    return tmc_motion_units_vmax_to_rpm(vmax, microsteps_per_rev, fclk_hz) / 60.0;
}

uint32_t tmc_motion_units_rps2_to_amax(double rps2, uint32_t microsteps_per_rev, uint32_t fclk_hz)
{
    if (rps2 < 0.0) rps2 = -rps2;
    double a_steps_per_s2 = rps2 * (double)microsteps_per_rev;
    double amax           = a_steps_per_s2 * TMC_AMAX_2POW / ((double)fclk_hz * (double)fclk_hz);
    amax                  = clamp_d(amax, 0.0, (double)TMC_MOTION_AMAX_MAX);
    return (uint32_t)(amax + 0.5);
}

double tmc_motion_units_amax_to_rps2(uint32_t amax, uint32_t microsteps_per_rev, uint32_t fclk_hz)
{
    if (microsteps_per_rev == 0 || fclk_hz == 0) return 0.0;
    double a_steps_per_s2 = (double)amax * (double)fclk_hz * (double)fclk_hz / TMC_AMAX_2POW;
    return a_steps_per_s2 / (double)microsteps_per_rev;
}

int32_t tmc_motion_units_degrees_to_steps(double degrees, uint32_t microsteps_per_rev)
{
    return round_to_int32(degrees * ((double)microsteps_per_rev / 360.0));
}

double tmc_motion_units_steps_to_degrees(int32_t steps, uint32_t microsteps_per_rev)
{
    if (microsteps_per_rev == 0) return 0.0;
    return (double)steps * (360.0 / (double)microsteps_per_rev);
}

double tmc_motion_units_vmax_signed_to_rpm(int32_t v_signed, uint32_t microsteps_per_rev, uint32_t fclk_hz)
{
    if (v_signed >= 0) return tmc_motion_units_vmax_to_rpm((uint32_t)v_signed, microsteps_per_rev, fclk_hz);
    return -tmc_motion_units_vmax_to_rpm((uint32_t)(-(int64_t)v_signed), microsteps_per_rev, fclk_hz);
}
