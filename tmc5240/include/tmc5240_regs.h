#pragma once

// Register addresses for the TMC5240 SPI protocol. Most addresses match
// the TMC5160 family; TMC5240-only registers are listed at the bottom.

#ifdef __cplusplus
extern "C" {
#endif

enum {
    TMC5240_REG_GCONF        = 0x00,
    TMC5240_REG_GSTAT        = 0x01,
    TMC5240_REG_IFCNT        = 0x02,
    TMC5240_REG_IOIN         = 0x04,
    TMC5240_REG_DRV_CONF     = 0x0A,
    TMC5240_REG_GLOBALSCALER = 0x0B,
    TMC5240_REG_IHOLD_IRUN   = 0x10,
    TMC5240_REG_TPOWERDOWN   = 0x11,
    TMC5240_REG_TSTEP        = 0x12,
    TMC5240_REG_TPWMTHRS     = 0x13,
    TMC5240_REG_TCOOLTHRS    = 0x14,
    TMC5240_REG_THIGH        = 0x15,
    TMC5240_REG_RAMPMODE     = 0x20,
    TMC5240_REG_XACTUAL      = 0x21,
    TMC5240_REG_VACTUAL      = 0x22,
    TMC5240_REG_VSTART       = 0x23,
    TMC5240_REG_A1           = 0x24,
    TMC5240_REG_V1           = 0x25,
    TMC5240_REG_AMAX         = 0x26,
    TMC5240_REG_VMAX         = 0x27,
    TMC5240_REG_DMAX         = 0x28,
    TMC5240_REG_TVMAX        = 0x29,    // (TMC5240-specific name; same address as TMC5160 D1)
    TMC5240_REG_D1           = 0x2A,
    TMC5240_REG_VSTOP        = 0x2B,
    TMC5240_REG_TZEROWAIT    = 0x2C,
    TMC5240_REG_XTARGET      = 0x2D,
    TMC5240_REG_SW_MODE      = 0x34,
    TMC5240_REG_RAMP_STAT    = 0x35,
    TMC5240_REG_ENCMODE      = 0x38,
    TMC5240_REG_X_ENC        = 0x39,
    TMC5240_REG_ENC_CONST    = 0x3A,
    TMC5240_REG_ENC_STATUS   = 0x3B,
    TMC5240_REG_ENC_LATCH    = 0x3C,
    TMC5240_REG_ENC_DEVIATION = 0x3D,
    TMC5240_REG_ADC_VSUPPLY_AIN = 0x50,
    TMC5240_REG_ADC_TEMP     = 0x51,
    TMC5240_REG_CHOPCONF     = 0x6C,
    TMC5240_REG_COOLCONF     = 0x6D,
    TMC5240_REG_DRV_STATUS   = 0x6F,
    TMC5240_REG_PWMCONF      = 0x70,
    TMC5240_REG_PWM_SCALE    = 0x71,
    TMC5240_REG_PWM_AUTO     = 0x72,
    TMC5240_REG_SG4_THRS     = 0x74,
    TMC5240_REG_SG4_RESULT   = 0x75,
};

// DRV_CONF.current_range bits [1:0] select the KIFS constant that, together
// with the external reference resistor RREF at pin IREF, sets the full-scale
// current:  IFS_peak = KIFS / RREF,  IFS_rms = IFS_peak / sqrt(2).
//
// It ALSO sets the overcurrent trip point. Prefer the lowest range that still
// covers the motor — finer resolution and earlier protection.
//
//   value  KIFS [A*kOhm]  IFS_rms @ RREF=12k   OCP trip
//   0b00      11.75            0.692 A          1.67 A
//   0b01      24.0             1.414 A          3.33 A
//   0b10      36.0             2.121 A          5.00 A
//   0b11      36.0             2.121 A          5.00 A   (identical to 0b10)
//
// There is NO V_FS and NO sense resistor on this part — see tmc5240.h.
enum {
    TMC5240_CURRENT_RANGE_1A = 0,
    TMC5240_CURRENT_RANGE_2A = 1,
    TMC5240_CURRENT_RANGE_3A = 2,
    TMC5240_CURRENT_RANGE_3A_ALT = 3,   // laut Datenblatt identisch zu 0b10
};

#ifdef __cplusplus
}
#endif
