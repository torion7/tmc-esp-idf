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
    TMC5240_REG_CHOPCONF     = 0x6C,
    TMC5240_REG_COOLCONF     = 0x6D,
    TMC5240_REG_DRV_STATUS   = 0x6F,
    TMC5240_REG_PWMCONF      = 0x70,
    TMC5240_REG_SG4_THRS     = 0x74,
    TMC5240_REG_SG4_RESULT   = 0x75,
};

// DRV_CONF.current_range bits [1:0] select the full-scale current reference.
// With external sense resistors the V_FS reference voltage scales accordingly:
//   0 -> low,        1 -> 250mA range,  2 -> 333mA range,  3 -> 500mA range
// (see TMC5240 datasheet section "DRV_CONF"). Use 3 to match the TMC5160's
// default 0.325V V_FS behaviour.
enum {
    TMC5240_CURRENT_RANGE_LOW    = 0,
    TMC5240_CURRENT_RANGE_MEDIUM = 1,
    TMC5240_CURRENT_RANGE_HIGH   = 2,
    TMC5240_CURRENT_RANGE_MAX    = 3,
};

#ifdef __cplusplus
}
#endif
