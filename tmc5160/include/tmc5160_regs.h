#pragma once

// Register addresses for the TMC5160 SPI protocol.
// (Most addresses are also valid for the closely related TMC5150/TMC5130,
// but TMC5160-specific registers like GLOBALSCALER are included.)

#ifdef __cplusplus
extern "C" {
#endif

enum {
    TMC5160_REG_GCONF        = 0x00,
    TMC5160_REG_GSTAT        = 0x01,
    TMC5160_REG_IOIN         = 0x04,
    TMC5160_REG_SHORT_CONF   = 0x09,
    TMC5160_REG_DRV_CONF     = 0x0A,
    TMC5160_REG_GLOBALSCALER = 0x0B,
    TMC5160_REG_IHOLD_IRUN   = 0x10,
    TMC5160_REG_TPOWERDOWN   = 0x11,
    TMC5160_REG_TSTEP        = 0x12,
    TMC5160_REG_TPWMTHRS     = 0x13,
    TMC5160_REG_TCOOLTHRS    = 0x14,
    TMC5160_REG_RAMPMODE     = 0x20,
    TMC5160_REG_XACTUAL      = 0x21,
    TMC5160_REG_VACTUAL      = 0x22,
    TMC5160_REG_VSTART       = 0x23,
    TMC5160_REG_A1           = 0x24,
    TMC5160_REG_V1           = 0x25,
    TMC5160_REG_AMAX         = 0x26,
    TMC5160_REG_VMAX         = 0x27,
    TMC5160_REG_D1           = 0x28,
    TMC5160_REG_DMAX         = 0x29,
    TMC5160_REG_VSTOP        = 0x2B,
    TMC5160_REG_TZEROWAIT    = 0x2C,
    TMC5160_REG_XTARGET      = 0x2D,
    TMC5160_REG_SW_MODE      = 0x34,
    TMC5160_REG_RAMP_STAT    = 0x35,
    TMC5160_REG_CHOPCONF     = 0x6C,
    TMC5160_REG_COOLCONF     = 0x6D,
    TMC5160_REG_DRV_STATUS   = 0x6F,
    TMC5160_REG_PWMCONF      = 0x70,
    TMC5160_REG_PWM_SCALE    = 0x71,
};

#ifdef __cplusplus
}
#endif
