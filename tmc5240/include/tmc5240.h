#pragma once

// Self-contained driver for the Trinamic TMC5240 stepper motor controller
// over SPI. Mirror of the `tmc5160` driver in API style and chip-select
// flexibility (native CS pin or callback-based CS via `cs_ops`).
//
// Key TMC5240 differences vs. TMC5160 handled here:
//   - DRV_CONF carries a `current_range` field selecting one of four V_FS
//     reference voltages used by the current scaler.
//   - GLOBALSCALER accepts the full 0..255 range (TMC5160 disallowed 1..31).
//
// All other operations (motion control, StallGuard, CoolStep, StealthChop,
// register access) follow the same protocol as the TMC5160.

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#include "tmc5240_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tmc5240_dev_t *tmc5240_handle_t;

typedef struct {
    esp_err_t (*select)(void *user);
    esp_err_t (*deselect)(void *user);
    void     *user;
} tmc5240_cs_ops_t;

typedef struct {
    // SPI bus
    spi_host_device_t spi_host;
    gpio_num_t        pin_mosi;
    gpio_num_t        pin_miso;
    gpio_num_t        pin_sclk;
    uint32_t          spi_clock_hz;     // 0 -> default 1 MHz
    bool              share_spi_bus;    // true: bus already initialized externally

    // Chip select: use either pin_cs (native) or cs_ops (callback). When
    // cs_ops is provided, set pin_cs to GPIO_NUM_NC.
    gpio_num_t              pin_cs;
    const tmc5240_cs_ops_t *cs_ops;

    // Optional driver-enable pin (DRV_ENN). Set to GPIO_NUM_NC if unused.
    gpio_num_t pin_enable;
    bool       enable_active_low;

    // Sense resistor used for current scaling (0 -> default 75 mOhm).
    uint32_t rsense_milliohm;

    // DRV_CONF.current_range bits [1:0] (see TMC5240_CURRENT_RANGE_*).
    // Default (0) maps to TMC5240_CURRENT_RANGE_LOW. Pass MAX to match
    // the TMC5160 V_FS = 0.325 V behaviour.
    uint8_t current_range;
} tmc5240_config_t;

typedef struct {
    uint32_t gconf;
    uint32_t chopconf;
    uint8_t  tpowerdown;
    uint8_t  ihold_delay;
    uint16_t run_current_ma;
    uint8_t  hold_current_percent;
    uint32_t amax;
    uint32_t vstart;
    uint32_t vstop;
    uint32_t tzerowait;
} tmc5240_defaults_t;

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
    uint32_t drv_status;
    uint32_t sg4_result;
    uint32_t tstep;
    int32_t  xactual;
    int32_t  vactual;
    uint32_t tpwmthrs;
    uint32_t pwm_scale;   // PWM_SCALE (0x71): [9:0] sum, [24:16] auto (signed)
    uint32_t adc_temp;    // ADC_TEMP (0x51): [12:0] raw temperature ADC value
} tmc5240_diag_t;

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
esp_err_t tmc5240_init(const tmc5240_config_t *cfg, tmc5240_handle_t *out_handle);
void      tmc5240_deinit(tmc5240_handle_t handle);

// ---------------------------------------------------------------------------
// Raw register access
// ---------------------------------------------------------------------------
esp_err_t tmc5240_write_register(tmc5240_handle_t handle, uint8_t reg, uint32_t value);
esp_err_t tmc5240_read_register (tmc5240_handle_t handle, uint8_t reg, uint32_t *value);

// ---------------------------------------------------------------------------
// Driver enable (DRV_ENN). Auto-detects polarity when the configured one
// disagrees with the IOIN read-back.
// ---------------------------------------------------------------------------
esp_err_t tmc5240_set_enabled(tmc5240_handle_t handle, bool enabled);

// ---------------------------------------------------------------------------
// One-shot setup of GCONF / CHOPCONF / motion profile / current.
// ---------------------------------------------------------------------------
esp_err_t tmc5240_apply_defaults(tmc5240_handle_t handle, const tmc5240_defaults_t *defaults);

// ---------------------------------------------------------------------------
// Current scaling
// ---------------------------------------------------------------------------
esp_err_t tmc5240_set_rms_current_ma(tmc5240_handle_t handle, uint16_t irms_ma, uint8_t hold_percent);
esp_err_t tmc5240_set_global_scaler  (tmc5240_handle_t handle, uint8_t scaler);

// Update DRV_CONF.current_range (0..3). The internal current scaler will
// use the new V_FS reference on the next set_rms_current_ma() call.
esp_err_t tmc5240_set_current_range  (tmc5240_handle_t handle, uint8_t range);

// ---------------------------------------------------------------------------
// Motion control
// ---------------------------------------------------------------------------
esp_err_t tmc5240_set_amax         (tmc5240_handle_t handle, uint32_t amax);
esp_err_t tmc5240_set_velocity_mode(tmc5240_handle_t handle, int32_t vmax);
esp_err_t tmc5240_set_position_mode(tmc5240_handle_t handle, int32_t xtarget);
esp_err_t tmc5240_stop             (tmc5240_handle_t handle);

// ---------------------------------------------------------------------------
// Feature configuration
// ---------------------------------------------------------------------------
esp_err_t tmc5240_configure_stallguard (tmc5240_handle_t handle,
                                        int16_t  sgt,
                                        bool     sfilt,
                                        uint32_t tcoolthrs,
                                        bool     sg_stop);

esp_err_t tmc5240_configure_coolstep   (tmc5240_handle_t handle,
                                        uint8_t semin,
                                        uint8_t semax,
                                        uint8_t seup,
                                        uint8_t sedn,
                                        bool    seimin);

esp_err_t tmc5240_configure_stealthchop(tmc5240_handle_t handle,
                                        bool     enable,
                                        uint32_t tpwmthrs);

// ---------------------------------------------------------------------------
// Encoder interface (ABN incremental encoder, used for closed-loop control)
// ---------------------------------------------------------------------------
//
// ENCMODE bits (excerpt; see TMC5240 datasheet for the full table):
//   bit 0  : pol_A         (1 = invert A polarity)
//   bit 1  : pol_B         (1 = invert B polarity)
//   bit 2  : pol_N         (1 = invert N polarity)
//   bit 3  : ignore_AB     (1 = N event regardless of A/B level)
//   bit 4  : clr_cont      (continuously clear X_ENC on N event)
//   bit 5  : clr_once      (clear X_ENC once on next N event)
//   bit 7-6: pos_neg_edge_N (edge selection on the N input)
//   bit 8  : clr_enc_x     (also clear XACTUAL when X_ENC is cleared)
//   bit 9  : latch_x_act   (latch XACTUAL into ENC_LATCH on N event)
//   bit 10 : enc_sel_decimal (1 = ENC_CONST low 16 bits are decimal 1/10000;
//                             0 = binary 1/65536)
//
// ENC_CONST: signed 32-bit, microsteps gained per encoder count.
//   - Binary  mode: high 16 = signed integer part, low 16 = fraction in 1/65536.
//   - Decimal mode: high 16 = signed integer part, low 16 = fraction in 1/10000.
//
// ENC_STATUS bits (write 1 to clear):
//   bit 0 : n_event       (N event was detected since last read)
//   bit 1 : deviation_warn (|XACTUAL - X_ENC*ENC_CONST| > ENC_DEVIATION)
typedef struct {
    uint32_t encmode;          // raw ENCMODE register value
    int32_t  enc_const;        // ENC_CONST register value (Q16.16 binary or Q16 decimal per encmode bit 10)
    uint32_t enc_deviation;    // microsteps; 0 disables the deviation warning
} tmc5240_encoder_config_t;

// Configures ENCMODE / ENC_CONST / ENC_DEVIATION in one shot. Does NOT
// reset X_ENC -- call tmc5240_set_encoder_position() if you need to.
esp_err_t tmc5240_configure_encoder(tmc5240_handle_t handle, const tmc5240_encoder_config_t *cfg);

// Reads the current encoder position (X_ENC, raw signed counts) and the
// latched ENC_STATUS flags. Either pointer may be NULL.
esp_err_t tmc5240_read_encoder(tmc5240_handle_t handle, int32_t *x_enc, uint32_t *enc_status);

// Forces X_ENC to a specific value (signed counts). Useful at startup or
// after homing to align the encoder to the chip's XACTUAL.
esp_err_t tmc5240_set_encoder_position(tmc5240_handle_t handle, int32_t x_enc);

// Clears latched ENC_STATUS bits (write-1-to-clear semantics).
esp_err_t tmc5240_clear_encoder_status(tmc5240_handle_t handle, uint32_t flags);

// Helper: builds an ENC_CONST register value from microsteps-per-count using
// binary fixed-point (Q16.16). Saturates on overflow.
int32_t   tmc5240_enc_const_from_ratio_q16(int32_t int_part, uint16_t frac_q16);

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
esp_err_t tmc5240_collect_diag(tmc5240_handle_t handle, tmc5240_diag_t *diag);

#ifdef __cplusplus
}
#endif
