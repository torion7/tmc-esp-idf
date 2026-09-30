#pragma once

// Self-contained driver for the Trinamic TMC5160 stepper motor controller
// over SPI. The library has no project-specific dependencies and can be
// dropped into any ESP-IDF project.
//
// Two SPI chip-select schemes are supported:
//   1. Native CS pin   -- set `pin_cs` to a valid GPIO; leave `cs_ops` NULL
//   2. External CS hook -- set `pin_cs = GPIO_NUM_NC` and provide `cs_ops`
//      callbacks (used e.g. with the `cs_mux` component for a 1-of-N
//      multiplexer that fans out a single CS to multiple drivers).
//
// The driver can either own the SPI bus (default) or attach itself to a
// bus that is already initialized by the application (`share_spi_bus`).
//
// Current scaling: the coil current is measured across external sense
// resistors RSENSE (a board property, see `rsense_milliohm`):
//
//     IFS_rms = V_FS / RSENSE / sqrt(2),   V_FS = 0.325 V
//     I_rms   = IFS_rms * GLOBALSCALER/256 * (IRUN+1)/32
//
// GLOBALSCALER 1..31 is "not allowed for operation"; 0 means full scale
// (= 256).

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#include "tmc5160_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tmc5160_dev_t *tmc5160_handle_t;

// Optional chip-select callback hooks. Both functions must be set when
// `cs_ops` is used. The same `user` pointer is forwarded to each call.
typedef struct {
    esp_err_t (*select)(void *user);
    esp_err_t (*deselect)(void *user);
    void     *user;
} tmc5160_cs_ops_t;

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
    const tmc5160_cs_ops_t *cs_ops;

    // Optional driver-enable pin (DRV_ENN). Set to GPIO_NUM_NC if unused.
    gpio_num_t pin_enable;
    bool       enable_active_low;

    // Sense resistor used for current scaling, in milliohms. This is a BOARD
    // property — read it off the schematic, it cannot be detected. Every
    // current the driver computes scales with 1/rsense_milliohm.
    // 0 -> default 75 mOhm.
    uint32_t rsense_milliohm;
} tmc5160_config_t;

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
} tmc5160_defaults_t;

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
    uint32_t tstep;
    int32_t  xactual;
    int32_t  vactual;
    uint32_t tpwmthrs;
    uint32_t pwm_scale;   // PWM_SCALE (0x71): [9:0] sum, [24:16] auto (signed)
} tmc5160_diag_t;

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
esp_err_t tmc5160_init(const tmc5160_config_t *cfg, tmc5160_handle_t *out_handle);
void      tmc5160_deinit(tmc5160_handle_t handle);

// ---------------------------------------------------------------------------
// Raw register access
// ---------------------------------------------------------------------------
esp_err_t tmc5160_write_register(tmc5160_handle_t handle, uint8_t reg, uint32_t value);
esp_err_t tmc5160_read_register (tmc5160_handle_t handle, uint8_t reg, uint32_t *value);

// ---------------------------------------------------------------------------
// Driver enable (DRV_ENN). Auto-detects polarity if the configured one
// disagrees with the IOIN read-back, useful for boards with inverted
// enable lines.
// ---------------------------------------------------------------------------
esp_err_t tmc5160_set_enabled(tmc5160_handle_t handle, bool enabled);

// ---------------------------------------------------------------------------
// One-shot setup of GCONF / CHOPCONF / motion profile / current.
// ---------------------------------------------------------------------------
esp_err_t tmc5160_apply_defaults(tmc5160_handle_t handle, const tmc5160_defaults_t *defaults);

// ---------------------------------------------------------------------------
// Current scaling
// ---------------------------------------------------------------------------
// Picks the GLOBALSCALER/IRUN pair closest to irms_ma that is at most 2 %
// above it (on equal error the higher IRUN). Requests above full scale are
// clamped to it, requests below the smallest step (full scale / 256) are
// raised to it — both with a warning. See tmc5160_get_realized_rms_ma().
esp_err_t tmc5160_set_rms_current_ma(tmc5160_handle_t handle, uint16_t irms_ma, uint8_t hold_percent);

// Rejects 1..31 with ESP_ERR_INVALID_ARG — that band is not allowed for
// operation. 0 = full scale (256).
esp_err_t tmc5160_set_global_scaler  (tmc5160_handle_t handle, uint8_t scaler);

// Update the board's sense resistor at runtime (0 -> 75 mOhm). Does not
// touch the chip; affects subsequent current maths.
esp_err_t tmc5160_set_rsense_milliohm(tmc5160_handle_t handle, uint32_t rsense_milliohm);

// ---------------------------------------------------------------------------
// Current maths, exposed so a UI can show what is actually achievable.
// ---------------------------------------------------------------------------
// Full-scale RMS coil current in mA for a sense resistor (0 -> 75 mOhm).
// Pure function — no handle, no hardware, usable in host tests and for UI
// previews.
float tmc5160_fullscale_rms_ma(uint32_t rsense_milliohm);

// Same for the handle's configured sense resistor.
float tmc5160_get_fullscale_rms_ma(tmc5160_handle_t handle);

// What the last successful tmc5160_set_rms_current_ma() actually realized,
// in mA RMS. GLOBALSCALER and IRUN are coarse steps, so the realized value
// can differ noticeably from the requested one — especially at low currents.
// Returns 0.0 before the first current has been set.
float tmc5160_get_realized_rms_ma(tmc5160_handle_t handle);

// ---------------------------------------------------------------------------
// Motion control
// ---------------------------------------------------------------------------
esp_err_t tmc5160_set_amax         (tmc5160_handle_t handle, uint32_t amax);
esp_err_t tmc5160_set_velocity_mode(tmc5160_handle_t handle, int32_t vmax);
esp_err_t tmc5160_set_position_mode(tmc5160_handle_t handle, int32_t xtarget);
esp_err_t tmc5160_stop             (tmc5160_handle_t handle);

// ---------------------------------------------------------------------------
// Feature configuration
// ---------------------------------------------------------------------------
esp_err_t tmc5160_configure_stallguard (tmc5160_handle_t handle,
                                        int16_t  sgt,
                                        bool     sfilt,
                                        uint32_t tcoolthrs,
                                        bool     sg_stop);

esp_err_t tmc5160_configure_coolstep   (tmc5160_handle_t handle,
                                        uint8_t semin,
                                        uint8_t semax,
                                        uint8_t seup,
                                        uint8_t sedn,
                                        bool    seimin);

esp_err_t tmc5160_configure_stealthchop(tmc5160_handle_t handle,
                                        bool     enable,
                                        uint32_t tpwmthrs);

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
esp_err_t tmc5160_collect_diag(tmc5160_handle_t handle, tmc5160_diag_t *diag);

#ifdef __cplusplus
}
#endif
