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

    // Sense resistor used for current scaling (0 -> default 75 mOhm).
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
esp_err_t tmc5160_set_rms_current_ma(tmc5160_handle_t handle, uint16_t irms_ma, uint8_t hold_percent);
esp_err_t tmc5160_set_global_scaler  (tmc5160_handle_t handle, uint8_t scaler);

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
                                        int8_t   sgt,
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
