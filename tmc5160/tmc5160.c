#include "tmc5160.h"

#include <inttypes.h>
#include <math.h>
#include <stdlib.h>

#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "tmc5160";

// ===========================================================================
// SPI framing constants
// ===========================================================================
#define TMC_SPI_FRAME_BITS 40
#define TMC_WRITE_BIT      0x80

// ===========================================================================
// Current scaling constants
// ===========================================================================
#define TMC_GSTAT_CLEAR_MASK         0x00000007UL
#define TMC_GLOBAL_SCALER_FULL_SCALE 256U
#define TMC_GLOBAL_SCALER_MIN_ALLOWED 32U
#define TMC_CURRENT_FULL_SCALE_VOLTS 0.325f
#define TMC_RSENSE_MOHM_DEFAULT      75U

// ===========================================================================
// Default values used in apply_defaults
// ===========================================================================
#define TMC_DEFAULT_TPOWERDOWN 10U
#define TMC_DEFAULT_IHOLDDELAY 6U
#define TMC_DEFAULT_VSTOP      10U

// ===========================================================================
// Internal device state
// ===========================================================================
struct tmc5160_dev_t {
    tmc5160_config_t cfg;
    spi_device_handle_t spi;
    bool     spi_bus_owned;        // true if this driver initialized the bus

    // Owned copy of the chip-select callbacks (the config-supplied
    // pointer is not retained, so callers may pass stack-allocated cs_ops).
    tmc5160_cs_ops_t cs_ops_storage;
    bool             has_cs_ops;

    // Cached register values (write-through shadow)
    uint32_t shadow_ihold_irun;
    uint8_t  shadow_tpowerdown;
    uint8_t  shadow_global_scaler;
    float    realized_rms_ma;          // was set_rms_current_ma zuletzt wirklich stellte
    uint32_t shadow_tcoolthrs;
    uint32_t shadow_coolconf;
    uint32_t shadow_gconf;
    uint32_t shadow_tpwmthrs;

    // Motion-profile state
    uint32_t amax;
    bool     motion_profile_valid;

    // Auto-detected enable polarity (may differ from cfg.enable_active_low)
    bool     enable_active_low_runtime;
};

typedef struct {
    uint8_t global_scaler_reg;
    uint8_t irun_reg;
} tmc5160_current_scaling_t;

// ===========================================================================
// Helper queries
// ===========================================================================
static bool tmc_has_enable_gpio(const struct tmc5160_dev_t *dev)
{
    return dev->cfg.pin_enable != GPIO_NUM_NC;
}

static int tmc_enable_level_for_state(bool enabled, bool active_low)
{
    if (active_low) {
        return enabled ? 0 : 1;
    }
    return enabled ? 1 : 0;
}

static bool tmc_ioin_drv_enn_is_high(uint32_t ioin)
{
    return (ioin & (1U << 4)) != 0;
}

// ===========================================================================
// SPI transfer (with optional CS callback)
// ===========================================================================
static esp_err_t tmc_spi_xfer(struct tmc5160_dev_t *dev, const uint8_t tx[5], uint8_t rx[5])
{
    spi_transaction_t t = {
        .length    = TMC_SPI_FRAME_BITS,
        .rxlength  = TMC_SPI_FRAME_BITS,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t err = ESP_OK;

    if (dev->has_cs_ops) {
        err = dev->cs_ops_storage.select(dev->cs_ops_storage.user);
        if (err != ESP_OK) {
            return err;
        }
    }

    esp_err_t xfer_err = spi_device_polling_transmit(dev->spi, &t);

    if (dev->has_cs_ops) {
        esp_err_t deselect_err = dev->cs_ops_storage.deselect(dev->cs_ops_storage.user);
        if (xfer_err == ESP_OK) {
            xfer_err = deselect_err;
        }
    }

    return xfer_err;
}

// ===========================================================================
// Current scaling math
// ===========================================================================
static uint32_t tmc_rsense_or_default(uint32_t rsense_milliohm)
{
    return rsense_milliohm == 0 ? TMC_RSENSE_MOHM_DEFAULT : rsense_milliohm;
}

float tmc5160_fullscale_rms_ma(uint32_t rsense_milliohm)
{
    float rsense_ohm = (float)tmc_rsense_or_default(rsense_milliohm) / 1000.0f;
    // V_FS [V] / RSENSE [Ohm] = IFS peak [A]; /sqrt(2) -> RMS; *1000 -> mA
    return TMC_CURRENT_FULL_SCALE_VOLTS / rsense_ohm / 1.41421356f * 1000.0f;
}

static float tmc_fullscale_irms_amps(uint32_t rsense_milliohm)
{
    return tmc5160_fullscale_rms_ma(rsense_milliohm) / 1000.0f;
}

// Sucht das GLOBALSCALER/IRUN-Paar, dessen Produkt (GS/256)*((IRUN+1)/32)
// dem Zielwert am naechsten kommt, OHNE nennenswert darueber zu liegen.
// Dieselbe Suche wie in tmc5240.c — nur der Vollausschlag unterscheidet sich.
//
// Wichtig gegenueber der frueheren Fassung:
//   * Es gibt keine kuenstliche Untergrenze mehr. Die alte Variante verlangte
//     im ersten Durchlauf IRUN >= 16 UND GS >= 128 und kehrte beim ersten
//     Treffer zurueck — die weiteren Durchlaeufe waren damit toter Code. Die
//     kleinste darstellbare Skalierung war (128/256)*(17/32) = 0,2656, und
//     jede kleinere Anforderung wurde stillschweigend darauf ANGEHOBEN.
//     Bei 75 mOhm hiess das: 500 mA angefordert, ~814 mA gestellt.
//   * Jetzt reicht der Bereich bis (32/256)*(1/32) = 0,0039 hinunter.
//   * GLOBALSCALER 1..31 wird nie erzeugt (laut Datenblatt unzulaessig).
//   * Bei gleichem Fehler gewinnt das hoehere IRUN: das Datenblatt empfiehlt
//     IRUN 16..31 und die Feinabstimmung ueber GLOBALSCALER; unter IRUN 8
//     arbeitet die StealthChop2-Autoregelung nicht mehr sauber.
static tmc5160_current_scaling_t tmc_choose_current_scalers(float target_scale)
{
    tmc5160_current_scaling_t best = {
        .global_scaler_reg = 0,      // 0 = Vollausschlag (256)
        .irun_reg          = 31,
    };

    if (target_scale >= 1.0f) return best;
    if (target_scale <= 0.0f) {
        best.global_scaler_reg = (uint8_t)TMC_GLOBAL_SCALER_MIN_ALLOWED;
        best.irun_reg          = 0;
        return best;
    }

    float best_error = 1e9f;
    bool  found      = false;

    for (int irun = 31; irun >= 0; --irun) {
        float irun_f = (float)(irun + 1) / 32.0f;
        float gs_ideal = target_scale / irun_f * 256.0f;

        // Ab- und aufgerundeten Kandidaten pruefen, damit der naechstgelegene
        // gueltige Punkt sicher getroffen wird.
        for (int rounding = 0; rounding < 2; ++rounding) {
            float gs_f = (rounding == 0) ? floorf(gs_ideal) : ceilf(gs_ideal);
            if (gs_f > (float)TMC_GLOBAL_SCALER_FULL_SCALE) gs_f = (float)TMC_GLOBAL_SCALER_FULL_SCALE;
            uint32_t gs_eff = (uint32_t)gs_f;
            if (gs_eff < TMC_GLOBAL_SCALER_MIN_ALLOWED) continue;

            float realized = ((float)gs_eff / 256.0f) * irun_f;
            // Nie mehr Strom liefern als verlangt (2 % Toleranz fuer die
            // Rasterung) — zu wenig Drehmoment ist harmlos, zu viel nicht.
            if (realized > target_scale * 1.02f) continue;

            float error = fabsf(realized - target_scale);
            if (!found || error < best_error) {
                best.global_scaler_reg = (gs_eff == TMC_GLOBAL_SCALER_FULL_SCALE)
                                             ? 0U : (uint8_t)gs_eff;
                best.irun_reg          = (uint8_t)irun;
                best_error             = error;
                found                  = true;
            }
        }
    }

    if (!found) {
        // Ziel liegt unter der kleinsten darstellbaren Stufe.
        best.global_scaler_reg = (uint8_t)TMC_GLOBAL_SCALER_MIN_ALLOWED;
        best.irun_reg          = 0;
    }
    return best;
}

static uint8_t tmc_hold_current_from_percent(uint8_t irun_reg, uint8_t hold_percent)
{
    if (hold_percent == 0U) {
        return 0U;
    }

    int hold_steps = (int)lroundf(((float)(irun_reg + 1U) * (float)hold_percent) / 100.0f);
    if (hold_steps < 1)  hold_steps = 1;
    if (hold_steps > 32) hold_steps = 32;

    return (uint8_t)(hold_steps - 1);
}

// ===========================================================================
// DRV_ENN polarity verification
// ===========================================================================
static esp_err_t tmc_verify_enable_state(struct tmc5160_dev_t *dev, bool enabled)
{
    uint32_t ioin = 0;
    esp_err_t err = tmc5160_read_register(dev, TMC5160_REG_IOIN, &ioin);
    if (err != ESP_OK) {
        return err;
    }

    bool observed_enabled = !tmc_ioin_drv_enn_is_high(ioin);
    if (observed_enabled == enabled) {
        return ESP_OK;
    }

    if (!tmc_has_enable_gpio(dev)) {
        ESP_LOGE(TAG, "DRV_ENN state mismatch (no enable pin). IOIN=0x%08" PRIX32, ioin);
        return ESP_ERR_INVALID_STATE;
    }

    bool flipped_active_low = !dev->enable_active_low_runtime;
    int  flipped_level      = tmc_enable_level_for_state(enabled, flipped_active_low);
    ESP_RETURN_ON_ERROR(gpio_set_level(dev->cfg.pin_enable, flipped_level), TAG, "flip enable failed");

    err = tmc5160_read_register(dev, TMC5160_REG_IOIN, &ioin);
    if (err != ESP_OK) {
        return err;
    }

    observed_enabled = !tmc_ioin_drv_enn_is_high(ioin);
    if (observed_enabled == enabled) {
        ESP_LOGW(TAG, "DRV_ENN polarity differs from config; switched runtime polarity to active_%s",
                 flipped_active_low ? "low" : "high");
        dev->enable_active_low_runtime = flipped_active_low;
        return ESP_OK;
    }

    ESP_LOGE(TAG, "DRV_ENN did not reach requested state. IOIN=0x%08" PRIX32 " pin=%d",
             ioin, dev->cfg.pin_enable);
    return ESP_ERR_INVALID_STATE;
}

// ===========================================================================
// Lifecycle
// ===========================================================================
esp_err_t tmc5160_init(const tmc5160_config_t *cfg, tmc5160_handle_t *out_handle)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "cfg null");
    ESP_RETURN_ON_FALSE(out_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "out_handle null");

    bool has_cs_cb = (cfg->cs_ops != NULL);
    if (has_cs_cb) {
        ESP_RETURN_ON_FALSE(cfg->cs_ops->select != NULL && cfg->cs_ops->deselect != NULL,
                            ESP_ERR_INVALID_ARG, TAG, "cs_ops select/deselect required");
        ESP_RETURN_ON_FALSE(cfg->pin_cs == GPIO_NUM_NC,
                            ESP_ERR_INVALID_ARG, TAG, "pin_cs must be NC when cs_ops is used");
    } else {
        ESP_RETURN_ON_FALSE(cfg->pin_cs != GPIO_NUM_NC,
                            ESP_ERR_INVALID_ARG, TAG, "pin_cs required when cs_ops is unused");
    }

    struct tmc5160_dev_t *dev = calloc(1, sizeof(*dev));
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_NO_MEM, TAG, "alloc failed");

    dev->cfg = *cfg;
    if (dev->cfg.spi_clock_hz == 0)    dev->cfg.spi_clock_hz    = 1000000;
    dev->cfg.rsense_milliohm = tmc_rsense_or_default(dev->cfg.rsense_milliohm);
    dev->enable_active_low_runtime = dev->cfg.enable_active_low;

    // Copy cs_ops by value so the caller's pointer needs no extra lifetime.
    if (has_cs_cb) {
        dev->cs_ops_storage = *cfg->cs_ops;
        dev->has_cs_ops     = true;
    }
    dev->cfg.cs_ops = NULL;  // never read the (potentially-dangling) pointer again

    // -- Driver-enable pin -------------------------------------------------
    if (tmc_has_enable_gpio(dev)) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << dev->cfg.pin_enable),
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), fail, TAG, "enable pin cfg failed");
        int disabled_level = tmc_enable_level_for_state(false, dev->enable_active_low_runtime);
        ESP_GOTO_ON_ERROR(gpio_set_level(dev->cfg.pin_enable, disabled_level), fail, TAG, "set enable failed");
    }

    // -- SPI bus -----------------------------------------------------------
    if (!cfg->share_spi_bus) {
        spi_bus_config_t bus_cfg = {
            .mosi_io_num = dev->cfg.pin_mosi,
            .miso_io_num = dev->cfg.pin_miso,
            .sclk_io_num = dev->cfg.pin_sclk,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 5,
        };
        // TMC frames are 5 bytes; DMA-disabled keeps stack buffers valid.
        ESP_GOTO_ON_ERROR(spi_bus_initialize(dev->cfg.spi_host, &bus_cfg, SPI_DMA_DISABLED),
                          fail, TAG, "spi bus init failed");
        dev->spi_bus_owned = true;
    }

    // -- SPI device --------------------------------------------------------
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = (int)dev->cfg.spi_clock_hz,
        .mode           = 3,
        .spics_io_num   = has_cs_cb ? GPIO_NUM_NC : dev->cfg.pin_cs,
        .queue_size     = 1,
    };
    ESP_GOTO_ON_ERROR(spi_bus_add_device(dev->cfg.spi_host, &dev_cfg, &dev->spi),
                      fail, TAG, "spi add device failed");

    *out_handle = dev;
    return ESP_OK;

fail:
    tmc5160_deinit(dev);
    return ret;
}

void tmc5160_deinit(tmc5160_handle_t handle)
{
    if (handle == NULL) return;

    struct tmc5160_dev_t *dev = handle;
    if (dev->spi != NULL) {
        spi_bus_remove_device(dev->spi);
        dev->spi = NULL;
    }
    if (dev->spi_bus_owned) {
        spi_bus_free(dev->cfg.spi_host);
        dev->spi_bus_owned = false;
    }
    free(dev);
}

// ===========================================================================
// Raw register access
// ===========================================================================
esp_err_t tmc5160_write_register(tmc5160_handle_t handle, uint8_t reg, uint32_t value)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");

    struct tmc5160_dev_t *dev = handle;
    uint8_t tx[5] = {
        (uint8_t)(reg | TMC_WRITE_BIT),
        (uint8_t)((value >> 24) & 0xFF),
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >>  8) & 0xFF),
        (uint8_t)( value        & 0xFF),
    };
    uint8_t rx[5] = {0};

    esp_err_t err = tmc_spi_xfer(dev, tx, rx);
    if (err != ESP_OK) {
        return err;
    }

    // Track values that we read back from cache rather than from chip.
    switch (reg) {
    case TMC5160_REG_IHOLD_IRUN:   dev->shadow_ihold_irun   = value;                          break;
    case TMC5160_REG_TPOWERDOWN:   dev->shadow_tpowerdown   = (uint8_t)(value & 0xFF);        break;
    case TMC5160_REG_GLOBALSCALER: dev->shadow_global_scaler= (uint8_t)(value & 0xFF);        break;
    case TMC5160_REG_TCOOLTHRS:    dev->shadow_tcoolthrs    = value & 0xFFFFFU;               break;
    case TMC5160_REG_COOLCONF:     dev->shadow_coolconf     = value;                          break;
    case TMC5160_REG_GCONF:        dev->shadow_gconf        = value;                          break;
    case TMC5160_REG_TPWMTHRS:     dev->shadow_tpwmthrs     = value & 0xFFFFFU;               break;
    default: break;
    }

    return ESP_OK;
}

esp_err_t tmc5160_read_register(tmc5160_handle_t handle, uint8_t reg, uint32_t *value)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");
    ESP_RETURN_ON_FALSE(value  != NULL, ESP_ERR_INVALID_ARG, TAG, "value null");

    struct tmc5160_dev_t *dev = handle;
    uint8_t req_tx[5]   = {(uint8_t)(reg & 0x7F), 0, 0, 0, 0};
    uint8_t req_rx[5]   = {0};
    uint8_t fetch_tx[5] = {(uint8_t)(reg & 0x7F), 0, 0, 0, 0};
    uint8_t fetch_rx[5] = {0};

    // TMC5160 returns the requested data in the *next* SPI frame.
    ESP_RETURN_ON_ERROR(tmc_spi_xfer(dev, req_tx,   req_rx),   TAG, "read request failed");
    ESP_RETURN_ON_ERROR(tmc_spi_xfer(dev, fetch_tx, fetch_rx), TAG, "read fetch failed");

    *value = ((uint32_t)fetch_rx[1] << 24) |
             ((uint32_t)fetch_rx[2] << 16) |
             ((uint32_t)fetch_rx[3] <<  8) |
             ((uint32_t)fetch_rx[4]      );
    return ESP_OK;
}

// ===========================================================================
// Driver enable
// ===========================================================================
esp_err_t tmc5160_set_enabled(tmc5160_handle_t handle, bool enabled)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");

    struct tmc5160_dev_t *dev = handle;
    if (!tmc_has_enable_gpio(dev)) {
        if (!enabled) {
            return ESP_OK;
        }
        return tmc_verify_enable_state(dev, true);
    }

    int level = tmc_enable_level_for_state(enabled, dev->enable_active_low_runtime);
    ESP_RETURN_ON_ERROR(gpio_set_level(dev->cfg.pin_enable, level), TAG, "set enable failed");
    return tmc_verify_enable_state(dev, enabled);
}

// ===========================================================================
// One-shot defaults
// ===========================================================================
esp_err_t tmc5160_apply_defaults(tmc5160_handle_t handle, const tmc5160_defaults_t *defaults)
{
    ESP_RETURN_ON_FALSE(handle  != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");
    ESP_RETURN_ON_FALSE(defaults != NULL, ESP_ERR_INVALID_ARG, TAG, "defaults null");
    ESP_RETURN_ON_FALSE(defaults->amax > 0, ESP_ERR_INVALID_ARG, TAG, "amax must be > 0");

    struct tmc5160_dev_t *dev = handle;
    uint8_t  tpowerdown = defaults->tpowerdown == 0 ? TMC_DEFAULT_TPOWERDOWN : defaults->tpowerdown;
    uint8_t  ihold_delay = defaults->ihold_delay > 15 ? 15 : defaults->ihold_delay;
    uint32_t vstop      = defaults->vstop == 0 ? TMC_DEFAULT_VSTOP : defaults->vstop;

    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_GSTAT,     TMC_GSTAT_CLEAR_MASK), TAG, "GSTAT");
    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_GCONF,     defaults->gconf),      TAG, "GCONF");
    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_CHOPCONF,  defaults->chopconf),   TAG, "CHOPCONF");
    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_TPOWERDOWN, tpowerdown),          TAG, "TPOWERDOWN");
    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_VSTART,    defaults->vstart),     TAG, "VSTART");
    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_VSTOP,     vstop),                TAG, "VSTOP");
    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_TZEROWAIT, defaults->tzerowait),  TAG, "TZEROWAIT");
    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_AMAX,      defaults->amax),       TAG, "AMAX");
    ESP_RETURN_ON_ERROR(tmc5160_set_rms_current_ma(handle, defaults->run_current_ma, defaults->hold_current_percent),
                        TAG, "set run current failed");

    if (ihold_delay != TMC_DEFAULT_IHOLDDELAY) {
        uint32_t ihold_irun = 0;
        ESP_RETURN_ON_ERROR(tmc5160_read_register(handle, TMC5160_REG_IHOLD_IRUN, &ihold_irun), TAG, "read IHOLD_IRUN");
        ihold_irun &= ~(0xFUL << 16);
        ihold_irun |= ((uint32_t)ihold_delay & 0xFUL) << 16;
        ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_IHOLD_IRUN, ihold_irun), TAG, "update IHOLDDELAY");
    }

    // Velocity mode is controlled via RAMPMODE+VMAX. Keep motor stopped after init.
    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_VMAX,     0), TAG, "VMAX");
    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_RAMPMODE, 1), TAG, "RAMPMODE");

    dev->amax = defaults->amax;
    dev->motion_profile_valid = true;
    return ESP_OK;
}

// ===========================================================================
// Current scaling
// ===========================================================================
esp_err_t tmc5160_set_global_scaler(tmc5160_handle_t handle, uint8_t scaler)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");
    ESP_RETURN_ON_FALSE(scaler == 0 || scaler >= TMC_GLOBAL_SCALER_MIN_ALLOWED,
                        ESP_ERR_INVALID_ARG, TAG, "GLOBALSCALER must be 0 or 32..255");
    return tmc5160_write_register(handle, TMC5160_REG_GLOBALSCALER, scaler);
}

esp_err_t tmc5160_set_rsense_milliohm(tmc5160_handle_t handle, uint32_t rsense_milliohm)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");
    struct tmc5160_dev_t *dev = handle;
    dev->cfg.rsense_milliohm = tmc_rsense_or_default(rsense_milliohm);
    return ESP_OK;
}

float tmc5160_get_fullscale_rms_ma(tmc5160_handle_t handle)
{
    if (!handle) return 0.0f;
    struct tmc5160_dev_t *dev = handle;
    return tmc5160_fullscale_rms_ma(dev->cfg.rsense_milliohm);
}

float tmc5160_get_realized_rms_ma(tmc5160_handle_t handle)
{
    if (!handle) return 0.0f;
    return ((struct tmc5160_dev_t *)handle)->realized_rms_ma;
}

esp_err_t tmc5160_set_rms_current_ma(tmc5160_handle_t handle, uint16_t irms_ma, uint8_t hold_percent)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");
    ESP_RETURN_ON_FALSE(irms_ma > 0, ESP_ERR_INVALID_ARG, TAG, "irms_ma must be > 0");
    ESP_RETURN_ON_FALSE(hold_percent <= 100, ESP_ERR_INVALID_ARG, TAG, "hold percent invalid");

    struct tmc5160_dev_t *dev = handle;
    float fullscale_irms  = tmc_fullscale_irms_amps(dev->cfg.rsense_milliohm);
    float target_scale    = ((float)irms_ma / 1000.0f) / fullscale_irms;
    if (target_scale > 1.0f) {
        ESP_LOGW(TAG, "%u mA ueberschreitet den Vollausschlag von %.0f mA "
                      "(RSENSE %lu mOhm) — auf Vollausschlag begrenzt",
                 (unsigned)irms_ma, (double)(fullscale_irms * 1000.0f),
                 (unsigned long)dev->cfg.rsense_milliohm);
        target_scale = 1.0f;
    }

    // Kleinste darstellbare Stufe ist (32/256)*(1/32) = 1/256 des
    // Vollausschlags. Darunter kann die Suche nur noch nach OBEN abweichen —
    // das muss gesagt werden, statt es stillschweigend zu tun.
    float min_ma = fullscale_irms * 1000.0f / 256.0f;
    if ((float)irms_ma < min_ma) {
        ESP_LOGW(TAG, "%u mA liegt unter der kleinsten einstellbaren Stufe von "
                      "%.1f mA (RSENSE %lu mOhm) — es werden %.1f mA gestellt",
                 (unsigned)irms_ma, (double)min_ma,
                 (unsigned long)dev->cfg.rsense_milliohm, (double)min_ma);
    }

    tmc5160_current_scaling_t scaling = tmc_choose_current_scalers(target_scale);
    uint8_t  ihold       = tmc_hold_current_from_percent(scaling.irun_reg, hold_percent);
    uint32_t ihold_irun =
          ((uint32_t)ihold              & 0x1F)        |
        (((uint32_t)scaling.irun_reg     & 0x1F) <<  8) |
         ((uint32_t)6                            << 16);

    ESP_RETURN_ON_ERROR(tmc5160_set_global_scaler(handle, scaling.global_scaler_reg), TAG, "set GLOBALSCALER");
    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_IHOLD_IRUN, ihold_irun),
                        TAG, "set IHOLD_IRUN");

    // Tatsaechlich gestellten Wert merken: GLOBALSCALER und IRUN sind grobe
    // Stufen, der realisierte Strom weicht vom Wunsch ab. Ein UI, das nur den
    // Wunschwert anzeigt, verschweigt das.
    uint32_t gs_eff = (scaling.global_scaler_reg == 0)
                          ? TMC_GLOBAL_SCALER_FULL_SCALE : scaling.global_scaler_reg;
    dev->realized_rms_ma = fullscale_irms * 1000.0f
                         * ((float)gs_eff / 256.0f)
                         * ((float)(scaling.irun_reg + 1U) / 32.0f);
    return ESP_OK;
}

// ===========================================================================
// Motion control
// ===========================================================================
esp_err_t tmc5160_set_amax(tmc5160_handle_t handle, uint32_t amax)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");
    ESP_RETURN_ON_FALSE(amax > 0, ESP_ERR_INVALID_ARG, TAG, "amax must be > 0");

    struct tmc5160_dev_t *dev = handle;
    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_AMAX, amax), TAG, "AMAX");
    dev->amax = amax;
    dev->motion_profile_valid = true;
    return ESP_OK;
}

esp_err_t tmc5160_set_velocity_mode(tmc5160_handle_t handle, int32_t vmax)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");

    struct tmc5160_dev_t *dev = handle;
    ESP_RETURN_ON_FALSE(dev->motion_profile_valid, ESP_ERR_INVALID_STATE, TAG, "no motion profile");

    if (vmax == 0) {
        return tmc5160_stop(handle);
    }

    uint32_t abs_vmax = (vmax < 0) ? (uint32_t)(-vmax) : (uint32_t)vmax;
    ESP_RETURN_ON_ERROR(tmc5160_set_amax(handle, dev->amax), TAG, "AMAX");
    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_VMAX, abs_vmax), TAG, "VMAX");
    return tmc5160_write_register(handle, TMC5160_REG_RAMPMODE, (vmax < 0) ? 2U : 1U);
}

esp_err_t tmc5160_set_position_mode(tmc5160_handle_t handle, int32_t xtarget)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");

    struct tmc5160_dev_t *dev = handle;
    ESP_RETURN_ON_FALSE(dev->motion_profile_valid, ESP_ERR_INVALID_STATE, TAG, "no motion profile");

    // D1 and DMAX must be non-zero in position mode; reuse AMAX for symmetric ramp.
    uint32_t dval = dev->amax > 0U ? dev->amax : 100U;
    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_DMAX,     dval),             TAG, "DMAX");
    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_D1,       dval),             TAG, "D1");
    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_RAMPMODE, 0),                TAG, "RAMPMODE");
    return tmc5160_write_register(handle, TMC5160_REG_XTARGET, (uint32_t)xtarget);
}

esp_err_t tmc5160_stop(tmc5160_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");
    return tmc5160_write_register(handle, TMC5160_REG_VMAX, 0);
}

// ===========================================================================
// Feature configuration
// ===========================================================================
esp_err_t tmc5160_configure_stallguard(tmc5160_handle_t handle,
                                       int16_t  sgt,
                                       bool     sfilt,
                                       uint32_t tcoolthrs,
                                       bool     sg_stop)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");
    ESP_RETURN_ON_FALSE(sgt >= -64 && sgt <= 63, ESP_ERR_INVALID_ARG, TAG, "sgt out of range");
    ESP_RETURN_ON_FALSE(tcoolthrs <= 0xFFFFFU, ESP_ERR_INVALID_ARG, TAG, "tcoolthrs out of range");

    struct tmc5160_dev_t *dev = handle;
    // Preserve CoolStep bits [15:0]; update SGT[22:16] and sfilt[24].
    uint32_t coolconf = dev->shadow_coolconf & 0x0000FFFFUL;
    coolconf |= ((uint32_t)((uint8_t)sgt & 0x7F)) << 16;
    if (sfilt) coolconf |= 1UL << 24;

    uint32_t sw_mode = sg_stop ? (1UL << 10) : 0U;

    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_COOLCONF,  coolconf),  TAG, "COOLCONF");
    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_TCOOLTHRS, tcoolthrs), TAG, "TCOOLTHRS");
    return tmc5160_write_register(handle, TMC5160_REG_SW_MODE, sw_mode);
}

esp_err_t tmc5160_configure_coolstep(tmc5160_handle_t handle,
                                     uint8_t semin,
                                     uint8_t semax,
                                     uint8_t seup,
                                     uint8_t sedn,
                                     bool    seimin)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");
    ESP_RETURN_ON_FALSE(semin <= 15U, ESP_ERR_INVALID_ARG, TAG, "semin out of range");
    ESP_RETURN_ON_FALSE(semax <= 15U, ESP_ERR_INVALID_ARG, TAG, "semax out of range");
    ESP_RETURN_ON_FALSE(seup  <=  3U, ESP_ERR_INVALID_ARG, TAG, "seup out of range");
    ESP_RETURN_ON_FALSE(sedn  <=  3U, ESP_ERR_INVALID_ARG, TAG, "sedn out of range");

    struct tmc5160_dev_t *dev = handle;
    // Preserve SGT[22:16] and sfilt[24]; update CoolStep bits [15:0].
    uint32_t coolconf = dev->shadow_coolconf & 0x017F0000UL;
    coolconf |= (uint32_t)(semin & 0x0FU);
    coolconf |= ((uint32_t)(seup  & 0x03U)) <<  5;
    coolconf |= ((uint32_t)(semax & 0x0FU)) <<  8;
    coolconf |= ((uint32_t)(sedn  & 0x03U)) << 13;
    if (seimin) coolconf |= 1UL << 15;
    return tmc5160_write_register(handle, TMC5160_REG_COOLCONF, coolconf);
}

esp_err_t tmc5160_configure_stealthchop(tmc5160_handle_t handle, bool enable, uint32_t tpwmthrs)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");

    struct tmc5160_dev_t *dev = handle;
    uint32_t gconf = dev->shadow_gconf;
    if (enable) gconf |=  (1UL << 2);   // en_pwm_mode
    else        gconf &= ~(1UL << 2);

    ESP_RETURN_ON_ERROR(tmc5160_write_register(handle, TMC5160_REG_GCONF,    gconf),                 TAG, "GCONF");
    return tmc5160_write_register(handle, TMC5160_REG_TPWMTHRS, tpwmthrs & 0xFFFFFU);
}

// ===========================================================================
// Diagnostics
// ===========================================================================
esp_err_t tmc5160_collect_diag(tmc5160_handle_t handle, tmc5160_diag_t *diag)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle null");
    ESP_RETURN_ON_FALSE(diag   != NULL, ESP_ERR_INVALID_ARG, TAG, "diag null");

    struct tmc5160_dev_t *dev = handle;
    ESP_RETURN_ON_ERROR(tmc5160_read_register(handle, TMC5160_REG_IOIN,       &diag->ioin),       TAG, "IOIN");
    ESP_RETURN_ON_ERROR(tmc5160_read_register(handle, TMC5160_REG_GSTAT,      &diag->gstat),      TAG, "GSTAT");
    ESP_RETURN_ON_ERROR(tmc5160_read_register(handle, TMC5160_REG_GCONF,      &diag->gconf),      TAG, "GCONF");
    ESP_RETURN_ON_ERROR(tmc5160_read_register(handle, TMC5160_REG_CHOPCONF,   &diag->chopconf),   TAG, "CHOPCONF");
    ESP_RETURN_ON_ERROR(tmc5160_read_register(handle, TMC5160_REG_COOLCONF,   &diag->coolconf),   TAG, "COOLCONF");
    ESP_RETURN_ON_ERROR(tmc5160_read_register(handle, TMC5160_REG_SW_MODE,    &diag->sw_mode),    TAG, "SW_MODE");
    ESP_RETURN_ON_ERROR(tmc5160_read_register(handle, TMC5160_REG_RAMP_STAT,  &diag->ramp_stat),  TAG, "RAMP_STAT");
    ESP_RETURN_ON_ERROR(tmc5160_read_register(handle, TMC5160_REG_DRV_STATUS, &diag->drv_status), TAG, "DRV_STATUS");
    ESP_RETURN_ON_ERROR(tmc5160_read_register(handle, TMC5160_REG_PWM_SCALE,  &diag->pwm_scale),  TAG, "PWM_SCALE");

    uint32_t raw_xactual = 0, raw_vactual = 0, raw_tstep = 0;
    ESP_RETURN_ON_ERROR(tmc5160_read_register(handle, TMC5160_REG_XACTUAL, &raw_xactual), TAG, "XACTUAL");
    ESP_RETURN_ON_ERROR(tmc5160_read_register(handle, TMC5160_REG_VACTUAL, &raw_vactual), TAG, "VACTUAL");
    ESP_RETURN_ON_ERROR(tmc5160_read_register(handle, TMC5160_REG_TSTEP,   &raw_tstep),   TAG, "TSTEP");

    diag->xactual = (int32_t)raw_xactual;
    if ((raw_vactual & 0x00800000UL) != 0U) {
        raw_vactual |= 0xFF000000UL;        // sign-extend 24-bit VACTUAL to 32-bit
    }
    diag->vactual       = (int32_t)raw_vactual;
    diag->tstep         = raw_tstep & 0xFFFFFU;
    diag->ihold_irun    = dev->shadow_ihold_irun;
    diag->tpowerdown    = dev->shadow_tpowerdown;
    diag->global_scaler = dev->shadow_global_scaler;
    diag->tcoolthrs     = dev->shadow_tcoolthrs;
    diag->tpwmthrs      = dev->shadow_tpwmthrs;
    return ESP_OK;
}
