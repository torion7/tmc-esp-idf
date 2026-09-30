// Host-Tests fuer den TMC5160-Registertreiber. tmc5160.c wird UNVERAENDERT
// gegen die IDF-Stubs (stubs/) und fake_spi.c gebaut, das jeden
// Registerzugriff aufzeichnet — geprueft wird, was auf dem Bus landet.
//
// Abgedeckter Fehler:
//   tmc5160_regs.h hatte D1 = 0x28 und DMAX = 0x29. Laut TMC5160-Datenblatt
//   (Ramp Generator Registers) ist DMAX = 0x28, D1 = 0x2A, 0x29 unbelegt.
//   set_position_mode() schrieb dadurch D1 nie (D1 = 0 ist im Positions-
//   modus unzulaessig) und DMAX auf die unbelegte Adresse.
//
// Erwartungswerte stehen als Datenblatt-Adressen im Test, nicht als Namen
// aus tmc5160_regs.h — sonst wuerde der Test den Fehler mitmachen.

#include <stdio.h>

#include "fake_spi.h"
#include "test_util.h"
#include "tmc5160.h"
#include "tmc5160_regs.h"
#include "tmc5240_regs.h"

static tmc5160_handle_t neues_geraet(uint32_t rsense_milliohm)
{
    fake_reset();
    tmc5160_config_t cfg = {
        .spi_host        = SPI2_HOST,
        .pin_mosi        = GPIO_NUM_1,
        .pin_miso        = GPIO_NUM_2,
        .pin_sclk        = GPIO_NUM_3,
        .pin_cs          = GPIO_NUM_4,
        .pin_enable      = GPIO_NUM_NC,
        .rsense_milliohm = rsense_milliohm,
    };
    tmc5160_handle_t h = NULL;
    CHECK(tmc5160_init(&cfg, &h) == ESP_OK);
    CHECK(h != NULL);
    return h;
}

static void test_register_adressen(void)
{
    // Rampengenerator laut TMC5160-Datenblatt: 0x20..0x2D, 0x29 unbelegt
    CHECK_EQ_U32(TMC5160_REG_RAMPMODE,  0x20);
    CHECK_EQ_U32(TMC5160_REG_XACTUAL,   0x21);
    CHECK_EQ_U32(TMC5160_REG_VACTUAL,   0x22);
    CHECK_EQ_U32(TMC5160_REG_VSTART,    0x23);
    CHECK_EQ_U32(TMC5160_REG_A1,        0x24);
    CHECK_EQ_U32(TMC5160_REG_V1,        0x25);
    CHECK_EQ_U32(TMC5160_REG_AMAX,      0x26);
    CHECK_EQ_U32(TMC5160_REG_VMAX,      0x27);
    CHECK_EQ_U32(TMC5160_REG_DMAX,      0x28);
    CHECK_EQ_U32(TMC5160_REG_D1,        0x2A);
    CHECK_EQ_U32(TMC5160_REG_VSTOP,     0x2B);
    CHECK_EQ_U32(TMC5160_REG_TZEROWAIT, 0x2C);
    CHECK_EQ_U32(TMC5160_REG_XTARGET,   0x2D);

    // Kein Name auf der unbelegten 0x29, keine zwei Namen auf einer Adresse.
    const unsigned alle[] = {
        TMC5160_REG_GCONF,     TMC5160_REG_GSTAT,        TMC5160_REG_IOIN,
        TMC5160_REG_SHORT_CONF, TMC5160_REG_DRV_CONF,    TMC5160_REG_GLOBALSCALER,
        TMC5160_REG_IHOLD_IRUN, TMC5160_REG_TPOWERDOWN,  TMC5160_REG_TSTEP,
        TMC5160_REG_TPWMTHRS,  TMC5160_REG_TCOOLTHRS,    TMC5160_REG_RAMPMODE,
        TMC5160_REG_XACTUAL,   TMC5160_REG_VACTUAL,      TMC5160_REG_VSTART,
        TMC5160_REG_A1,        TMC5160_REG_V1,           TMC5160_REG_AMAX,
        TMC5160_REG_VMAX,      TMC5160_REG_DMAX,         TMC5160_REG_D1,
        TMC5160_REG_VSTOP,     TMC5160_REG_TZEROWAIT,    TMC5160_REG_XTARGET,
        TMC5160_REG_SW_MODE,   TMC5160_REG_RAMP_STAT,    TMC5160_REG_CHOPCONF,
        TMC5160_REG_COOLCONF,  TMC5160_REG_DRV_STATUS,   TMC5160_REG_PWMCONF,
        TMC5160_REG_PWM_SCALE,
    };
    const size_t n = sizeof(alle) / sizeof(alle[0]);
    int auf_0x29 = 0, doppelt = 0;
    for (size_t i = 0; i < n; ++i) {
        if (alle[i] == 0x29) auf_0x29++;
        for (size_t j = i + 1; j < n; ++j) {
            if (alle[i] == alle[j]) doppelt++;
        }
    }
    CHECK_EQ_I32(auf_0x29, 0);
    CHECK_EQ_I32(doppelt, 0);

    // Gemeinsame Register liegen beim TMC5240 auf denselben Adressen; dort
    // ist 0x29 TVMAX (nur TMC5240).
    CHECK_EQ_U32(TMC5160_REG_DMAX,         TMC5240_REG_DMAX);
    CHECK_EQ_U32(TMC5160_REG_D1,           TMC5240_REG_D1);
    CHECK_EQ_U32(TMC5240_REG_TVMAX,        0x29);
    CHECK_EQ_U32(TMC5160_REG_RAMPMODE,     TMC5240_REG_RAMPMODE);
    CHECK_EQ_U32(TMC5160_REG_XACTUAL,      TMC5240_REG_XACTUAL);
    CHECK_EQ_U32(TMC5160_REG_VACTUAL,      TMC5240_REG_VACTUAL);
    CHECK_EQ_U32(TMC5160_REG_VSTART,       TMC5240_REG_VSTART);
    CHECK_EQ_U32(TMC5160_REG_A1,           TMC5240_REG_A1);
    CHECK_EQ_U32(TMC5160_REG_V1,           TMC5240_REG_V1);
    CHECK_EQ_U32(TMC5160_REG_AMAX,         TMC5240_REG_AMAX);
    CHECK_EQ_U32(TMC5160_REG_VMAX,         TMC5240_REG_VMAX);
    CHECK_EQ_U32(TMC5160_REG_VSTOP,        TMC5240_REG_VSTOP);
    CHECK_EQ_U32(TMC5160_REG_TZEROWAIT,    TMC5240_REG_TZEROWAIT);
    CHECK_EQ_U32(TMC5160_REG_XTARGET,      TMC5240_REG_XTARGET);
    CHECK_EQ_U32(TMC5160_REG_GCONF,        TMC5240_REG_GCONF);
    CHECK_EQ_U32(TMC5160_REG_GSTAT,        TMC5240_REG_GSTAT);
    CHECK_EQ_U32(TMC5160_REG_IOIN,         TMC5240_REG_IOIN);
    CHECK_EQ_U32(TMC5160_REG_GLOBALSCALER, TMC5240_REG_GLOBALSCALER);
    CHECK_EQ_U32(TMC5160_REG_IHOLD_IRUN,   TMC5240_REG_IHOLD_IRUN);
    CHECK_EQ_U32(TMC5160_REG_TPOWERDOWN,   TMC5240_REG_TPOWERDOWN);
    CHECK_EQ_U32(TMC5160_REG_TSTEP,        TMC5240_REG_TSTEP);
    CHECK_EQ_U32(TMC5160_REG_TPWMTHRS,     TMC5240_REG_TPWMTHRS);
    CHECK_EQ_U32(TMC5160_REG_TCOOLTHRS,    TMC5240_REG_TCOOLTHRS);
    CHECK_EQ_U32(TMC5160_REG_SW_MODE,      TMC5240_REG_SW_MODE);
    CHECK_EQ_U32(TMC5160_REG_RAMP_STAT,    TMC5240_REG_RAMP_STAT);
    CHECK_EQ_U32(TMC5160_REG_CHOPCONF,     TMC5240_REG_CHOPCONF);
    CHECK_EQ_U32(TMC5160_REG_COOLCONF,     TMC5240_REG_COOLCONF);
    CHECK_EQ_U32(TMC5160_REG_DRV_STATUS,   TMC5240_REG_DRV_STATUS);
    CHECK_EQ_U32(TMC5160_REG_PWMCONF,      TMC5240_REG_PWMCONF);
    CHECK_EQ_U32(TMC5160_REG_PWM_SCALE,    TMC5240_REG_PWM_SCALE);
}

static void test_positionsmodus_schreibt_dmax_und_d1(void)
{
    tmc5160_handle_t h = neues_geraet(75);
    CHECK(tmc5160_set_amax(h, 5000) == ESP_OK);
    fake_clear_log();

    CHECK(tmc5160_set_position_mode(h, 123456) == ESP_OK);

    CHECK_EQ_I32(fake_writes_to(0x29), 0);         // unbelegt: nie beschreiben
    CHECK_EQ_I32(fake_writes_to(0x28), 1);         // DMAX
    CHECK_EQ_U32(fake_reg(0x28), 5000);
    CHECK_EQ_I32(fake_writes_to(0x2A), 1);         // D1: im Positionsmodus != 0
    CHECK_EQ_U32(fake_reg(0x2A), 5000);
    CHECK_EQ_U32(fake_reg(0x20), 0);               // RAMPMODE: Positionsmodus
    CHECK_EQ_U32(fake_reg(0x2D), 123456);          // XTARGET

    // Die Rampe steht, bevor XTARGET als letzter Zugriff die Fahrt ausloest.
    const int letzter = (int)fake_write_count() - 1;
    CHECK(letzter >= 0 && fake_write_at((size_t)letzter)->reg == 0x2D);
    CHECK(fake_last_write_index(0x28) < letzter);
    CHECK(fake_last_write_index(0x2A) < letzter);
    CHECK(fake_last_write_index(0x20) < letzter);

    // Negatives Ziel im Zweierkomplement
    CHECK(tmc5160_set_position_mode(h, -1000) == ESP_OK);
    CHECK_EQ_U32(fake_reg(0x2D), 0xFFFFFC18UL);
    CHECK_EQ_I32(fake_writes_to(0x29), 0);

    tmc5160_deinit(h);
}

int main(void)
{
    test_register_adressen();
    test_positionsmodus_schreibt_dmax_und_d1();
    return test_report("tmc5160");
}
