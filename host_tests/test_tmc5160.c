// Host-Tests fuer den TMC5160-Registertreiber. tmc5160.c wird UNVERAENDERT
// gegen die IDF-Stubs (stubs/) und fake_spi.c gebaut, das jeden
// Registerzugriff aufzeichnet — geprueft wird, was auf dem Bus landet.
//
// Abgedeckte Fehler:
//  1. tmc5160_regs.h hatte D1 = 0x28 und DMAX = 0x29. Laut TMC5160-Datenblatt
//     (Ramp Generator Registers) ist DMAX = 0x28, D1 = 0x2A, 0x29 unbelegt.
//     set_position_mode() schrieb dadurch D1 nie (D1 = 0 ist im Positions-
//     modus unzulaessig) und DMAX auf die unbelegte Adresse.
//     Erwartungswerte stehen als Datenblatt-Adressen im Test, nicht als Namen
//     aus tmc5160_regs.h — sonst wuerde der Test den Fehler mitmachen.
//  2. Die Stromsuche verlangte GLOBALSCALER >= 128 und IRUN >= 16: kleinste
//     Stufe (128/256)*(17/32) = 0,2656 des Vollausschlags, kleinere Stroeme
//     wurden darauf ANGEHOBEN (75 mOhm: 500 mA verlangt, 813,9 mA gestellt).
//
// Strom-Erwartungswerte von Hand gerechnet:
//   IFS_rms = 0,325 V / RSENSE / sqrt(2)          (75 mOhm -> 3064,13 mA)
//   I_rms   = IFS_rms * GS/256 * (IRUN+1)/32      (GLOBALSCALER 0 = 256)
// bzw. in Einheiten von 1/8192 des Vollausschlags: P = GS * (IRUN+1).

#include <math.h>
#include <stdio.h>

#include "fake_spi.h"
#include "test_util.h"
#include "tmc5160.h"
#include "tmc5160_regs.h"
#include "tmc5240_regs.h"

#define IHOLD_VON(v)      ((v) & 0x1FUL)
#define IRUN_VON(v)       (((v) >> 8) & 0x1FUL)
#define IHOLDDELAY_VON(v) (((v) >> 16) & 0xFUL)

// Vollausschlag nach Datenblatt, unabhaengig vom Treiber gerechnet: mV / Ohm = mA
static double ifs_ma(uint32_t rsense_milliohm)
{
    return 325.0 / (rsense_milliohm / 1000.0) / sqrt(2.0);
}

// Strom aus den zuletzt geschriebenen Registern (nicht aus dem Treiber)
static double strom_aus_registern_ma(uint32_t rsense_milliohm)
{
    uint32_t gs   = fake_reg(TMC5160_REG_GLOBALSCALER);
    uint32_t irun = IRUN_VON(fake_reg(TMC5160_REG_IHOLD_IRUN));
    return ifs_ma(rsense_milliohm) * (gs == 0 ? 256 : gs) / 256.0 * (irun + 1) / 32.0;
}

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

static void test_strom_referenzwerte(void)
{
    tmc5160_handle_t h = neues_geraet(75);
    CHECK_NEAR(tmc5160_fullscale_rms_ma(75), 3064.13, 0.01);
    CHECK_NEAR(tmc5160_get_fullscale_rms_ma(h), 3064.13, 0.01);
    CHECK_NEAR(tmc5160_get_realized_rms_ma(h), 0.0, 1e-9);     // noch nichts gestellt

    // Der gemeldete Fall. Alt: GS 128, IRUN 16 -> 813,9 mA.
    // Neu: Ziel P = 500 / 3064,13 * 8192 = 1336,75; naechstes bildbares P ist
    // 1337 = 191 * 7 -> GS 191, IRUN 6 -> 500,09 mA.
    CHECK(tmc5160_set_rms_current_ma(h, 500, 50) == ESP_OK);
    uint32_t ihr = fake_reg(TMC5160_REG_IHOLD_IRUN);
    CHECK_EQ_U32(fake_reg(TMC5160_REG_GLOBALSCALER), 191);
    CHECK_EQ_U32(IRUN_VON(ihr), 6);
    CHECK_EQ_U32(IHOLD_VON(ihr), 3);                 // round(7 * 50 %) = 4 Stufen
    CHECK_EQ_U32(IHOLDDELAY_VON(ihr), 6);
    CHECK_NEAR(strom_aus_registern_ma(75), 500.09, 0.01);
    CHECK_NEAR(tmc5160_get_realized_rms_ma(h), strom_aus_registern_ma(75), 0.01);
    CHECK(tmc5160_get_realized_rms_ma(h) <= 500.0 * 1.02);

    // Gleichstand: 766 mA -> P = 2047,91; bestes P = 2048 = 64*32 = 128*16 =
    // 256*8 (je 766,03 mA). Bei gleichem Strom gewinnt das hoechste IRUN.
    // Alt: wieder 813,9 mA.
    CHECK(tmc5160_set_rms_current_ma(h, 766, 0) == ESP_OK);
    ihr = fake_reg(TMC5160_REG_IHOLD_IRUN);
    CHECK_EQ_U32(fake_reg(TMC5160_REG_GLOBALSCALER), 64);
    CHECK_EQ_U32(IRUN_VON(ihr), 31);
    CHECK_EQ_U32(IHOLD_VON(ihr), 0);                 // Haltestrom 0 %
    CHECK_NEAR(tmc5160_get_realized_rms_ma(h), 766.03, 0.01);

    // Knapp unter Vollausschlag: P = 8192 -> GLOBALSCALER 256 wird als 0 geschrieben.
    CHECK(tmc5160_set_rms_current_ma(h, 3064, 100) == ESP_OK);
    CHECK_EQ_U32(fake_reg(TMC5160_REG_GLOBALSCALER), 0);
    CHECK_EQ_U32(IRUN_VON(fake_reg(TMC5160_REG_IHOLD_IRUN)), 31);
    CHECK_EQ_I32(fake_warnings(), 0);

    // Ueber Vollausschlag: begrenzt, mit Warnung.
    CHECK(tmc5160_set_rms_current_ma(h, 4000, 50) == ESP_OK);
    CHECK_EQ_I32(fake_warnings(), 1);
    CHECK_EQ_U32(fake_reg(TMC5160_REG_GLOBALSCALER), 0);
    CHECK_EQ_U32(IRUN_VON(fake_reg(TMC5160_REG_IHOLD_IRUN)), 31);
    CHECK_NEAR(tmc5160_get_realized_rms_ma(h), 3064.13, 0.01);

    // Unter der kleinsten Stufe (3064,13 / 256 = 11,97 mA): GS 32, IRUN 0, Warnung.
    CHECK(tmc5160_set_rms_current_ma(h, 5, 50) == ESP_OK);
    CHECK_EQ_I32(fake_warnings(), 2);
    CHECK_EQ_U32(fake_reg(TMC5160_REG_GLOBALSCALER), 32);
    CHECK_EQ_U32(IRUN_VON(fake_reg(TMC5160_REG_IHOLD_IRUN)), 0);
    CHECK_NEAR(tmc5160_get_realized_rms_ma(h), 11.97, 0.01);

    // Knapp ueber der kleinsten Stufe: dasselbe Paar, aber keine Warnung.
    CHECK(tmc5160_set_rms_current_ma(h, 12, 50) == ESP_OK);
    CHECK_EQ_I32(fake_warnings(), 2);
    CHECK_EQ_U32(fake_reg(TMC5160_REG_GLOBALSCALER), 32);
    CHECK_EQ_U32(IRUN_VON(fake_reg(TMC5160_REG_IHOLD_IRUN)), 0);

    tmc5160_deinit(h);
}

// Unabhaengige Referenz fuer die Stromsuche: alle zulaessigen Paare
// (GS 32..256, IRUN 0..31) als Produkt P = GS * (IRUN+1).
// max_irun1[P] = hoechstes IRUN+1, mit dem sich P bilden laesst (0 = gar nicht).
static int max_irun1[8193];

static void referenz_aufbauen(void)
{
    for (int gs = 32; gs <= 256; ++gs) {
        for (int i1 = 1; i1 <= 32; ++i1) {
            if (i1 > max_irun1[gs * i1]) max_irun1[gs * i1] = i1;
        }
    }
}

// Bestes P fuer das Ziel x (in 1/8192): kleinster Abstand, hoechstens 2 %
// darueber; ab Vollausschlag 8192, unter der kleinsten Stufe 32.
static int referenz_produkt(double x)
{
    if (x >= 8192.0) return 8192;
    int    best      = 0;
    double best_abst = 1e30;
    for (int p = 32; p <= 8192; ++p) {
        if (max_irun1[p] == 0 || p > x * 1.02) continue;
        double abst = fabs(p - x);
        if (abst < best_abst) {
            best      = p;
            best_abst = abst;
        }
    }
    return best ? best : 32;
}

// Jeder ganzzahlige Strom von 1 mA bis 105 % Vollausschlag fuer mehrere
// Sense-Widerstaende: so gut wie die Referenz, nie mehr als +2 %, nie
// GLOBALSCALER 1..31, bei gleichem Strom das hoechste IRUN, realized_rms_ma
// passt zu den Registern, Warnung genau bei Ueber-/Unterschreiten.
// Die alte Suche verletzt "+2 %" hier bei rund einem Viertel aller Stroeme.
static void test_strom_sweep(void)
{
    referenz_aufbauen();
    const uint32_t rsense[] = { 22, 50, 75, 110, 220 };

    for (size_t k = 0; k < sizeof(rsense) / sizeof(rsense[0]); ++k) {
        const double     ifs = ifs_ma(rsense[k]);
        tmc5160_handle_t h   = neues_geraet(rsense[k]);
        int abweichungen = 0;

        for (uint32_t ma = 1; ma <= (uint32_t)(ifs * 1.05); ++ma) {
            const int w0 = fake_warnings();
            fake_clear_log();
            const esp_err_t err = tmc5160_set_rms_current_ma(h, (uint16_t)ma, 0);

            const uint32_t gs     = fake_reg(TMC5160_REG_GLOBALSCALER);
            const uint32_t irun   = IRUN_VON(fake_reg(TMC5160_REG_IHOLD_IRUN));
            const int      p      = (int)((gs == 0 ? 256 : gs) * (irun + 1));
            const double   x      = fmin(ma / ifs * 8192.0, 8192.0);
            const int      p_ref  = referenz_produkt(x);
            const int      w_soll = (ma > ifs) + (ma < ifs / 256.0);

            const bool ok = err == ESP_OK
                && fake_writes_to(TMC5160_REG_GLOBALSCALER) == 1
                && fake_writes_to(TMC5160_REG_IHOLD_IRUN) == 1
                && (gs == 0 || gs >= 32)
                && fabs(p - x) <= fabs(p_ref - x) + 0.01
                && (x < 32.0 ? (p == 32 && irun == 0) : p <= x * 1.02 + 0.01)
                && (int)irun + 1 == max_irun1[p]
                && fabs(tmc5160_get_realized_rms_ma(h) - ifs * p / 8192.0) <= 1e-5 * ifs
                && fake_warnings() - w0 == w_soll;

            if (!ok && abweichungen++ < 5) {
                fprintf(stderr, "  RSENSE %lu mOhm, %lu mA: GS %lu IRUN %lu (P %d), "
                                "Referenz P %d, Warnungen %d statt %d\n",
                        (unsigned long)rsense[k], (unsigned long)ma, (unsigned long)gs,
                        (unsigned long)irun, p, p_ref, fake_warnings() - w0, w_soll);
            }
        }
        CHECK_EQ_I32(abweichungen, 0);
        CHECK_EQ_I32(fake_errors(), 0);
        tmc5160_deinit(h);
    }
}

static void test_rsense_api(void)
{
    tmc5160_handle_t h = neues_geraet(0);                     // 0 -> Vorgabe 75 mOhm
    CHECK_NEAR(tmc5160_get_fullscale_rms_ma(h), 3064.13, 0.01);
    CHECK_NEAR(tmc5160_fullscale_rms_ma(0), 3064.13, 0.01);

    // 50 mOhm: IFS = 325 / 0,05 / sqrt(2) = 4596,19 mA. 500 mA -> P = 891,18;
    // 891 = 33 * 27 -> GS 33, IRUN 26 -> 499,90 mA.
    CHECK(tmc5160_set_rsense_milliohm(h, 50) == ESP_OK);
    CHECK_NEAR(tmc5160_get_fullscale_rms_ma(h), 4596.19, 0.01);
    CHECK(tmc5160_set_rms_current_ma(h, 500, 0) == ESP_OK);
    CHECK_EQ_U32(fake_reg(TMC5160_REG_GLOBALSCALER), 33);
    CHECK_EQ_U32(IRUN_VON(fake_reg(TMC5160_REG_IHOLD_IRUN)), 26);
    CHECK_NEAR(tmc5160_get_realized_rms_ma(h), 499.90, 0.01);
    CHECK_NEAR(strom_aus_registern_ma(50), 499.90, 0.01);

    CHECK(tmc5160_set_rsense_milliohm(h, 0) == ESP_OK);      // zurueck auf 75 mOhm
    CHECK_NEAR(tmc5160_get_fullscale_rms_ma(h), 3064.13, 0.01);
    // realized bleibt der Wert der letzten Stromeinstellung
    CHECK_NEAR(tmc5160_get_realized_rms_ma(h), 499.90, 0.01);

    CHECK(tmc5160_set_rsense_milliohm(NULL, 75) == ESP_ERR_INVALID_ARG);
    CHECK_NEAR(tmc5160_get_fullscale_rms_ma(NULL), 0.0, 1e-9);
    CHECK_NEAR(tmc5160_get_realized_rms_ma(NULL), 0.0, 1e-9);

    tmc5160_deinit(h);
}

int main(void)
{
    test_register_adressen();
    test_positionsmodus_schreibt_dmax_und_d1();
    test_strom_referenzwerte();
    test_strom_sweep();
    test_rsense_api();
    return test_report("tmc5160");
}
