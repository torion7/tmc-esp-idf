// Referenzvektoren fuer tmc_motion_decode_diag (Bitfeld-Dekodierung der
// TMC5240/5160-Register). Die Erwartungswerte sind UNABHAENGIG vom Code aus
// den Datenblatt-Layouts abgeleitet — der Test schuetzt vor stillen
// Verschiebungen bei Register-Refactorings.

#include <math.h>
#include <stdio.h>

#include "tmc_motion.h"
#include "test_util.h"

static tmc_motion_diag_t leer(void)
{
    tmc_motion_diag_t d;
    memset(&d, 0, sizeof(d));
    return d;
}

static void test_drv_status_bits(void)
{
    tmc_motion_diag_t d = leer();
    tmc_motion_decoded_diag_t o;

    // stst(31) | olb(30) | ola(29) | s2gb(28) | s2ga(27) | otpw(26) |
    // ot(25) | stallGuard(24) | fs_active(15) | stealth(14) | s2vsb(13) |
    // s2vsa(12)
    d.drv_status = (1UL << 31) | (1UL << 26) | (1UL << 24) | (1UL << 14);
    tmc_motion_decode_diag(&d, &o);
    CHECK(o.stst);
    CHECK(o.otpw);
    CHECK(!o.ot);
    CHECK(o.stall);
    CHECK(o.stealth);
    CHECK(!o.ola && !o.olb && !o.s2ga && !o.s2gb && !o.s2vsa && !o.s2vsb);
    CHECK(!o.fsactive);

    d.drv_status = (1UL << 30) | (1UL << 29) | (1UL << 28) | (1UL << 27) |
                   (1UL << 25) | (1UL << 15) | (1UL << 13) | (1UL << 12);
    tmc_motion_decode_diag(&d, &o);
    CHECK(o.olb && o.ola && o.s2gb && o.s2ga && o.ot && o.fsactive &&
          o.s2vsb && o.s2vsa);
    CHECK(!o.stst && !o.otpw && !o.stall && !o.stealth);
}

static void test_cs_actual_und_sg_result(void)
{
    tmc_motion_diag_t d = leer();
    tmc_motion_decoded_diag_t o;

    // CS_ACTUAL = DRV_STATUS[20:16], SG_RESULT = DRV_STATUS[9:0]
    d.drv_status = (0x1FUL << 16) | 0x3FFUL;
    tmc_motion_decode_diag(&d, &o);
    CHECK_EQ_U32(o.cs_actual, 31);
    CHECK_EQ_U32(o.sg_result, 1023);

    d.drv_status = (0x0AUL << 16) | 0x155UL;
    tmc_motion_decode_diag(&d, &o);
    CHECK_EQ_U32(o.cs_actual, 10);
    CHECK_EQ_U32(o.sg_result, 0x155);

    d.sg4_result = 0xFFFFFFFFUL;      // nur [9:0] gueltig
    tmc_motion_decode_diag(&d, &o);
    CHECK_EQ_U32(o.sg4_result, 1023);
}

static void test_ramp_stat_bits(void)
{
    tmc_motion_diag_t d = leer();
    tmc_motion_decoded_diag_t o;

    // status_sg(13) | position_reached(9) | velocity_reached(8) |
    // event_stop_sg(6)
    d.ramp_stat = (1UL << 13) | (1UL << 6);
    tmc_motion_decode_diag(&d, &o);
    CHECK(o.status_sg);
    CHECK(o.event_stop_sg);
    CHECK(!o.position_reached && !o.velocity_reached);

    d.ramp_stat = (1UL << 9) | (1UL << 8);
    tmc_motion_decode_diag(&d, &o);
    CHECK(o.position_reached && o.velocity_reached);
    CHECK(!o.status_sg && !o.event_stop_sg);
}

static void test_ioin_bits(void)
{
    tmc_motion_diag_t d = leer();
    tmc_motion_decoded_diag_t o;

    d.ioin = (1UL << 4) | (1UL << 6) | (1UL << 7);
    tmc_motion_decode_diag(&d, &o);
    CHECK(o.drv_enn && o.sd_mode && o.swcomp);

    d.ioin = 0;
    tmc_motion_decode_diag(&d, &o);
    CHECK(!o.drv_enn && !o.sd_mode && !o.swcomp);
}

static void test_pwm_scale(void)
{
    tmc_motion_diag_t d = leer();
    tmc_motion_decoded_diag_t o;

    // PWM_SCALE_SUM = [9:0], PWM_SCALE_AUTO = [24:16] als 9-bit signed.
    d.pwm_scale = (16UL << 16) | 512UL;          // auto = +16, sum = 512
    tmc_motion_decode_diag(&d, &o);
    CHECK_EQ_U32(o.pwm_scale_sum, 512);
    CHECK_EQ_I32(o.pwm_scale_auto, 16);
    CHECK_EQ_U32(o.pwm_scale_pct, 512 * 100 / 1023);   // 50

    d.pwm_scale = (0x1F0UL << 16) | 100UL;       // 0x1F0 = -16 (Zweierkompl.)
    tmc_motion_decode_diag(&d, &o);
    CHECK_EQ_I32(o.pwm_scale_auto, -16);
    CHECK_EQ_U32(o.pwm_scale_sum, 100);

    d.pwm_scale = (0x100UL << 16);               // -256 = Minimalwert
    tmc_motion_decode_diag(&d, &o);
    CHECK_EQ_I32(o.pwm_scale_auto, -256);

    d.pwm_scale = 1023UL;                        // Vollaussteuerung
    tmc_motion_decode_diag(&d, &o);
    CHECK_EQ_U32(o.pwm_scale_pct, 100);
}

static void test_chip_temperatur(void)
{
    tmc_motion_diag_t d = leer();
    tmc_motion_decoded_diag_t o;

    // Datenblatt: T = (ADC_TEMP - 2038) / 7.7
    d.adc_temp = 2038;                            // exakt 0 degC
    tmc_motion_decode_diag(&d, &o);
    CHECK(fabsf(o.chip_temp_c - 0.0f) < 0.01f);

    d.adc_temp = 2038 + 77;                       // -> 10.0 degC
    tmc_motion_decode_diag(&d, &o);
    CHECK(fabsf(o.chip_temp_c - 10.0f) < 0.01f);

    d.adc_temp = 2038 + 770;                      // -> 100.0 degC
    tmc_motion_decode_diag(&d, &o);
    CHECK(fabsf(o.chip_temp_c - 100.0f) < 0.01f);

    d.adc_temp = 2038 - 154;                      // -> -20.0 degC
    tmc_motion_decode_diag(&d, &o);
    CHECK(fabsf(o.chip_temp_c - (-20.0f)) < 0.01f);

    d.adc_temp = 0;                               // kein Sensorwert -> 0.0
    tmc_motion_decode_diag(&d, &o);
    CHECK(o.chip_temp_c == 0.0f);
}

int main(void)
{
    test_drv_status_bits();
    test_cs_actual_und_sg_result();
    test_ramp_stat_bits();
    test_ioin_bits();
    test_pwm_scale();
    test_chip_temperatur();
    return test_report("decode_diag");
}
