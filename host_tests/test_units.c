// Referenzvektoren fuer tmc_motion_units (Datenblatt-Umrechnungen
// TMC5160/5240):  v[usteps/s] = VMAX * fCLK / 2^24,
//                 a[usteps/s^2] = AMAX * fCLK^2 / 2^41.
// Erwartungswerte von Hand gerechnet (12 MHz, 200 * 256 usteps/rev).

#include <stdio.h>

#include "tmc_motion_units.h"
#include "test_util.h"

#define USTEPS_REV 51200u        // 200 Vollschritte * 256 Mikroschritte
#define FCLK       12000000u     // interner TMC-Oszillator

static void test_rpm_vmax(void)
{
    // 60 rpm = 1 U/s = 51200 usteps/s -> VMAX = 51200 * 2^24 / 12e6 = 71582,8
    CHECK_EQ_U32(tmc_motion_units_rpm_to_vmax(60.0, USTEPS_REV, FCLK), 71583);
    // Ruecktransformation trifft den Ausgangswert (Rundungsfehler < 0,01 rpm)
    CHECK_NEAR(tmc_motion_units_vmax_to_rpm(71583, USTEPS_REV, FCLK), 60.0, 0.01);

    CHECK_EQ_U32(tmc_motion_units_rpm_to_vmax(0.0, USTEPS_REV, FCLK), 0);
    // Vorzeichen wird verworfen (VMAX ist ein Betrag)
    CHECK_EQ_U32(tmc_motion_units_rpm_to_vmax(-60.0, USTEPS_REV, FCLK), 71583);
    // Division durch 0 abgesichert
    CHECK_NEAR(tmc_motion_units_vmax_to_rpm(1000, 0, FCLK), 0.0, 1e-12);
    CHECK_NEAR(tmc_motion_units_vmax_to_rpm(1000, USTEPS_REV, 0), 0.0, 1e-12);
}

static void test_rps_vmax(void)
{
    // 1 rev/s entspricht 60 rpm
    CHECK_EQ_U32(tmc_motion_units_rps_to_vmax(1.0, USTEPS_REV, FCLK),
                 tmc_motion_units_rpm_to_vmax(60.0, USTEPS_REV, FCLK));
    CHECK_NEAR(tmc_motion_units_vmax_to_rps(71583, USTEPS_REV, FCLK), 1.0, 0.001);
}

static void test_rps2_amax(void)
{
    // 2 rev/s^2 = 102400 usteps/s^2
    // AMAX = 102400 * 2^41 / (12e6)^2 = 1563,5...
    CHECK_EQ_U32(tmc_motion_units_rps2_to_amax(2.0, USTEPS_REV, FCLK), 1564);
    CHECK_NEAR(tmc_motion_units_amax_to_rps2(1564, USTEPS_REV, FCLK), 2.0, 0.01);
    CHECK_EQ_U32(tmc_motion_units_rps2_to_amax(0.0, USTEPS_REV, FCLK), 0);
}

static void test_grad_steps(void)
{
    CHECK_EQ_I32(tmc_motion_units_degrees_to_steps(360.0, USTEPS_REV), 51200);
    CHECK_EQ_I32(tmc_motion_units_degrees_to_steps(-360.0, USTEPS_REV), -51200);
    CHECK_EQ_I32(tmc_motion_units_degrees_to_steps(1.0, USTEPS_REV), 142);
    CHECK_NEAR(tmc_motion_units_steps_to_degrees(51200, USTEPS_REV), 360.0, 1e-9);
    CHECK_NEAR(tmc_motion_units_steps_to_degrees(-25600, USTEPS_REV), -180.0, 1e-9);
    CHECK_NEAR(tmc_motion_units_steps_to_degrees(1, 0), 0.0, 1e-12);
}

static void test_vmax_signed(void)
{
    CHECK_NEAR(tmc_motion_units_vmax_signed_to_rpm(71583, USTEPS_REV, FCLK),
               60.0, 0.01);
    CHECK_NEAR(tmc_motion_units_vmax_signed_to_rpm(-71583, USTEPS_REV, FCLK),
               -60.0, 0.01);
    CHECK_NEAR(tmc_motion_units_vmax_signed_to_rpm(0, USTEPS_REV, FCLK),
               0.0, 1e-12);
}

int main(void)
{
    test_rpm_vmax();
    test_rps_vmax();
    test_rps2_amax();
    test_grad_steps();
    test_vmax_signed();
    return test_report("units");
}
