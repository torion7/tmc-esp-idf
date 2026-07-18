// tmc_motion_decode — Rohregister -> dekodierte Diagnose. Bewusst OHNE
// ESP-IDF-Abhaengigkeiten (nur libc): die Referenzvektoren in host_tests/
// compilieren diese Datei direkt mit dem Host-gcc.

#include <string.h>

#include "tmc_motion.h"

static bool drv_status_flag(uint32_t drv_status, uint8_t bit)
{
    return (drv_status & (1UL << bit)) != 0U;
}

void tmc_motion_decode_diag(const tmc_motion_diag_t *diag, tmc_motion_decoded_diag_t *out)
{
    memset(out, 0, sizeof(*out));

    out->drv_enn          = ((diag->ioin >> 4) & 0x1U) != 0;
    out->sd_mode          = ((diag->ioin >> 6) & 0x1U) != 0;
    out->swcomp           = ((diag->ioin >> 7) & 0x1U) != 0;

    out->stst             = drv_status_flag(diag->drv_status, 31);
    out->ola              = drv_status_flag(diag->drv_status, 29);
    out->olb              = drv_status_flag(diag->drv_status, 30);
    out->s2ga             = drv_status_flag(diag->drv_status, 27);
    out->s2gb             = drv_status_flag(diag->drv_status, 28);
    out->s2vsa            = drv_status_flag(diag->drv_status, 12);
    out->s2vsb            = drv_status_flag(diag->drv_status, 13);
    out->otpw             = drv_status_flag(diag->drv_status, 26);
    out->ot               = drv_status_flag(diag->drv_status, 25);
    out->stall            = drv_status_flag(diag->drv_status, 24);
    out->fsactive         = drv_status_flag(diag->drv_status, 15);
    out->stealth          = drv_status_flag(diag->drv_status, 14);

    out->status_sg        = ((diag->ramp_stat >> 13) & 0x1U) != 0;
    out->event_stop_sg    = ((diag->ramp_stat >>  6) & 0x1U) != 0;
    out->position_reached = ((diag->ramp_stat >>  9) & 0x1U) != 0;
    out->velocity_reached = ((diag->ramp_stat >>  8) & 0x1U) != 0;

    out->cs_actual        = (diag->drv_status >> 16) & 0x1FU;
    out->sg_result        =  diag->drv_status        & 0x3FFU;
    out->sg4_result       =  diag->sg4_result        & 0x3FFU;

    // stealthChop-PWM-Aussteuerung (PWM_SCALE 0x71, Layout auf 5160 und
    // 5240 identisch): [9:0] Summe, [24:16] Auto-Anteil als 9-bit signed.
    out->pwm_scale_sum    = diag->pwm_scale & 0x3FFU;
    int32_t auto9         = (int32_t)((diag->pwm_scale >> 16) & 0x1FFU);
    if (auto9 & 0x100) {
        auto9 -= 0x200;                    // 9-bit Zweierkomplement
    }
    out->pwm_scale_auto   = auto9;
    out->pwm_scale_pct    = (out->pwm_scale_sum * 100U) / 1023U;

    // Chip-Temperatur (nur Chips mit Temperatur-ADC, aktuell TMC5240):
    // T[degC] = (ADC_TEMP - 2038) / 7.7 (Datenblatt). adc_temp==0 heisst
    // "kein/kein gelesener Sensor" -> 0.0 statt unsinniger -264 degC.
    out->chip_temp_c      = (diag->adc_temp != 0U)
                                ? ((float)diag->adc_temp - 2038.0f) / 7.7f
                                : 0.0f;
}
