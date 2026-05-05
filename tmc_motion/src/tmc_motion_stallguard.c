// StallGuard test + calibration sweep, ported to the multi-instance
// tmc_motion handle API. Each motor gets its own pair of FreeRTOS tasks
// with its own context; tasks are gated by the per-handle busy flag.

#include "tmc_motion.h"
#include "tmc_motion_internal.h"

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "tmc_motion_sg";

#define DEFAULT_SG_TEST_DURATION_MS    5000U
#define DEFAULT_SG_TEST_PERIOD_MS       100U
#define DEFAULT_SG_TEST_VELOCITY      18000

#define DEFAULT_SG_CAL_POINT_DURATION_MS 1200U
#define DEFAULT_SG_CAL_SAMPLE_PERIOD_MS    80U
#define DEFAULT_SG_CAL_SETTLE_MS          350U

// ===========================================================================
// Sanitization helpers
// ===========================================================================
static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
static uint32_t san_sample_period_ms (uint32_t v) { return clamp_u32(v,  20U, 2000U); }
static uint32_t san_duration_ms      (uint32_t v) { return clamp_u32(v, 250U, 60000U); }
static uint32_t san_cal_sample_period(uint32_t v) { return clamp_u32(v,  30U, 1000U); }
static uint32_t san_cal_point_duration(uint32_t v){ return clamp_u32(v, 300U, 8000U); }
static uint32_t san_cal_settle_ms    (uint32_t v) { return v > 4000U ? 4000U : v; }

static uint32_t count_sweep_u32(uint32_t lo, uint32_t hi, uint32_t step) {
    if (step == 0 || lo > hi) return 0;
    return ((hi - lo) / step) + 1U;
}
static uint32_t count_sweep_i32(int32_t lo, int32_t hi, int32_t step) {
    if (step <= 0 || lo > hi) return 0;
    return (uint32_t)(((uint32_t)(hi - lo) / (uint32_t)step) + 1U);
}

// ===========================================================================
// Sample / point ring buffers (caller holds lock)
// ===========================================================================
void tmc_motion_reset_stallguard_samples_locked(tmc_motion_handle_t h)
{
    h->sg_sample_head  = 0;
    h->sg_sample_count = 0;
    memset(h->sg_samples, 0, sizeof(h->sg_samples));
}

void tmc_motion_push_stallguard_sample_locked(tmc_motion_handle_t h, const tmc_motion_diag_t *diag)
{
    size_t index = h->sg_sample_head;
    tmc_motion_stallguard_sample_t *sample = &h->sg_samples[index];
    tmc_motion_decoded_diag_t       decoded;
    tmc_motion_decode_diag(diag, &decoded);

    sample->timestamp_ms  = (uint32_t)(esp_timer_get_time() / 1000ULL);
    sample->xactual       = diag->xactual;
    sample->vactual       = diag->vactual;
    sample->sg_result     = decoded.sg_result;
    sample->tstep         = diag->tstep;
    sample->ramp_stat     = diag->ramp_stat;
    sample->drv_status    = diag->drv_status;
    sample->status_sg     = decoded.status_sg;
    sample->event_stop_sg = decoded.event_stop_sg;

    h->sg_sample_head = (index + 1U) % TMC_MOTION_STALLGUARD_SAMPLE_CAPACITY;
    if (h->sg_sample_count < TMC_MOTION_STALLGUARD_SAMPLE_CAPACITY) {
        h->sg_sample_count++;
    } else {
        h->sg_test.overflow = true;
    }
    h->sg_test.sample_count = (uint32_t)h->sg_sample_count;
}

void tmc_motion_reset_stallguard_calibration_points_locked(tmc_motion_handle_t h)
{
    h->sg_cal_head  = 0;
    h->sg_cal_count = 0;
    memset(h->sg_cal_points, 0, sizeof(h->sg_cal_points));
}

void tmc_motion_push_stallguard_calibration_point_locked(tmc_motion_handle_t h,
                                                         const tmc_motion_stallguard_calibration_point_t *p)
{
    size_t index = h->sg_cal_head;
    h->sg_cal_points[index] = *p;
    h->sg_cal_head = (index + 1U) % TMC_MOTION_STALLGUARD_CAL_POINT_CAPACITY;
    if (h->sg_cal_count < TMC_MOTION_STALLGUARD_CAL_POINT_CAPACITY) {
        h->sg_cal_count++;
    }
}

// ===========================================================================
// Recommendation derivation
// ===========================================================================
void tmc_motion_derive_stallguard_recommendation_locked(tmc_motion_handle_t h)
{
    if (h->sg_cal_count == 0) {
        h->sg_cal.has_recommendation  = false;
        h->sg_cal.suggested_sgt       = h->stallguard.sgt;
        h->sg_cal.suggested_tcoolthrs = h->stallguard.tcoolthrs;
        h->sg_cal.worst_sg_min        = 0;
        return;
    }

    uint32_t worst_sg_min = UINT32_MAX;
    uint64_t tstep_sum    = 0;
    uint32_t tstep_count  = 0;

    size_t start = (h->sg_cal_head + TMC_MOTION_STALLGUARD_CAL_POINT_CAPACITY - h->sg_cal_count)
                   % TMC_MOTION_STALLGUARD_CAL_POINT_CAPACITY;
    for (size_t i = 0; i < h->sg_cal_count; ++i) {
        const tmc_motion_stallguard_calibration_point_t *p =
            &h->sg_cal_points[(start + i) % TMC_MOTION_STALLGUARD_CAL_POINT_CAPACITY];
        if (p->sg_min < worst_sg_min) worst_sg_min = p->sg_min;
        if (p->tstep_avg > 0) { tstep_sum += p->tstep_avg; tstep_count++; }
    }

    int8_t suggested_sgt = h->stallguard.sgt;
    if      (worst_sg_min <  80U) suggested_sgt = (int8_t)(suggested_sgt + 10);
    else if (worst_sg_min < 120U) suggested_sgt = (int8_t)(suggested_sgt +  5);
    else if (worst_sg_min > 300U) suggested_sgt = (int8_t)(suggested_sgt -  8);
    else if (worst_sg_min > 220U) suggested_sgt = (int8_t)(suggested_sgt -  4);
    if (suggested_sgt >  63) suggested_sgt =  63;
    if (suggested_sgt < -64) suggested_sgt = -64;

    uint32_t suggested_tcoolthrs = h->stallguard.tcoolthrs;
    if (tstep_count > 0) {
        uint32_t avg_tstep   = (uint32_t)(tstep_sum / tstep_count);
        suggested_tcoolthrs  = avg_tstep + (avg_tstep / 5U);
        if (suggested_tcoolthrs > 0xFFFFFU) suggested_tcoolthrs = 0xFFFFFU;
    }

    h->sg_cal.has_recommendation  = true;
    h->sg_cal.suggested_sgt       = suggested_sgt;
    h->sg_cal.suggested_tcoolthrs = suggested_tcoolthrs;
    h->sg_cal.worst_sg_min        = (worst_sg_min == UINT32_MAX) ? 0U : worst_sg_min;
}

// ===========================================================================
// StallGuard test task
// ===========================================================================
typedef struct {
    tmc_motion_handle_t                  h;
    tmc_motion_stallguard_test_request_t req;
} sg_test_arg_t;

static void stallguard_test_task(void *arg_void)
{
    sg_test_arg_t *arg = (sg_test_arg_t *)arg_void;
    tmc_motion_handle_t h = arg->h;
    tmc_motion_stallguard_test_request_t req = arg->req;
    free(arg);

    uint64_t start_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);

    // Setup
    if (TMC_MOTION_BLOCKING_TAKE(h)) {
        tmc_motion_clear_last_error_locked(h);
        if (tmc_motion_start_velocity_with_stallguard_arm_locked(h, req.velocity, h->stallguard.sg_stop) != ESP_OK) {
            tmc_motion_set_last_error_locked(h, ESP_FAIL, "stallguard test setup failed");
            h->sg_test.active     = false;
            h->sg_test.completed  = true;
            h->sg_test_task       = NULL;
            TMC_MOTION_GIVE(h);
            vTaskDelete(NULL);
            return;
        }
        h->commanded_velocity = req.velocity;
        TMC_MOTION_GIVE(h);
    }

    // Sample loop
    while (true) {
        bool should_stop = false;
        if (TMC_MOTION_BLOCKING_TAKE(h)) {
            uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
            h->sg_test.elapsed_ms = (uint32_t)(now_ms - start_ms);

            if (tmc_motion_refresh_diag_locked(h) == ESP_OK) {
                tmc_motion_push_stallguard_sample_locked(h, &h->last_diag);
                tmc_motion_decoded_diag_t decoded;
                tmc_motion_decode_diag(&h->last_diag, &decoded);
                if (decoded.event_stop_sg) {
                    h->sg_test.event_stop_sg = true;
                    should_stop = true;
                }
            }

            if (h->sg_test.cancel_requested ||
                h->sg_test.elapsed_ms >= req.duration_ms) {
                should_stop = true;
            }
            TMC_MOTION_GIVE(h);
        }
        if (should_stop) break;
        vTaskDelay(pdMS_TO_TICKS(req.sample_period_ms));
    }

    // Tear down
    if (TMC_MOTION_BLOCKING_TAKE(h)) {
        if (h->ops->stop(h->chip_handle) != ESP_OK) {
            tmc_motion_set_last_error_locked(h, ESP_FAIL, "stop after stallguard test failed");
        }
        h->commanded_velocity = 0;
        vTaskDelay(pdMS_TO_TICKS(50));
        (void)tmc_motion_clear_ramp_status_events_locked(h);
        (void)tmc_motion_refresh_diag_locked(h);
        h->sg_test.active     = false;
        h->sg_test.completed  = true;
        h->sg_test_task       = NULL;
        TMC_MOTION_GIVE(h);
    }

    vTaskDelete(NULL);
}

esp_err_t tmc_motion_start_stallguard_test(tmc_motion_handle_t h,
                                           const tmc_motion_stallguard_test_request_t *request)
{
    ESP_RETURN_ON_FALSE(h && request, ESP_ERR_INVALID_ARG, TAG, "null arg");

    sg_test_arg_t *arg = calloc(1, sizeof(*arg));
    ESP_RETURN_ON_FALSE(arg != NULL, ESP_ERR_NO_MEM, TAG, "alloc test arg");

    arg->h   = h;
    arg->req = *request;
    if (arg->req.duration_ms      == 0) arg->req.duration_ms      = DEFAULT_SG_TEST_DURATION_MS;
    if (arg->req.sample_period_ms == 0) arg->req.sample_period_ms = DEFAULT_SG_TEST_PERIOD_MS;
    if (arg->req.velocity         == 0) arg->req.velocity         = DEFAULT_SG_TEST_VELOCITY;
    arg->req.duration_ms      = san_duration_ms     (arg->req.duration_ms);
    arg->req.sample_period_ms = san_sample_period_ms(arg->req.sample_period_ms);

    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");
    if (tmc_motion_busy_locked(h)) {
        tmc_motion_unlock_internal(h);
        free(arg);
        return ESP_ERR_INVALID_STATE;
    }

    tmc_motion_reset_stallguard_samples_locked(h);
    memset(&h->sg_test, 0, sizeof(h->sg_test));
    h->sg_test.active           = true;
    h->sg_test.duration_ms      = arg->req.duration_ms;
    h->sg_test.sample_period_ms = arg->req.sample_period_ms;
    h->sg_test.velocity         = arg->req.velocity;
    tmc_motion_clear_last_error_locked(h);

    BaseType_t ok = xTaskCreate(stallguard_test_task, "tm_sg_test",
                                6144, arg, 5, &h->sg_test_task);
    if (ok != pdPASS) {
        h->sg_test.active = false;
        h->sg_test_task   = NULL;
        tmc_motion_unlock_internal(h);
        free(arg);
        return ESP_ERR_NO_MEM;
    }
    tmc_motion_unlock_internal(h);
    return ESP_OK;
}

esp_err_t tmc_motion_get_stallguard_samples(tmc_motion_handle_t h,
                                            tmc_motion_stallguard_sample_t *samples,
                                            size_t max_samples,
                                            size_t *out_count)
{
    ESP_RETURN_ON_FALSE(h && samples && out_count, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");

    size_t count = h->sg_sample_count;
    if (count > max_samples) count = max_samples;

    size_t start = (h->sg_sample_head + TMC_MOTION_STALLGUARD_SAMPLE_CAPACITY - h->sg_sample_count)
                   % TMC_MOTION_STALLGUARD_SAMPLE_CAPACITY;
    for (size_t i = 0; i < count; ++i) {
        samples[i] = h->sg_samples[(start + i) % TMC_MOTION_STALLGUARD_SAMPLE_CAPACITY];
    }
    *out_count = count;
    tmc_motion_unlock_internal(h);
    return ESP_OK;
}

// ===========================================================================
// Calibration sweep task
// ===========================================================================
typedef struct {
    tmc_motion_handle_t                          h;
    tmc_motion_stallguard_calibration_request_t  req;
} sg_cal_arg_t;

static void calibration_collect_one_point(tmc_motion_handle_t h,
                                          const tmc_motion_stallguard_calibration_request_t *req,
                                          uint16_t current_ma,
                                          uint32_t amax,
                                          int32_t  velocity,
                                          bool    *out_cancel)
{
    *out_cancel = false;

    bool setup_ok = false;
    if (TMC_MOTION_BLOCKING_TAKE(h)) {
        esp_err_t err = h->ops->set_rms_current_ma(h->chip_handle, current_ma, h->motion_phys.hold_percent);
        if (err == ESP_OK) err = h->ops->set_amax(h->chip_handle, amax);
        if (err == ESP_OK) err = h->ops->configure_stallguard(h->chip_handle,
                                                              h->stallguard.sgt,
                                                              h->stallguard.sfilt,
                                                              h->stallguard.tcoolthrs, false);
        if (err == ESP_OK) err = tmc_motion_clear_ramp_status_events_locked(h);
        if (err == ESP_OK) err = h->ops->set_velocity_mode(h->chip_handle, velocity);

        if (err == ESP_OK) {
            h->motion_phys.run_current_ma = current_ma;
            h->commanded_velocity         = velocity;
            setup_ok = true;
        } else {
            tmc_motion_set_last_error_locked(h, err, "calibration setup failed");
        }
        TMC_MOTION_GIVE(h);
    }
    if (!setup_ok) { *out_cancel = true; return; }

    if (req->settle_ms > 0U) vTaskDelay(pdMS_TO_TICKS(req->settle_ms));

    if (TMC_MOTION_BLOCKING_TAKE(h)) {
        if (h->sg_cal.cancel_requested) { TMC_MOTION_GIVE(h); *out_cancel = true; return; }
        (void)tmc_motion_clear_ramp_status_events_locked(h);
        TMC_MOTION_GIVE(h);
    }

    const uint32_t sample_count = req->point_duration_ms / req->sample_period_ms;
    uint64_t sg_sum = 0, tstep_sum = 0;
    uint32_t sg_min = UINT32_MAX, sg_max = 0;
    uint32_t tstep_valid = 0, sample_valid = 0;
    bool     event_stop  = false;

    for (uint32_t i = 0; i < sample_count; ++i) {
        vTaskDelay(pdMS_TO_TICKS(req->sample_period_ms));

        if (TMC_MOTION_BLOCKING_TAKE(h)) {
            if (h->sg_cal.cancel_requested) { TMC_MOTION_GIVE(h); *out_cancel = true; return; }

            if (tmc_motion_refresh_diag_locked(h) == ESP_OK) {
                tmc_motion_decoded_diag_t decoded;
                tmc_motion_decode_diag(&h->last_diag, &decoded);
                uint32_t sg = decoded.sg_result;
                if (sg < sg_min) sg_min = sg;
                if (sg > sg_max) sg_max = sg;
                sg_sum += sg;
                sample_valid++;
                if (h->last_diag.tstep > 0U) { tstep_sum += h->last_diag.tstep; tstep_valid++; }
                if (decoded.event_stop_sg) event_stop = true;
            }
            TMC_MOTION_GIVE(h);
        }
    }

    if (TMC_MOTION_BLOCKING_TAKE(h)) {
        (void)h->ops->stop(h->chip_handle);
        h->commanded_velocity = 0;

        tmc_motion_stallguard_calibration_point_t point = {
            .run_current_ma = current_ma,
            .amax           = amax,
            .velocity       = velocity,
            .sg_min         = sample_valid > 0 ? sg_min                              : 0,
            .sg_max         = sample_valid > 0 ? sg_max                              : 0,
            .sg_avg         = sample_valid > 0 ? (uint32_t)(sg_sum    / sample_valid) : 0,
            .tstep_avg      = tstep_valid  > 0 ? (uint32_t)(tstep_sum / tstep_valid)  : 0,
            .event_stop_sg  = event_stop,
        };
        tmc_motion_push_stallguard_calibration_point_locked(h, &point);
        h->sg_cal.tested_points++;
        TMC_MOTION_GIVE(h);
    }
}

static void stallguard_calibration_task(void *arg_void)
{
    sg_cal_arg_t *arg = (sg_cal_arg_t *)arg_void;
    tmc_motion_handle_t h = arg->h;
    tmc_motion_stallguard_calibration_request_t req = arg->req;
    free(arg);

    // Snapshot original config so we can restore it after the sweep.
    tmc_motion_config_phys_t       orig_motion     = {0};
    tmc_motion_stallguard_config_t orig_stallguard = {0};
    bool has_orig = false;

    if (TMC_MOTION_BLOCKING_TAKE(h)) {
        orig_motion     = h->motion_phys;
        orig_stallguard = h->stallguard;
        has_orig        = true;
        tmc_motion_clear_last_error_locked(h);
        TMC_MOTION_GIVE(h);
    }

    bool cancel = false;
    for (uint16_t cur = req.current_min_ma; cur <= req.current_max_ma && !cancel;
         cur = (uint16_t)(cur + req.current_step_ma)) {
        for (uint32_t a = req.amax_min; a <= req.amax_max && !cancel; a += req.amax_step) {
            for (int32_t v = req.velocity_min; v <= req.velocity_max && !cancel; v += req.velocity_step) {

                if (TMC_MOTION_BLOCKING_TAKE(h)) {
                    cancel = h->sg_cal.cancel_requested;
                    TMC_MOTION_GIVE(h);
                }
                if (cancel) break;

                calibration_collect_one_point(h, &req, cur, a, v, &cancel);
            }
        }
    }

    if (TMC_MOTION_BLOCKING_TAKE(h)) {
        (void)h->ops->stop(h->chip_handle);
        h->commanded_velocity = 0;

        if (has_orig) {
            if (h->ops->set_rms_current_ma(h->chip_handle,
                                           orig_motion.run_current_ma,
                                           orig_motion.hold_percent) == ESP_OK) {
                h->motion_phys.run_current_ma = orig_motion.run_current_ma;
                h->motion_phys.hold_percent   = orig_motion.hold_percent;
            }
            (void)tmc_motion_apply_profile_locked(h, 0);
            if (h->ops->configure_stallguard(h->chip_handle,
                                             orig_stallguard.sgt,
                                             orig_stallguard.sfilt,
                                             orig_stallguard.tcoolthrs,
                                             orig_stallguard.sg_stop) == ESP_OK) {
                h->stallguard = orig_stallguard;
            }
        }

        tmc_motion_derive_stallguard_recommendation_locked(h);
        h->sg_cal.active           = false;
        h->sg_cal.completed        = true;
        h->sg_cal.cancel_requested = false;
        h->sg_cal_task             = NULL;
        TMC_MOTION_GIVE(h);
    }

    vTaskDelete(NULL);
}

esp_err_t tmc_motion_start_stallguard_calibration(tmc_motion_handle_t h,
                                                  const tmc_motion_stallguard_calibration_request_t *request)
{
    ESP_RETURN_ON_FALSE(h && request, ESP_ERR_INVALID_ARG, TAG, "null arg");

    sg_cal_arg_t *arg = calloc(1, sizeof(*arg));
    ESP_RETURN_ON_FALSE(arg != NULL, ESP_ERR_NO_MEM, TAG, "alloc cal arg");
    arg->h   = h;
    arg->req = *request;

    // Defaults / sanitation
    if (arg->req.current_min_ma == 0)                       arg->req.current_min_ma  = h->motion_phys.run_current_ma;
    if (arg->req.current_max_ma  < arg->req.current_min_ma) arg->req.current_max_ma  = arg->req.current_min_ma;
    if (arg->req.current_step_ma == 0)                      arg->req.current_step_ma = 100;

    if (arg->req.amax_min == 0)                              arg->req.amax_min  = tmc_motion_units_rps2_to_amax(h->motion_phys.amax_rps2,
                                                                                                                h->microsteps_per_rev, h->fclk_hz);
    if (arg->req.amax_max < arg->req.amax_min)               arg->req.amax_max  = arg->req.amax_min;
    if (arg->req.amax_step == 0)                             arg->req.amax_step = 200;

    if (arg->req.velocity_min == 0)                          arg->req.velocity_min  = 12000;
    if (arg->req.velocity_max  < arg->req.velocity_min)      arg->req.velocity_max  = arg->req.velocity_min;
    if (arg->req.velocity_step <= 0)                         arg->req.velocity_step = 4000;

    arg->req.settle_ms         = san_cal_settle_ms      (arg->req.settle_ms         == 0 ? DEFAULT_SG_CAL_SETTLE_MS         : arg->req.settle_ms);
    arg->req.point_duration_ms = san_cal_point_duration (arg->req.point_duration_ms == 0 ? DEFAULT_SG_CAL_POINT_DURATION_MS : arg->req.point_duration_ms);
    arg->req.sample_period_ms  = san_cal_sample_period  (arg->req.sample_period_ms  == 0 ? DEFAULT_SG_CAL_SAMPLE_PERIOD_MS  : arg->req.sample_period_ms);

    uint32_t total_points = count_sweep_u32(arg->req.current_min_ma, arg->req.current_max_ma, arg->req.current_step_ma)
                          * count_sweep_u32(arg->req.amax_min,        arg->req.amax_max,       arg->req.amax_step)
                          * count_sweep_i32(arg->req.velocity_min,    arg->req.velocity_max,   arg->req.velocity_step);
    if (total_points == 0) {
        free(arg);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");
    if (tmc_motion_busy_locked(h)) {
        tmc_motion_unlock_internal(h);
        free(arg);
        return ESP_ERR_INVALID_STATE;
    }

    tmc_motion_reset_stallguard_calibration_points_locked(h);
    memset(&h->sg_cal, 0, sizeof(h->sg_cal));
    h->sg_cal.active            = true;
    h->sg_cal.total_points      = total_points;
    h->sg_cal.settle_ms         = arg->req.settle_ms;
    h->sg_cal.point_duration_ms = arg->req.point_duration_ms;
    h->sg_cal.sample_period_ms  = arg->req.sample_period_ms;
    tmc_motion_clear_last_error_locked(h);

    BaseType_t ok = xTaskCreate(stallguard_calibration_task, "tm_sg_cal",
                                7168, arg, 5, &h->sg_cal_task);
    if (ok != pdPASS) {
        h->sg_cal.active = false;
        h->sg_cal_task   = NULL;
        tmc_motion_unlock_internal(h);
        free(arg);
        return ESP_ERR_NO_MEM;
    }
    tmc_motion_unlock_internal(h);
    return ESP_OK;
}

esp_err_t tmc_motion_get_stallguard_calibration_points(tmc_motion_handle_t h,
                                                       tmc_motion_stallguard_calibration_point_t *points,
                                                       size_t max_points,
                                                       size_t *out_count)
{
    ESP_RETURN_ON_FALSE(h && points && out_count, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_ERROR(tmc_motion_lock_internal(h), TAG, "lock");

    size_t count = h->sg_cal_count;
    if (count > max_points) count = max_points;

    size_t start = (h->sg_cal_head + TMC_MOTION_STALLGUARD_CAL_POINT_CAPACITY - h->sg_cal_count)
                   % TMC_MOTION_STALLGUARD_CAL_POINT_CAPACITY;
    for (size_t i = 0; i < count; ++i) {
        points[i] = h->sg_cal_points[(start + i) % TMC_MOTION_STALLGUARD_CAL_POINT_CAPACITY];
    }
    *out_count = count;
    tmc_motion_unlock_internal(h);
    return ESP_OK;
}
