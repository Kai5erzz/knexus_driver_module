/*
 * test_drv8701e.c - DRV8701E current-loop signed step (staircase) test
 *
 * Generates a 4-segment signed staircase waveform per period:
 *   +peak -> +base -> -peak -> -base   (each segment = period/4)
 * With default base=0 this becomes: +peak -> 0 -> -peak -> 0
 *
 * ADC current sensing is updated by DRV8701E_Current_*().
 * Current PI parameters and waveform parameters are global on purpose so
 * OctoLink/GDB can tune them at runtime.
 */
#include "test_drv8701e.h"
#include "knx_time.h"
#include "knx_gpio.h"
#include "knx_board.h"

#include "drv8701e.h"
#include "encoder.h"
#include "knx_telemetry.h"

/* ---- LED heartbeat via platform GPIO ---- */
static knx_gpio_t gpio_led;

/* ---- Current loop test parameters (tunable globals) ---- */
float test_current_peak_a = 0.50f;
float test_current_base_a = 0.0f;
float test_current_period_ms = 3000.0f;
float test_current_enable = 1.0f;
float test_current_left_enable = 1.0f;
float test_current_right_enable = 1.0f;
float test_current_left_scale = 1.0f;
float test_current_right_scale = 1.0f;

/* ---- Timing state ---- */
static uint32_t current_tick;
static uint32_t ctrl_tick;
static uint32_t debug_tick;
static uint32_t step_start;
static uint32_t led_tick;
static uint32_t last_control_sample_count;
static uint32_t last_phase_ms;
static uint32_t last_target_segment;
static float last_current_enable;
#define LED_PERIOD_MS  500

/* ============================================================ */

void test_drv8701e_init(void *octo)
{
    (void)octo;

    const knx_gpio_t *debug_led = knx_board_get_debug_led();
    if (debug_led != NULL) {
        gpio_led = *debug_led;
    }

    DRV8701E_StopAll();
    DRV8701E_CurrentCtrlInit();

    DRV8701E_Current_Start();
    DRV8701E_Current_CalibrateZero(100, 1);

    current_tick = 0;
    ctrl_tick    = 0;
    debug_tick   = 0;
    led_tick     = 0;
    step_start   = knx_millis();
    last_control_sample_count = drv8701e_current.sample_count;
    last_phase_ms = 0;
    last_target_segment = 0;
    last_current_enable = test_current_enable;
}

/*
 * Signed staircase waveform: one period = 4 equal segments
 *   [0,  25%)  +peak
 *   [25%, 50%) +base
 *   [50%, 75%) -peak
 *   [75%,100%) -base
 */
static float test_current_target_at(uint32_t now_ms)
{
    float period_ms = test_current_period_ms;
    if (period_ms < 1.0f) period_ms = 1.0f;

    uint32_t period_u32 = (uint32_t)period_ms;
    if (period_u32 == 0U) period_u32 = 1U;

    uint32_t phase_ms = (now_ms - step_start) % period_u32;
    uint32_t seg = (phase_ms * 4U) / period_u32;   /* 0..3 */
    if (seg > 3U) seg = 3U;

    if (test_current_enable <= 0.0f) {
        return 0.0f;
    }

    switch (seg) {
    case 0:  return  test_current_peak_a;
    case 1:  return  test_current_base_a;
    case 2:  return -test_current_peak_a;
    default: return -test_current_base_a;
    }
}

void test_drv8701e_loop(void)
{
    uint32_t now = knx_millis();

    /* ---- 1ms: consume ADC DMA sample and run current PI ---- */
    if (now - current_tick >= 1) {
        current_tick = now;

        DRV8701E_Current_Poll(1);

        if (test_current_enable > 0.0f && last_current_enable <= 0.0f) {
            step_start = now;
            last_phase_ms = 0;
            cc_left.integral = 0.0f;
            cc_right.integral = 0.0f;
            cc_left.output = 0.0f;
            cc_right.output = 0.0f;
        }
        last_current_enable = test_current_enable;

        float period_ms = test_current_period_ms;
        if (period_ms < 1.0f) period_ms = 1.0f;
        uint32_t period_u32 = (uint32_t)period_ms;
        if (period_u32 == 0U) period_u32 = 1U;
        uint32_t phase_ms = (now - step_start) % period_u32;
        uint32_t target_segment = (phase_ms * 4U) / period_u32;
        if (target_segment > 3U) target_segment = 3U;
        if (test_current_enable > 0.0f && phase_ms < last_phase_ms) {
            cc_left.integral = 0.0f;
            cc_right.integral = 0.0f;
            cc_left.output = 0.0f;
            cc_right.output = 0.0f;
        }
        if (test_current_enable > 0.0f && target_segment != last_target_segment) {
            cc_left.integral = 0.0f;
            cc_right.integral = 0.0f;
            cc_left.output = 0.0f;
            cc_right.output = 0.0f;
        }
        last_phase_ms = phase_ms;
        last_target_segment = target_segment;

        float target = test_current_target_at(now);
        cc_left.target = (test_current_left_enable > 0.0f)
                       ? (target * test_current_left_scale * MOTOR1_TARGET_CURRENT_SIGN)
                       : 0.0f;
        cc_right.target = (test_current_right_enable > 0.0f)
                        ? (target * test_current_right_scale * MOTOR2_TARGET_CURRENT_SIGN)
                        : 0.0f;

        if (test_current_enable <= 0.0f) {
            cc_left.integral = 0.0f;
            cc_right.integral = 0.0f;
            cc_left.output = 0.0f;
            cc_right.output = 0.0f;
            DRV8701E_StopAll();
        } else if (drv8701e_current.valid &&
                   drv8701e_current.sample_count != last_control_sample_count) {
            last_control_sample_count = drv8701e_current.sample_count;
            if (test_current_left_enable > 0.0f) {
                DRV8701E_CurrentControl(&cc_left,
                                        &motor_left,
                                        drv8701e_current.current_left_filtered_a);
            } else {
                cc_left.integral = 0.0f;
                cc_left.output = 0.0f;
                DRV8701E_Stop(&motor_left);
            }

            if (test_current_right_enable > 0.0f) {
                DRV8701E_CurrentControl(&cc_right,
                                        &motor_right,
                                        drv8701e_current.current_right_filtered_a);
            } else {
                cc_right.integral = 0.0f;
                cc_right.output = 0.0f;
                DRV8701E_Stop(&motor_right);
            }
        }
    }

    /* ---- 20ms: optional encoder refresh for local diagnostics ---- */
    if (now - ctrl_tick >= 20) {
        ctrl_tick = now;
        Encoder_CalcSpeed(&encoder_left);
        Encoder_CalcSpeed(&encoder_right);
    }

    /* ---- 20ms: telemetry output (internally calls DRV8701E_DebugOcto) ---- */
    if (now - debug_tick >= 20) {
        debug_tick = now;
        knx_telemetry_update();
    }

    /* ---- 500ms: LED heartbeat ---- */
    if (now - led_tick >= LED_PERIOD_MS) {
        led_tick = now;
        knx_gpio_toggle(gpio_led);
    }
}
