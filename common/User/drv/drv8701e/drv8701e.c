/**
 * @file    drv8701e.c
 * @author  kaiser
 * @version V5.0.0
 * @date    2026-06-02
 * @brief   DRV8701E 鐢垫満椹卞姩 + ADC 鐢垫祦閲囨牱瀹炵幇
 *
 * v5.0 鍙樻洿:
 *   - ADC 鐢垫祦閲囨牱杩佺Щ鍒?knx_adc platform 鎶借薄
 *   - drv8701e.h 涓嶅啀 include HAL 澶存枃浠? *   - HAL 渚濊禆浠呴檺浜庢湰 .c 鏂囦欢鐨?TIM 瀹?(PWM/GPIO 宸茶蛋 platform)
 */

#include "drv8701e.h"
#include "knexus_config.h"
#include "encoder.h"
#include "PID.h"
#include "knx_time.h"
#include <math.h>

/* TIM 瀹忎粛闇€瑕?HAL, 浠呴檺鏈枃浠朵娇鐢?*/

/* ==================== 瀹夊叏闄愬箙 ==================== */
#ifndef DRV8701E_SAFE_DUTY_LIMIT
#define DRV8701E_SAFE_DUTY_LIMIT  1.0f
#endif

/* ==================== 绉佹湁鍙橀噺 ==================== */
static float lpf_alpha = 0.0f;
static float lpf_state_left = 0.0f;
static float lpf_state_right = 0.0f;

static float offset_left_a = 0.0f;
static float offset_right_a = 0.0f;

static uint32_t adc_consecutive_timeout = 0;
static uint32_t last_adc_dma_sequence = 0;
static uint32_t debug_seq = 0;

static int8_t last_dir_left  = 0;
static int8_t last_dir_right = 0;
static const knx_pwm_channel_t *current_sample_trigger = NULL;

static uint32_t drv8701e_pwm_period(const knx_pwm_channel_t *pwm, uint32_t fallback)
{
    uint32_t period = 0U;

    if (pwm != NULL && knx_pwm_get_period(pwm, &period) == KNX_OK && period > 0U) {
        return period;
    }
    if (fallback > 0U) {
        return fallback;
    }
    return 1U;
}

static const knx_pwm_channel_t *drv8701e_sample_observer(void)
{
    if (current_sample_trigger != NULL) {
        return current_sample_trigger;
    }
    if (motor_left.port != NULL) {
        return &motor_left.port->pwm;
    }
    if (motor_right.port != NULL) {
        return &motor_right.port->pwm;
    }
    return NULL;
}

static float drv8701e_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float drv8701e_copysignf(float mag, float sign)
{
    return (sign >= 0.0f) ? drv8701e_absf(mag) : -drv8701e_absf(mag);
}

static float drv8701e_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static pid_obj_t *drv8701e_pid_sync(void **slot,
                                    float kp,
                                    float ki,
                                    float kd,
                                    float out_limit,
                                    float integral_limit)
{
    if (out_limit < 0.0f) {
        out_limit = -out_limit;
    }
    if (integral_limit < 0.0f) {
        integral_limit = -integral_limit;
    }

    if (*slot == NULL) {
        pid_config_t config = INIT_PID_CONFIG(kp,
                                              ki,
                                              kd,
                                              integral_limit,
                                              out_limit,
                                              PID_Integral_Limit);
        *slot = pid_register(&config);
    }

    pid_obj_t *pid = (pid_obj_t *)(*slot);
    if (pid != NULL) {
        pid->Kp = kp;
        pid->Ki = ki;
        pid->Kd = kd;
        pid->MaxOut = out_limit;
        pid->IntegralLimit = integral_limit;
        pid->Improve = PID_Integral_Limit;
        pid->DeadBand = 0.0f;
    }
    return pid;
}

float drv8701e_current_sample_fraction = 0.50f;
float drv8701e_current_sample_min_cnt = 20.0f;
float drv8701e_current_sample_default_cnt = 250.0f;
uint32_t drv8701e_current_sample_compare_cnt = 250U;
float drv8701e_velocity_kv_cnt_per_mps = KNEXUS_MOTOR_FEEDFORWARD_CNT_PER_MPS_DEFAULT;
float drv8701e_velocity_duty_deadzone_cnt = KNEXUS_MOTOR_DUTY_DEADZONE_CNT_DEFAULT;
float drv8701e_velocity_acc_limit_mps2 = KNEXUS_MOTOR_ACCEL_LIMIT_MPS2_DEFAULT;

/* ==================== 绉佹湁鍑芥暟 ==================== */
static float DRV8701E_LPF_CalcAlpha(float cutoff_hz, float sample_hz)
{
    float rc = 1.0f / (2.0f * 3.14159265f * cutoff_hz);
    float dt = 1.0f / sample_hz;
    return dt / (rc + dt);
}

static float safe_pwm_set(const drv8701e_port_t *port, float duty_norm)
{
    if (isnan(duty_norm) || isinf(duty_norm)) {
        knx_pwm_set_duty((knx_pwm_channel_t *)&port->pwm, 0.0f);
        return 0.0f;
    }
    if (duty_norm < 0.0f) duty_norm = 0.0f;
    if (duty_norm > DRV8701E_SAFE_DUTY_LIMIT) {
        duty_norm = DRV8701E_SAFE_DUTY_LIMIT;
    }
    knx_pwm_set_duty((knx_pwm_channel_t *)&port->pwm, duty_norm);
    return duty_norm;
}

static uint32_t abs_duty_cnt(const DRV8701E_Motor_t *motor, uint32_t arr)
{
    if (motor == NULL || motor->port == NULL) return 0U;

    int32_t duty = motor->duty;
    if (duty < 0) duty = -duty;
    if ((uint32_t)duty > arr) duty = (int32_t)arr;

    float limit = DRV8701E_SAFE_DUTY_LIMIT;
    if (limit < 0.0f) limit = 0.0f;
    if (limit > 1.0f) limit = 1.0f;

    uint32_t limited = (uint32_t)((float)arr * limit);
    if ((uint32_t)duty > limited) duty = (int32_t)limited;

    return (uint32_t)duty;
}

static void update_current_sample_point(void)
{
    const knx_pwm_channel_t *trigger = drv8701e_sample_observer();
    uint32_t arr = drv8701e_pwm_period(trigger, 0U);
    if (arr < 2U) arr = 2U;

    uint32_t left_cnt = abs_duty_cnt(&motor_left, arr);
    uint32_t right_cnt = abs_duty_cnt(&motor_right, arr);
    uint32_t active_cnt = 0U;

    if (left_cnt > 0U && right_cnt > 0U) {
        active_cnt = (left_cnt < right_cnt) ? left_cnt : right_cnt;
    } else if (left_cnt > 0U) {
        active_cnt = left_cnt;
    } else if (right_cnt > 0U) {
        active_cnt = right_cnt;
    }

    float fraction = drv8701e_current_sample_fraction;
    if (isnan(fraction) || isinf(fraction) || fraction <= 0.0f) fraction = 0.50f;
    if (fraction > 0.90f) fraction = 0.90f;

    float min_cnt_f = drv8701e_current_sample_min_cnt;
    if (isnan(min_cnt_f) || isinf(min_cnt_f) || min_cnt_f < 1.0f) min_cnt_f = 1.0f;
    if (min_cnt_f > (float)(arr - 1U)) min_cnt_f = (float)(arr - 1U);
    uint32_t min_cnt = (uint32_t)min_cnt_f;
    if (min_cnt < 1U) min_cnt = 1U;

    uint32_t sample_cnt;
    if (active_cnt > 0U) {
        sample_cnt = (uint32_t)((float)active_cnt * fraction);
        if (sample_cnt < 1U) sample_cnt = 1U;
        if (active_cnt > (min_cnt * 2U) && sample_cnt < min_cnt) {
            sample_cnt = min_cnt;
        }
        if (sample_cnt >= active_cnt) {
            sample_cnt = active_cnt - 1U;
        }
    } else {
        float default_cnt_f = drv8701e_current_sample_default_cnt;
        if (isnan(default_cnt_f) || isinf(default_cnt_f) || default_cnt_f < 1.0f) default_cnt_f = 250.0f;
        sample_cnt = (uint32_t)default_cnt_f;
    }

    if (sample_cnt < 1U) sample_cnt = 1U;
    if (sample_cnt >= arr) sample_cnt = arr - 1U;

    drv8701e_current_sample_compare_cnt = sample_cnt;
    if (current_sample_trigger != NULL) {
        (void)knx_pwm_set_compare((knx_pwm_channel_t *)current_sample_trigger, sample_cnt);
    }
}

static void set_duty_impl(DRV8701E_Motor_t *motor, int32_t duty)
{
    const drv8701e_port_t *port = motor->port;

    int32_t arr = (int32_t)drv8701e_pwm_period(&port->pwm, port->pwm.arr);
    if (arr <= 0) arr = 1;

    if (duty > arr) duty = arr;
    if (duty < -arr) duty = -arr;

    motor->duty = duty;

    int32_t signed_cmd = duty * motor->dir_sign;
    int8_t dir = (signed_cmd >= 0) ? 1 : -1;
    float duty_norm = (float)((signed_cmd >= 0) ? signed_cmd : -signed_cmd)
                    / (float)arr;

    int8_t *last_dir = (motor == &motor_left) ? &last_dir_left : &last_dir_right;
    if (dir != *last_dir && duty_norm > 0.0f) {
        knx_pwm_set_duty((knx_pwm_channel_t *)&port->pwm, 0.0f);
    }

    if (port->phase_gpio.port != NULL) {
        uint8_t physical_fwd = (dir > 0);
        uint8_t pin_high = physical_fwd ^ port->invert;
        knx_gpio_write(port->phase_gpio,
                       pin_high ? KNX_GPIO_HIGH : KNX_GPIO_LOW);
    }

    safe_pwm_set(port, duty_norm);
    update_current_sample_point();
    *last_dir = dir;
}

/* ==================== 鍏ㄥ眬瀹炰緥 ==================== */
DRV8701E_Motor_t motor_left = {
    .port = NULL,
    .dir_sign = MOTOR1_DIR_SIGN,
    .target_speed_sign = MOTOR1_TARGET_SPEED_SIGN,
    .duty = 0,
};

DRV8701E_Motor_t motor_right = {
    .port = NULL,
    .dir_sign = MOTOR2_DIR_SIGN,
    .target_speed_sign = MOTOR2_TARGET_SPEED_SIGN,
    .duty = 0,
};

DRV8701E_CurrentSample_t drv8701e_current = {0};

DRV8701E_VelocityCtrl_t vc_left = {0};
DRV8701E_VelocityCtrl_t vc_right = {0};
DRV8701E_CurrentCtrl_t cc_left = {0};
DRV8701E_CurrentCtrl_t cc_right = {0};

/* ==================== 鐢垫満 API ==================== */

void DRV8701E_Init(void)
{
    if (motor_left.port != NULL) {
        (void)knx_pwm_set_compare((knx_pwm_channel_t *)&motor_left.port->pwm, 0U);
        (void)knx_pwm_start((knx_pwm_channel_t *)&motor_left.port->pwm);
    }
    if (motor_right.port != NULL) {
        (void)knx_pwm_set_compare((knx_pwm_channel_t *)&motor_right.port->pwm, 0U);
        (void)knx_pwm_start((knx_pwm_channel_t *)&motor_right.port->pwm);
    }
    update_current_sample_point();
    if (current_sample_trigger != NULL) {
        (void)knx_pwm_start_compare((knx_pwm_channel_t *)current_sample_trigger);
    }

    DRV8701E_StopAll();

    lpf_alpha = DRV8701E_LPF_CalcAlpha(DRV8701E_LPF_CUTOFF_HZ, DRV8701E_LPF_SAMPLE_HZ);
    lpf_state_left = 0.0f;
    lpf_state_right = 0.0f;

    offset_left_a = 0.0f;
    offset_right_a = 0.0f;

    drv8701e_current.valid = 0;
    drv8701e_current.sample_count = 0;
    drv8701e_current.invalid_count = 0;
    drv8701e_current.timeout_count = 0;
    drv8701e_current.raw_left = 0;
    drv8701e_current.raw_right = 0;
    drv8701e_current.voltage_left_mv = 0.0f;
    drv8701e_current.voltage_right_mv = 0.0f;
    drv8701e_current.current_left_a = 0.0f;
    drv8701e_current.current_right_a = 0.0f;
    drv8701e_current.current_left_filtered_a = 0.0f;
    drv8701e_current.current_right_filtered_a = 0.0f;
    drv8701e_current.tim_cnt_when_read = 0;
    drv8701e_current.tim_arr = 0;
    drv8701e_current.adc_restart_count = 0;
    drv8701e_current.adc_alive = 0;
    last_adc_dma_sequence = 0;

    last_dir_left  = 0;
    last_dir_right = 0;
}

void DRV8701E_AttachPorts(const drv8701e_port_t *left,
                          const drv8701e_port_t *right)
{
    if (left != NULL) {
        motor_left.port = left;
        (void)knx_pwm_set_compare((knx_pwm_channel_t *)&left->pwm, 0U);
        knx_pwm_start((knx_pwm_channel_t *)&left->pwm);
    }
    if (right != NULL) {
        motor_right.port = right;
        (void)knx_pwm_set_compare((knx_pwm_channel_t *)&right->pwm, 0U);
        knx_pwm_start((knx_pwm_channel_t *)&right->pwm);
    }
    DRV8701E_StopAll();
}

void DRV8701E_AttachCurrentSampleTrigger(const knx_pwm_channel_t *trigger)
{
    current_sample_trigger = trigger;
    if (current_sample_trigger != NULL) {
        (void)knx_pwm_set_compare((knx_pwm_channel_t *)current_sample_trigger,
                                  drv8701e_current_sample_compare_cnt);
        (void)knx_pwm_start_compare((knx_pwm_channel_t *)current_sample_trigger);
    }
}

void DRV8701E_SetDuty(DRV8701E_Motor_t *motor, int32_t duty)
{
    if (motor == NULL || motor->port == NULL) return;
    set_duty_impl(motor, duty);
}

void DRV8701E_SetDutyNorm(DRV8701E_Motor_t *motor, float duty_norm)
{
    if (motor == NULL || motor->port == NULL || motor->port->pwm.timer == NULL) return;

    if (duty_norm > 1.0f) {
        duty_norm = 1.0f;
    } else if (duty_norm < -1.0f) {
        duty_norm = -1.0f;
    }

    int32_t arr = (int32_t)drv8701e_pwm_period(&motor->port->pwm, motor->port->pwm.arr);
    if (arr <= 0) {
        arr = 1;
    }

    DRV8701E_SetDuty(motor, (int32_t)(duty_norm * (float)arr));
}

void DRV8701E_Stop(DRV8701E_Motor_t *motor)
{
    if (motor == NULL || motor->port == NULL) return;
    knx_pwm_set_duty((knx_pwm_channel_t *)&motor->port->pwm, 0.0f);
    motor->duty = 0;
    if (motor == &motor_left)  last_dir_left  = 0;
    if (motor == &motor_right) last_dir_right = 0;
    update_current_sample_point();
}

void DRV8701E_StopAll(void)
{
    DRV8701E_Stop(&motor_left);
    DRV8701E_Stop(&motor_right);
}

/* ==================== ADC 鐢垫祦閲囨牱 (knx_adc 璺緞) ==================== */

void DRV8701E_Current_Start(void)
{
    adc_consecutive_timeout = 0;
    last_adc_dma_sequence = 0;
    drv8701e_current.adc_alive = 0;
    drv8701e_current.valid = 0;

    if (motor_left.port == NULL) {
        return;
    }

    if (knx_adc_start_dma_pair(&motor_left.port->current_adc) == KNX_OK) {
        drv8701e_current.adc_alive = 1;
    } else {
        drv8701e_current.timeout_count++;
    }
}

void DRV8701E_Current_Poll(uint32_t timeout_ms)
{
    (void)timeout_ms;

    if (motor_left.port == NULL || motor_right.port == NULL) {
        drv8701e_current.valid = 0;
        return;
    }

    uint16_t rank1_pc0_right_raw = 0;
    uint16_t rank2_pa4_left_raw = 0;
    uint32_t dma_sequence = 0;
    knx_status_t st = knx_adc_read_dma_pair(&motor_left.port->current_adc,
                                            &rank1_pc0_right_raw,
                                            &rank2_pa4_left_raw,
                                            &dma_sequence);

    if (st == KNX_NOT_READY) {
        drv8701e_current.valid = 0;
        return;
    }

    if (st != KNX_OK) {
        drv8701e_current.timeout_count++;
        drv8701e_current.valid = 0;
        drv8701e_current.adc_alive = 0;
        adc_consecutive_timeout++;
        if (adc_consecutive_timeout >= 10) {
            drv8701e_current.adc_restart_count++;
            adc_consecutive_timeout = 0;
            last_adc_dma_sequence = 0;
            (void)knx_adc_restart_dma_pair(&motor_left.port->current_adc);
        }
        return;
    }

    if (dma_sequence == last_adc_dma_sequence) {
        return;
    }
    last_adc_dma_sequence = dma_sequence;

    adc_consecutive_timeout = 0;
    drv8701e_current.adc_alive = 1;

    uint32_t cnt = 0U;
    uint32_t arr = 0U;
    const knx_pwm_channel_t *observer = drv8701e_sample_observer();
    if (observer != NULL) {
        (void)knx_pwm_get_counter(observer, &cnt);
        (void)knx_pwm_get_period(observer, &arr);
    }
    drv8701e_current.tim_cnt_when_read = cnt;
    drv8701e_current.tim_arr = arr;

    drv8701e_current.valid = 1;
    drv8701e_current.sample_count++;

    drv8701e_current.raw_left = rank2_pa4_left_raw;
    drv8701e_current.raw_right = rank1_pc0_right_raw;

    drv8701e_current.voltage_left_mv = (float)drv8701e_current.raw_left
                                       / DRV8701E_ADC_MAX_VALUE
                                       * DRV8701E_ADC_VREF_MV;
    drv8701e_current.voltage_right_mv = (float)drv8701e_current.raw_right
                                        / DRV8701E_ADC_MAX_VALUE
                                        * DRV8701E_ADC_VREF_MV;

    float rsense_ohm = DRV8701E_RSENSE_MOHM / 1000.0f;
    float r_total = rsense_ohm * DRV8701E_CSA_GAIN;

    float current_left = (drv8701e_current.voltage_left_mv / 1000.0f) / r_total - offset_left_a;
    float current_right = (drv8701e_current.voltage_right_mv / 1000.0f) / r_total - offset_right_a;

    drv8701e_current.current_left_a = current_left;
    drv8701e_current.current_right_a = current_right;

    lpf_state_left = lpf_alpha * current_left + (1.0f - lpf_alpha) * lpf_state_left;
    lpf_state_right = lpf_alpha * current_right + (1.0f - lpf_alpha) * lpf_state_right;

    drv8701e_current.current_left_filtered_a = lpf_state_left;
    drv8701e_current.current_right_filtered_a = lpf_state_right;
}

void DRV8701E_Current_CalibrateZero(uint16_t samples, uint32_t timeout_ms)
{
    float sum_left = 0.0f;
    float sum_right = 0.0f;
    uint16_t valid_count = 0;

    float old_offset_left = offset_left_a;
    float old_offset_right = offset_right_a;
    offset_left_a = 0.0f;
    offset_right_a = 0.0f;

    uint32_t per_sample_wait_ms = (timeout_ms > 0U) ? timeout_ms : 1U;
    uint32_t max_wait_ms = ((uint32_t)samples * per_sample_wait_ms * 4U) + 10U;
    uint32_t start_ms = knx_millis();

    while (valid_count < samples && (knx_millis() - start_ms) <= max_wait_ms) {
        DRV8701E_Current_Poll(timeout_ms);
        if (drv8701e_current.valid) {
            sum_left += drv8701e_current.current_left_a;
            sum_right += drv8701e_current.current_right_a;
            valid_count++;
        } else {
            knx_delay_ms(1U);
        }
    }

    if (valid_count > 0) {
        offset_left_a = sum_left / (float)valid_count;
        offset_right_a = sum_right / (float)valid_count;
    } else {
        offset_left_a = old_offset_left;
        offset_right_a = old_offset_right;
    }

    lpf_state_left = 0.0f;
    lpf_state_right = 0.0f;
}

/* ==================== 閫熷害鐜?PID ==================== */

void DRV8701E_VelocityInit(void)
{
    vc_left.kp = KNEXUS_MOTOR_LEFT_KP_DEFAULT;
    vc_left.ki = KNEXUS_MOTOR_LEFT_KI_DEFAULT;
    vc_left.kd = KNEXUS_MOTOR_LEFT_KD_DEFAULT;
    vc_left.target = 0.0f;
    vc_left.target_used = 0.0f;
    vc_left.integral = 0.0f;
    vc_left.prev_error = 0.0f;
    vc_left.feedforward = 0.0f;
    vc_left.correction = 0.0f;
    vc_left.output = 0.0f;
    vc_left.out_min = -6000.0f;
    vc_left.out_max =  6000.0f;
    vc_left.integral_max = KNEXUS_MOTOR_INTEGRAL_MAX_DEFAULT;
    vc_left.last_ramp_tick_ms = knx_millis();
    if (vc_left.rm_pid != NULL) {
        pid_clear((pid_obj_t *)vc_left.rm_pid);
    }

    vc_right.kp = KNEXUS_MOTOR_RIGHT_KP_DEFAULT;
    vc_right.ki = KNEXUS_MOTOR_RIGHT_KI_DEFAULT;
    vc_right.kd = KNEXUS_MOTOR_RIGHT_KD_DEFAULT;
    vc_right.target = 0.0f;
    vc_right.target_used = 0.0f;
    vc_right.integral = 0.0f;
    vc_right.prev_error = 0.0f;
    vc_right.feedforward = 0.0f;
    vc_right.correction = 0.0f;
    vc_right.output = 0.0f;
    vc_right.out_min = -6000.0f;
    vc_right.out_max =  6000.0f;
    vc_right.integral_max = KNEXUS_MOTOR_INTEGRAL_MAX_DEFAULT;
    vc_right.last_ramp_tick_ms = knx_millis();
    if (vc_right.rm_pid != NULL) {
        pid_clear((pid_obj_t *)vc_right.rm_pid);
    }
}

void DRV8701E_VelocityControl(DRV8701E_VelocityCtrl_t *vc,
                               DRV8701E_Motor_t *motor,
                               float speed_actual)
{
    if (vc == NULL || motor == NULL || motor->port == NULL) return;

    float target_cmd = vc->target * motor->target_speed_sign;
    int32_t arr = (int32_t)drv8701e_pwm_period(&motor->port->pwm, motor->port->pwm.arr);
    if (arr <= 0) arr = 1;

    float out_limit = drv8701e_absf(vc->out_max);
    if (drv8701e_absf(vc->out_min) > out_limit) {
        out_limit = drv8701e_absf(vc->out_min);
    }
    if (out_limit > (float)arr) {
        out_limit = (float)arr;
    }

    pid_obj_t *pid = drv8701e_pid_sync(&vc->rm_pid,
                                       vc->kp,
                                       vc->ki,
                                       vc->kd,
                                       out_limit,
                                       vc->integral_max);
    if (pid == NULL) return;

    uint32_t now_ms = knx_millis();
    uint32_t dt_ms = now_ms - vc->last_ramp_tick_ms;
    vc->last_ramp_tick_ms = now_ms;
    if (dt_ms == 0U) {
        dt_ms = 1U;
    }
    if (dt_ms > 100U) {
        dt_ms = 100U;
    }

    float acc_limit = drv8701e_absf(drv8701e_velocity_acc_limit_mps2);
    if (acc_limit <= 0.0001f) {
        vc->target_used = target_cmd;
    } else {
        float max_step = acc_limit * ((float)dt_ms * 0.001f);
        float delta = drv8701e_clampf(target_cmd - vc->target_used, -max_step, max_step);
        vc->target_used += delta;
    }

    if (drv8701e_absf(target_cmd) <= 0.0001f &&
        drv8701e_absf(vc->target_used) <= 0.0001f &&
        drv8701e_absf(speed_actual) <= 0.001f) {
        pid_clear(pid);
        vc->integral = 0.0f;
        vc->prev_error = 0.0f;
        vc->feedforward = 0.0f;
        vc->correction = 0.0f;
        vc->output = 0.0f;
        DRV8701E_SetDuty(motor, 0);
        return;
    }

    pid->Iout = vc->integral;
    float correction = pid_calculate(pid, speed_actual, vc->target_used);
    float feedforward = 0.0f;
    if (drv8701e_absf(vc->target_used) > 0.001f) {
        feedforward = drv8701e_copysignf(drv8701e_velocity_duty_deadzone_cnt,
                                         vc->target_used)
                    + vc->target_used * drv8701e_velocity_kv_cnt_per_mps;
    }
    float output = feedforward + correction;
    if (output >  out_limit) output =  out_limit;
    if (output < -out_limit) output = -out_limit;

    vc->integral = pid->Iout;
    vc->prev_error = pid->Err;
    vc->feedforward = feedforward;
    vc->correction = correction;
    vc->output = output;
    pid->Output = output;

    DRV8701E_SetDuty(motor, (int32_t)output);
}

void DRV8701E_CurrentCtrlInit(void)
{
    cc_left.kp = 0.6f;
    cc_left.ki = 0.09f;
    cc_left.target = 0.0f;
    cc_left.integral = 0.0f;
    cc_left.output = 0.0f;
    cc_left.out_min = 0.0f;
    cc_left.out_max =  1.0f;
    cc_left.integral_max = 20.0f;
    if (cc_left.rm_pid != NULL) {
        pid_clear((pid_obj_t *)cc_left.rm_pid);
    }

    cc_right.kp = 0.6f;
    cc_right.ki = 0.09f;
    cc_right.target = 0.0f;
    cc_right.integral = 0.0f;
    cc_right.output = 0.0f;
    cc_right.out_min = 0.0f;
    cc_right.out_max =  1.0f;
    cc_right.integral_max = 20.0f;
    if (cc_right.rm_pid != NULL) {
        pid_clear((pid_obj_t *)cc_right.rm_pid);
    }
}

void DRV8701E_CurrentControl(DRV8701E_CurrentCtrl_t *cc,
                             DRV8701E_Motor_t *motor,
                             float current_actual)
{
    if (cc == NULL || motor == NULL || motor->port == NULL) return;

    /* Signed target support: negative target = reverse current.
     * PI works on magnitudes; sign is applied to final duty output. */
    float target_sign = (cc->target >= 0.0f) ? 1.0f : -1.0f;
    float target_mag = fabsf(cc->target);
    float actual_mag = fabsf(current_actual);

    pid_obj_t *pid = drv8701e_pid_sync(&cc->rm_pid,
                                       cc->kp,
                                       cc->ki,
                                       0.0f,
                                       cc->out_max,
                                       cc->integral_max);
    if (pid == NULL) return;

    if (target_mag <= 0.0001f) {
        pid_clear(pid);
        cc->integral = 0.0f;
        cc->output = 0.0f;
        DRV8701E_SetDuty(motor, 0);
        return;
    }

    pid->Iout = cc->integral;
    float output = pid_calculate(pid, actual_mag, target_mag);
    if (output > cc->out_max) output = cc->out_max;
    if (output < cc->out_min) output = cc->out_min;

    cc->integral = pid->Iout;
    cc->output = output;
    pid->Output = output;

    int32_t arr = (int32_t)drv8701e_pwm_period(&motor->port->pwm, motor->port->pwm.arr);
    if (arr <= 0) arr = 1;

    float signed_duty = output * target_sign;
    DRV8701E_SetDuty(motor, (int32_t)(signed_duty * (float)arr * motor->dir_sign));
}

void DRV8701E_DebugOcto(Octolinker_Instance_t *octo)
{
    if (octo == NULL) return;

    debug_seq++;

    float left_target_sign = (cc_left.target >= 0.0f) ? 1.0f : -1.0f;
    float right_target_sign = (cc_right.target >= 0.0f) ? 1.0f : -1.0f;

    Octolinker_SendF32(octo, 1, cc_left.target);
    Octolinker_SendF32(octo, 2, drv8701e_current.current_left_filtered_a * left_target_sign);
    Octolinker_SendF32(octo, 3, cc_right.target);
    Octolinker_SendF32(octo, 4, drv8701e_current.current_right_filtered_a * right_target_sign);
}
