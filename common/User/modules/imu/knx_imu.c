#include "knx_imu.h"
#include "knexus_config.h"
#include "bmi088.h"
#include "knx_time.h"
#include "QuaternionEKF.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define KNX_IMU_RAD_TO_DEG 57.29577951308232f
#define KNX_IMU_DEG_TO_RAD 0.01745329251994f

float knx_imu_ekf_enable  = KNEXUS_IMU_EKF_ENABLE_DEFAULT;
float knx_imu_ekf_q1      = 10.0f;
float knx_imu_ekf_q2      = 0.001f;
float knx_imu_ekf_r       = 1000000.0f;
float knx_imu_ekf_lambda  = 1.0f;
float knx_imu_ekf_acc_lpf = 0.0085f;
float knx_imu_zero_drift_enable = KNEXUS_IMU_ZERO_DRIFT_ENABLE_DEFAULT;
float knx_imu_zero_drift_gyro_enter = KNEXUS_IMU_ZERO_DRIFT_GYRO_ENTER_DEFAULT;
float knx_imu_zero_drift_accel_enter = KNEXUS_IMU_ZERO_DRIFT_ACCEL_ENTER_DEFAULT;
float knx_imu_zero_drift_bias_tau_s = KNEXUS_IMU_ZERO_DRIFT_BIAS_TAU_S_DEFAULT;
float knx_imu_zero_drift_default_bias_z = KNEXUS_IMU_ZERO_DRIFT_BIAS_Z_DEFAULT;
uint32_t knx_imu_zero_drift_confirm_ms = KNEXUS_IMU_ZERO_DRIFT_CONFIRM_MS_DEFAULT;

static knx_imu_data_t s_imu_data;
static uint8_t s_ekf_ready = 0;
static uint32_t s_last_update_ms = 0;
static volatile bool s_stationary_hint = true;
static float s_runtime_gyro_bias[3];
static float s_corrected_gyro[3];
static float s_zero_sum[3];
static uint32_t s_zero_count;
static uint32_t s_zero_candidate_ms;
static uint32_t s_zero_dwell_ms;
static bool s_zero_stationary;
static bool s_zero_temp_ready;
static bool s_yaw_stationary_prev;
static float s_yaw_hold_total_deg;
static float s_yaw_output_offset_deg;

static void zero_drift_reset_candidate(void)
{
    s_zero_candidate_ms = 0U;
    s_zero_dwell_ms = 0U;
    s_zero_count = 0U;
    memset(s_zero_sum, 0, sizeof(s_zero_sum));
}

static void zero_drift_update(uint32_t now_ms, float dt_s)
{
    const float gx = imu_data.gyro[0];
    const float gy = imu_data.gyro[1];
    const float gz = imu_data.gyro[2];

    if (knx_imu_zero_drift_enable < 0.5f ||
        !imu_data.accel_ok || !imu_data.gyro_ok) {
        s_zero_stationary = false;
        s_zero_temp_ready = false;
        zero_drift_reset_candidate();
        for (uint8_t i = 0U; i < 3U; ++i) {
            s_corrected_gyro[i] = imu_data.gyro[i] - s_runtime_gyro_bias[i];
        }
        return;
    }

    float accel_norm = sqrtf(imu_data.accel[0] * imu_data.accel[0] +
                             imu_data.accel[1] * imu_data.accel[1] +
                             imu_data.accel[2] * imu_data.accel[2]);
    float gyro_norm = sqrtf(gx * gx + gy * gy + gz * gz);
    float accel_error = fabsf(accel_norm - 9.81f);
    s_zero_temp_ready = fabsf(imu_data.temperature - bmi088_temp_target_c) <= 2.0f;

    bool enter_ok = s_stationary_hint && s_zero_temp_ready &&
                    gyro_norm <= knx_imu_zero_drift_gyro_enter &&
                    accel_error <= knx_imu_zero_drift_accel_enter;
    bool hold_ok = s_stationary_hint && s_zero_temp_ready &&
                   gyro_norm <= 0.060f && accel_error <= 0.30f;

    if (!s_zero_stationary) {
        if (!enter_ok) {
            zero_drift_reset_candidate();
        } else {
            if (s_zero_candidate_ms == 0U) {
                s_zero_candidate_ms = now_ms;
                memset(s_zero_sum, 0, sizeof(s_zero_sum));
                s_zero_count = 0U;
            }
            s_zero_sum[0] += gx;
            s_zero_sum[1] += gy;
            s_zero_sum[2] += gz;
            s_zero_count++;
            s_zero_dwell_ms = now_ms - s_zero_candidate_ms;
            uint32_t confirm_ms = knx_imu_zero_drift_confirm_ms;
            if (confirm_ms < 100U) confirm_ms = 100U;
            if (s_zero_dwell_ms >= confirm_ms && s_zero_count > 0U) {
                float inv_count = 1.0f / (float)s_zero_count;
                for (uint8_t i = 0U; i < 3U; ++i) {
                    s_runtime_gyro_bias[i] = s_zero_sum[i] * inv_count;
                }
                s_zero_stationary = true;
            }
        }
    } else if (!hold_ok) {
        s_zero_stationary = false;
        zero_drift_reset_candidate();
    } else {
        float tau = knx_imu_zero_drift_bias_tau_s;
        if (tau < 1.0f) tau = 1.0f;
        float alpha = dt_s / (tau + dt_s);
        s_runtime_gyro_bias[0] += alpha * (gx - s_runtime_gyro_bias[0]);
        s_runtime_gyro_bias[1] += alpha * (gy - s_runtime_gyro_bias[1]);
        s_runtime_gyro_bias[2] += alpha * (gz - s_runtime_gyro_bias[2]);
        if (s_zero_candidate_ms != 0U) {
            s_zero_dwell_ms = now_ms - s_zero_candidate_ms;
        }
    }

    for (uint8_t i = 0U; i < 3U; ++i) {
        s_corrected_gyro[i] = imu_data.gyro[i] - s_runtime_gyro_bias[i];
        if (s_zero_stationary) {
            s_corrected_gyro[i] = 0.0f;
        }
    }
}

static void euler_to_quat(float roll_deg, float pitch_deg, float yaw_deg, float q[4])
{
    float roll = roll_deg * KNX_IMU_DEG_TO_RAD;
    float pitch = pitch_deg * KNX_IMU_DEG_TO_RAD;
    float yaw = yaw_deg * KNX_IMU_DEG_TO_RAD;

    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

static void init_quat_from_accel(float q[4])
{
    float ax = imu_data.accel[0];
    float ay = imu_data.accel[1];
    float az = imu_data.accel[2];
    float norm = sqrtf(ax * ax + ay * ay + az * az);

    if (norm < 1.0f || !imu_data.accel_ok) {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }

    float roll = atan2f(ay, az) * KNX_IMU_RAD_TO_DEG;
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * KNX_IMU_RAD_TO_DEG;
    euler_to_quat(roll, pitch, 0.0f, q);
}

static float valid_dt(uint32_t now_ms)
{
    float dt = (s_last_update_ms == 0U) ? 0.001f
                                        : (float)(now_ms - s_last_update_ms) * 0.001f;
    if (!isfinite(dt) || dt <= 0.0f) {
        return -1.0f; // 信号:跳过本次更新,不更新时间戳
    }
    if (dt > 0.05f) {
        dt = 0.01f; // 大dt钳位到标称周期(10ms)而非1ms
    }
    s_last_update_ms = now_ms;
    return dt;
}

static void copy_driver_common(void)
{
    taskENTER_CRITICAL();
    memcpy(s_imu_data.accel, imu_data.accel, sizeof(s_imu_data.accel));
    memcpy(s_imu_data.gyro, s_corrected_gyro, sizeof(s_imu_data.gyro));
    memcpy(s_imu_data.gyro_runtime_bias, s_runtime_gyro_bias,
           sizeof(s_imu_data.gyro_runtime_bias));
    s_imu_data.temperature = imu_data.temperature;
    s_imu_data.accel_ok = imu_data.accel_ok != 0U;
    s_imu_data.gyro_ok = imu_data.gyro_ok != 0U;
    s_imu_data.cali_success = imu_data.cali_success != 0U;
    s_imu_data.cali_retry = imu_data.cali_retry;
    s_imu_data.zero_drift_stationary = s_zero_stationary;
    s_imu_data.zero_drift_temp_ready = s_zero_temp_ready;
    s_imu_data.zero_drift_dwell_ms = s_zero_dwell_ms;
    s_imu_data.last_update_ms = knx_millis();
    taskEXIT_CRITICAL();
}

static void copy_ekf_output_to_public(void)
{
    float internal_yaw_total = QEKF_INS.YawTotalAngle;
    float output_yaw_total;
    if (s_zero_stationary) {
        if (!s_yaw_stationary_prev) {
            s_yaw_hold_total_deg = internal_yaw_total + s_yaw_output_offset_deg;
        }
        output_yaw_total = s_yaw_hold_total_deg;
    } else {
        if (s_yaw_stationary_prev) {
            s_yaw_output_offset_deg = s_yaw_hold_total_deg - internal_yaw_total;
        }
        output_yaw_total = internal_yaw_total + s_yaw_output_offset_deg;
    }
    s_yaw_stationary_prev = s_zero_stationary;

    float output_yaw = output_yaw_total;
    while (output_yaw > 180.0f) output_yaw -= 360.0f;
    while (output_yaw < -180.0f) output_yaw += 360.0f;

    taskENTER_CRITICAL();
    s_imu_data.roll = QEKF_INS.Roll;
    s_imu_data.pitch = QEKF_INS.Pitch;
    s_imu_data.yaw = output_yaw;
    s_imu_data.roll_total = QEKF_INS.Roll;
    s_imu_data.pitch_total = QEKF_INS.Pitch;
    s_imu_data.yaw_total = output_yaw_total;
    memcpy(s_imu_data.q, QEKF_INS.q, sizeof(s_imu_data.q));
    memcpy(s_imu_data.gyro_bias, QEKF_INS.GyroBias, sizeof(s_imu_data.gyro_bias));
    s_imu_data.gyro_norm = QEKF_INS.gyro_norm;
    s_imu_data.accel_norm = QEKF_INS.accl_norm;
    s_imu_data.ekf_ready = s_ekf_ready != 0U;
    s_imu_data.ekf_converged = QEKF_INS.ConvergeFlag != 0U;
    s_imu_data.ekf_stable = QEKF_INS.StableFlag != 0U;
    taskEXIT_CRITICAL();

    taskENTER_CRITICAL();
    imu_data.roll = s_imu_data.roll;
    imu_data.pitch = s_imu_data.pitch;
    imu_data.yaw = s_imu_data.yaw;
    imu_data.roll_total = s_imu_data.roll_total;
    imu_data.pitch_total = s_imu_data.pitch_total;
    imu_data.yaw_total = s_imu_data.yaw_total;
    memcpy(imu_data.q, s_imu_data.q, sizeof(imu_data.q));
    taskEXIT_CRITICAL();
}

static void copy_driver_attitude_fallback(void)
{
    taskENTER_CRITICAL();
    s_imu_data.roll = imu_data.roll;
    s_imu_data.pitch = imu_data.pitch;
    s_imu_data.yaw = imu_data.yaw;
    s_imu_data.roll_total = imu_data.roll_total;
    s_imu_data.pitch_total = imu_data.pitch_total;
    s_imu_data.yaw_total = imu_data.yaw_total;
    memcpy(s_imu_data.q, imu_data.q, sizeof(s_imu_data.q));
    s_imu_data.ekf_ready = false;
    s_imu_data.ekf_converged = false;
    s_imu_data.ekf_stable = false;
    taskEXIT_CRITICAL();
}

knx_status_t knx_imu_init(void)
{
    BMI088_Read();
    memset(s_runtime_gyro_bias, 0, sizeof(s_runtime_gyro_bias));
    s_runtime_gyro_bias[2] = knx_imu_zero_drift_default_bias_z;
    memcpy(s_corrected_gyro, imu_data.gyro, sizeof(s_corrected_gyro));
    s_zero_stationary = false;
    s_zero_temp_ready = false;
    s_yaw_stationary_prev = false;
    s_yaw_hold_total_deg = 0.0f;
    s_yaw_output_offset_deg = 0.0f;
    zero_drift_reset_candidate();
    copy_driver_common();

    float init_q[4];
    init_quat_from_accel(init_q);
    memset(&QEKF_INS, 0, sizeof(QEKF_INS));
    IMU_QuaternionEKF_Init(init_q,
                           knx_imu_ekf_q1,
                           knx_imu_ekf_q2,
                           knx_imu_ekf_r,
                           knx_imu_ekf_lambda,
                           knx_imu_ekf_acc_lpf);
    s_ekf_ready = 1;
    s_last_update_ms = knx_millis();
    copy_ekf_output_to_public();
    return KNX_OK;
}

knx_status_t knx_imu_update(void)
{
    BMI088_Read();
    uint32_t now_ms = knx_millis();
    float dt = valid_dt(now_ms);
    zero_drift_update(now_ms, (dt > 0.0f) ? dt : 0.01f);
    copy_driver_common();

    if (knx_imu_ekf_enable <= 0.0f || !imu_data.accel_ok || !imu_data.gyro_ok) {
        copy_driver_attitude_fallback();
        return KNX_OK;
    }

    if (!s_ekf_ready) {
        Kalman_Filter_Deinit(&QEKF_INS.IMU_QuaternionEKF); // 释放旧矩阵,防止内存泄漏
        return knx_imu_init();
    }

    if (dt <= 0.0f) {
        return KNX_OK; // dt无效,跳过本次EKF更新,保留上次有效状态
    }
    QEKF_INS.Q1 = knx_imu_ekf_q1;
    QEKF_INS.Q2 = knx_imu_ekf_q2;
    QEKF_INS.R = knx_imu_ekf_r;
    QEKF_INS.lambda = (knx_imu_ekf_lambda > 1.0f) ? 1.0f : knx_imu_ekf_lambda;
    QEKF_INS.accLPFcoef = knx_imu_ekf_acc_lpf;
    IMU_QuaternionEKF_Update(s_corrected_gyro[0],
                             s_corrected_gyro[1],
                             s_corrected_gyro[2],
                             imu_data.accel[0],
                             imu_data.accel[1],
                             imu_data.accel[2],
                             dt);
    copy_ekf_output_to_public();
    return KNX_OK;
}

void knx_imu_set_stationary_hint(bool stationary)
{
    s_stationary_hint = stationary;
}

const knx_imu_data_t *knx_imu_get(void)
{
    return &s_imu_data;
}

void knx_imu_snapshot(knx_imu_data_t *out)
{
    if (out == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *out = s_imu_data;
    taskEXIT_CRITICAL();
}

bool knx_imu_is_ready(void)
{
    return s_imu_data.accel_ok &&
           s_imu_data.gyro_ok &&
           s_imu_data.ekf_ready;
}

bool knx_imu_is_fresh(uint32_t timeout_ms)
{
    return (knx_millis() - s_imu_data.last_update_ms) <= timeout_ms;
}
