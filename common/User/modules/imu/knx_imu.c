#include "knx_imu.h"
#include "bmi088.h"
#include "knx_time.h"
#include "QuaternionEKF.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define KNX_IMU_RAD_TO_DEG 57.29577951308232f
#define KNX_IMU_DEG_TO_RAD 0.01745329251994f

float knx_imu_ekf_enable  = 1.0f;
float knx_imu_ekf_q1      = 10.0f;
float knx_imu_ekf_q2      = 0.001f;
float knx_imu_ekf_r       = 1000000.0f;
float knx_imu_ekf_lambda  = 1.0f;
float knx_imu_ekf_acc_lpf = 0.0085f;

static knx_imu_data_t s_imu_data;
static uint8_t s_ekf_ready = 0;
static uint32_t s_last_update_ms = 0;

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
    memcpy(s_imu_data.gyro, imu_data.gyro, sizeof(s_imu_data.gyro));
    s_imu_data.temperature = imu_data.temperature;
    s_imu_data.accel_ok = imu_data.accel_ok != 0U;
    s_imu_data.gyro_ok = imu_data.gyro_ok != 0U;
    s_imu_data.cali_success = imu_data.cali_success != 0U;
    s_imu_data.cali_retry = imu_data.cali_retry;
    s_imu_data.last_update_ms = knx_millis();
    taskEXIT_CRITICAL();
}

static void copy_ekf_output_to_public(void)
{
    taskENTER_CRITICAL();
    s_imu_data.roll = QEKF_INS.Roll;
    s_imu_data.pitch = QEKF_INS.Pitch;
    s_imu_data.yaw = QEKF_INS.Yaw;
    s_imu_data.roll_total = QEKF_INS.Roll;
    s_imu_data.pitch_total = QEKF_INS.Pitch;
    s_imu_data.yaw_total = QEKF_INS.YawTotalAngle;
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
    copy_driver_common();

    if (knx_imu_ekf_enable <= 0.0f || !imu_data.accel_ok || !imu_data.gyro_ok) {
        copy_driver_attitude_fallback();
        return KNX_OK;
    }

    if (!s_ekf_ready) {
        Kalman_Filter_Deinit(&QEKF_INS.IMU_QuaternionEKF); // 释放旧矩阵,防止内存泄漏
        return knx_imu_init();
    }

    float dt = valid_dt(knx_millis());
    if (dt <= 0.0f) {
        return KNX_OK; // dt无效,跳过本次EKF更新,保留上次有效状态
    }
    QEKF_INS.Q1 = knx_imu_ekf_q1;
    QEKF_INS.Q2 = knx_imu_ekf_q2;
    QEKF_INS.R = knx_imu_ekf_r;
    QEKF_INS.lambda = (knx_imu_ekf_lambda > 1.0f) ? 1.0f : knx_imu_ekf_lambda;
    QEKF_INS.accLPFcoef = knx_imu_ekf_acc_lpf;
    IMU_QuaternionEKF_Update(imu_data.gyro[0],
                             imu_data.gyro[1],
                             imu_data.gyro[2],
                             imu_data.accel[0],
                             imu_data.accel[1],
                             imu_data.accel[2],
                             dt);
    copy_ekf_output_to_public();
    return KNX_OK;
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
