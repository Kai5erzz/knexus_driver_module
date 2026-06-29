/*
 * test_bmi088_imu_est.c - BMI088 IMU attitude estimation test mode
 *
 * Verifies the BMI088 -> knx_imu / EKF -> roll/pitch/yaw output chain.
 * Optionally runs MahonyAHRSupdateIMU in parallel as a roll/pitch reference.
 *
 * EKF yaw is the primary yaw used by the control loop (drift ~12 deg/h).
 * Mahony yaw is a raw-gyro-integration reference only, not a replacement.
 *
 * Debug channels (CH0+):
 *   CH0:  ekf_roll_deg
 *   CH1:  ekf_pitch_deg
 *   CH2:  ekf_yaw_deg
 *   CH3:  ekf_yaw_total_deg
 *   CH4:  gyro_x (rad/s)
 *   CH5:  gyro_y (rad/s)
 *   CH6:  gyro_z (rad/s)
 *   CH7:  accel_x (m/s^2)
 *   CH8:  accel_y (m/s^2)
 *   CH9:  accel_z (m/s^2)
 *   CH10: acc_norm (m/s^2)
 *   CH11: gyro_norm (rad/s)
 *   CH12: imu_valid (1.0 = ok)
 *   CH13: ekf_converged (1.0 = ok)
 *   CH14: ekf_stable (1.0 = ok)
 *   CH15: temperature (C)
 *   CH16: gyro_bias_x (rad/s)
 *   CH17: gyro_bias_y (rad/s)
 *   CH18: gyro_bias_z (rad/s)
 *   CH19: mahony_roll_deg
 *   CH20: mahony_pitch_deg
 *   CH21: mahony_yaw_deg (raw integration reference)
 *   CH22: mahony_q_norm
 *
 * Matrix output:
 *   var_id 100: 8x16 U8_MATRIX �� 5-pointed star, breathing on/off
 *   CH100:      breath phase [0.0 .. 1.0]
 */
#include "test_bmi088_imu_est.h"
#include "knx_imu.h"
#include "knx_time.h"
#include "octolinker.h"
#include "MahonyAHRS.h"
#include <math.h>
#include <string.h>

#define RAD_TO_DEG  57.29577951308232f
#define PI_F        3.14159265f

/* ---- Mahony ---- */
static float s_mahony_q[4];
float test_imu_est_mahony_kp = 0.5f;
float test_imu_est_mahony_ki = 0.0f;
float test_imu_est_debug_hz  = 50.0f;

static Octolinker_Instance_t *s_octo;
static uint32_t s_debug_tick;

/* ---- Breathing star ---- */
#define STAR_ROWS  16
#define STAR_COLS  32
#define STAR_BYTES ((STAR_ROWS * STAR_COLS + 7) / 8)

static const uint8_t s_star_packed[STAR_BYTES] = {
    0x00,0xc0,0x03,0x00,0x00,0xe0,0x07,0x00,0x00,0xe0,0x07,0x00,0xf8,0xff,0xff,0x1f,
    0xf0,0xff,0xff,0x0f,0xe0,0xff,0xff,0x07,0xc0,0xff,0xff,0x03,0x80,0xff,0xff,0x01,
    0x00,0xfe,0x7f,0x00,0x00,0xfc,0x3f,0x00,0x00,0xfc,0x3f,0x00,0x00,0xfc,0x3f,0x00,
    0x00,0xfc,0x3f,0x00,0x00,0xfe,0x7f,0x00,0x00,0x7e,0x7e,0x00,0x00,0x3e,0x7c,0x00,
};

float test_imu_est_breath_hz = 0.5f;  /* breathing frequency (tunable) */
static uint32_t s_breath_start;

/* ---- Euler from quaternion ---- */
static void quat_to_euler_deg(const float q[4], float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    float sinr = 2.0f * (w * x + y * z);
    float cosr = 1.0f - 2.0f * (x * x + y * y);
    *roll_deg = atan2f(sinr, cosr) * RAD_TO_DEG;

    float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f) {
        *pitch_deg = copysignf(90.0f, sinp);
    } else {
        *pitch_deg = asinf(sinp) * RAD_TO_DEG;
    }

    float siny = 2.0f * (w * z + x * y);
    float cosy = 1.0f - 2.0f * (y * y + z * z);
    *yaw_deg = atan2f(siny, cosy) * RAD_TO_DEG;
}

/* ---- Init / Loop ---- */

void test_bmi088_imu_est_init(void *octo)
{
    s_octo = (Octolinker_Instance_t *)octo;
    s_debug_tick = 0U;

    s_mahony_q[0] = 1.0f;
    s_mahony_q[1] = 0.0f;
    s_mahony_q[2] = 0.0f;
    s_mahony_q[3] = 0.0f;

    s_breath_start = knx_millis();
}

void test_bmi088_imu_est_loop(void)
{
    knx_imu_update();

    /* Mahony at 1 kHz */
    {
        const knx_imu_data_t *imu = knx_imu_get();
        if (imu != NULL && imu->accel_ok && imu->gyro_ok) {
            twoKp = 2.0f * test_imu_est_mahony_kp;
            twoKi = 2.0f * test_imu_est_mahony_ki;

            MahonyAHRSupdateIMU(s_mahony_q,
                                imu->gyro[0], imu->gyro[1], imu->gyro[2],
                                imu->accel[0], imu->accel[1], imu->accel[2],
                                0.001f);

            float qn = s_mahony_q[0] * s_mahony_q[0]
                     + s_mahony_q[1] * s_mahony_q[1]
                     + s_mahony_q[2] * s_mahony_q[2]
                     + s_mahony_q[3] * s_mahony_q[3];
            if (qn < 0.5f || qn > 2.0f || qn != qn) {
                s_mahony_q[0] = 1.0f;
                s_mahony_q[1] = 0.0f;
                s_mahony_q[2] = 0.0f;
                s_mahony_q[3] = 0.0f;
            }
        }
    }

    /* Debug output */
    uint32_t now = knx_millis();
    float debug_period_ms = 1000.0f / test_imu_est_debug_hz;
    if (debug_period_ms < 1.0f) debug_period_ms = 1.0f;
    if (now - s_debug_tick < (uint32_t)debug_period_ms) {
        return;
    }
    s_debug_tick = now;

    if (s_octo == NULL) return;

    const knx_imu_data_t *imu = knx_imu_get();
    if (imu == NULL) return;

    float m_roll, m_pitch, m_yaw;
    quat_to_euler_deg(s_mahony_q, &m_roll, &m_pitch, &m_yaw);

    float imu_valid = (imu->accel_ok && imu->gyro_ok && imu->ekf_ready) ? 1.0f : 0.0f;

    /* CH0~CH3: EKF attitude */
    Octolinker_SendF32(s_octo, 0,  imu->roll);
    Octolinker_SendF32(s_octo, 1,  imu->pitch);
    Octolinker_SendF32(s_octo, 2,  imu->yaw);
    Octolinker_SendF32(s_octo, 3,  imu->yaw_total);

    /* CH4~CH6: gyro */
    Octolinker_SendF32(s_octo, 4,  imu->gyro[0]);
    Octolinker_SendF32(s_octo, 5,  imu->gyro[1]);
    Octolinker_SendF32(s_octo, 6,  imu->gyro[2]);

    /* CH7~CH9: accel */
    Octolinker_SendF32(s_octo, 7,  imu->accel[0]);
    Octolinker_SendF32(s_octo, 8,  imu->accel[1]);
    Octolinker_SendF32(s_octo, 9,  imu->accel[2]);

    /* CH10~CH12: norms and validity */
    Octolinker_SendF32(s_octo, 10, imu->accel_norm);
    Octolinker_SendF32(s_octo, 11, imu->gyro_norm);
    Octolinker_SendF32(s_octo, 12, imu_valid);

    /* CH13~CH14: EKF status */
    Octolinker_SendF32(s_octo, 13, imu->ekf_converged ? 1.0f : 0.0f);
    Octolinker_SendF32(s_octo, 14, imu->ekf_stable ? 1.0f : 0.0f);

    /* CH15: temperature */
    Octolinker_SendF32(s_octo, 15, imu->temperature);

    /* CH16~CH18: gyro bias */
    Octolinker_SendF32(s_octo, 16, imu->gyro_bias[0]);
    Octolinker_SendF32(s_octo, 17, imu->gyro_bias[1]);
    Octolinker_SendF32(s_octo, 18, imu->gyro_bias[2]);

    /* CH19~CH22: Mahony reference */
    Octolinker_SendF32(s_octo, 19, m_roll);
    Octolinker_SendF32(s_octo, 20, m_pitch);
    Octolinker_SendF32(s_octo, 21, m_yaw);
    float m_qnorm = sqrtf(s_mahony_q[0] * s_mahony_q[0]
                        + s_mahony_q[1] * s_mahony_q[1]
                        + s_mahony_q[2] * s_mahony_q[2]
                        + s_mahony_q[3] * s_mahony_q[3]);
    Octolinker_SendF32(s_octo, 22, m_qnorm);

    /* ---- Breathing star ---- */
    {
        float hz = test_imu_est_breath_hz;
        if (hz < 0.05f) hz = 0.05f;
        float period_ms = 1000.0f / hz;
        float elapsed = (float)(now - s_breath_start);
        float phase = fmodf(elapsed, period_ms) / period_ms;  /* 0..1 */
        /* breath: smooth sine curve, 0 = dark, 1 = bright */
        float breath = 0.5f * (1.0f - cosf(2.0f * PI_F * phase));

        Octolinker_SendF32(s_octo, 100, breath);

        /* Send pre-packed star as bit matrix, or blank if breath is low */
        if (breath > 0.5f) {
            Octolinker_SendBitMatrix(s_octo, 100, s_star_packed, STAR_ROWS, STAR_COLS);
        } else {
            uint8_t blank[STAR_BYTES];
            memset(blank, 0, sizeof(blank));
            Octolinker_SendBitMatrix(s_octo, 100, blank, STAR_ROWS, STAR_COLS);
        }
    }
}