#ifndef KNX_IMU_H
#define KNX_IMU_H

#include "knx_types.h"

typedef struct {
    float accel[3];       /* m/s^2 */
    float gyro[3];        /* rad/s, raw driver output after static offset */
    float roll;           /* degrees, wrapped */
    float pitch;          /* degrees, wrapped */
    float yaw;            /* degrees, wrapped [-180, 180] */
    float roll_total;     /* degrees, currently mirrors roll */
    float pitch_total;    /* degrees, currently mirrors pitch */
    float yaw_total;      /* degrees, cumulative */
    float q[4];           /* quaternion [w, x, y, z] */
    float gyro_bias[3];   /* EKF-estimated gyro bias, rad/s */
    float gyro_norm;      /* rad/s */
    float accel_norm;     /* m/s^2 */
    float temperature;    /* Celsius */
    bool  accel_ok;
    bool  gyro_ok;
    bool  ekf_ready;
    bool  ekf_converged;
    bool  ekf_stable;
    bool  cali_success;
    uint8_t cali_retry;
    uint32_t last_update_ms;
} knx_imu_data_t;

/* Runtime-tunable EKF parameters. */
extern float knx_imu_ekf_enable;
extern float knx_imu_ekf_q1;
extern float knx_imu_ekf_q2;
extern float knx_imu_ekf_r;
extern float knx_imu_ekf_lambda;
extern float knx_imu_ekf_acc_lpf;

knx_status_t knx_imu_init(void);
knx_status_t knx_imu_update(void);
const knx_imu_data_t *knx_imu_get(void);
void knx_imu_snapshot(knx_imu_data_t *out);
bool knx_imu_is_ready(void);
bool knx_imu_is_fresh(uint32_t timeout_ms);

#endif /* KNX_IMU_H */
