#ifndef DM_IMU_L1_H
#define DM_IMU_L1_H

#include "knx_can.h"
#include "knx_types.h"
#include "octolinker.h"
#include <stdint.h>

#define DM_IMU_L1_DEFAULT_CAN_ID      0x11U
#define DM_IMU_L1_ACTIVE_REPORT_CAN_ID 0x000U
#define DM_IMU_L1_REQUEST_CAN_ID      0x6FFU
#define DM_IMU_L1_CAN_DLC             8U

#define DM_IMU_L1_FLAG_ACCEL          0x01U
#define DM_IMU_L1_FLAG_GYRO           0x02U
#define DM_IMU_L1_FLAG_EULER          0x04U
#define DM_IMU_L1_FLAG_QUATERNION     0x08U
#define DM_IMU_L1_FLAG_ALL_READY      0x0FU

typedef enum {
    DM_IMU_L1_DATA_ACCEL      = 0x01U,
    DM_IMU_L1_DATA_GYRO       = 0x02U,
    DM_IMU_L1_DATA_EULER      = 0x03U,
    DM_IMU_L1_DATA_QUATERNION = 0x04U,
} dm_imu_l1_data_type_t;

typedef enum {
    DM_IMU_L1_RATE_100HZ  = 0x01U,
    DM_IMU_L1_RATE_125HZ  = 0x02U,
    DM_IMU_L1_RATE_200HZ  = 0x03U,
    DM_IMU_L1_RATE_250HZ  = 0x04U,
    DM_IMU_L1_RATE_500HZ  = 0x05U,
    DM_IMU_L1_RATE_1000HZ = 0x06U,
} dm_imu_l1_rate_t;

typedef struct {
    float accel[3];
    float gyro[3];
    float pitch;
    float yaw;
    float roll;
    float yaw_total;
    float roll_total;
    float pitch_total;
    float q[4];
    float temperature;
    float gyro_offset[3];
    float accel_scale;
    float g_norm;
    uint32_t timestamp;
    uint8_t data_flags;
    uint8_t is_calibrated;
    uint32_t frame_count;
    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t error_count;
    uint32_t last_rx_id;
    uint32_t last_tx_id;
    uint8_t last_reg;
    knx_status_t last_status;
} dm_imu_l1_data_t;

extern dm_imu_l1_data_t dm_imu_l1_data;

void DM_IMU_L1_AttachCAN(knx_can_t *can, uint16_t can_id);
knx_status_t DM_IMU_L1_Init(void);
knx_status_t DM_IMU_L1_InitExternalRx(void);
knx_status_t DM_IMU_L1_SetActiveReport(uint8_t enable);
knx_status_t DM_IMU_L1_SetActiveIntervalMs(uint16_t interval_ms);
knx_status_t DM_IMU_L1_SetReportRate(dm_imu_l1_rate_t rate);
knx_status_t DM_IMU_L1_RequestData(dm_imu_l1_data_type_t reg);
void DM_IMU_L1_UpdateData(uint32_t std_id, const uint8_t *data, uint8_t len);
uint8_t DM_IMU_L1_IsDataReady(void);
void DM_IMU_L1_DebugOcto(Octolinker_Instance_t *octo, uint16_t base_id);

#endif /* DM_IMU_L1_H */
