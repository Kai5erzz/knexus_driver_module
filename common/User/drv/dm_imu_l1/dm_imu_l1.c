#include "dm_imu_l1.h"
#include "knx_health.h"
#include "knx_time.h"
#include <stddef.h>
#include <string.h>

#define ACCEL_CAN_MAX        235.2f
#define ACCEL_CAN_MIN       -235.2f
#define GYRO_CAN_MAX          34.88f
#define GYRO_CAN_MIN         -34.88f
#define PITCH_CAN_MAX         90.0f
#define PITCH_CAN_MIN        -90.0f
#define ROLL_CAN_MAX         180.0f
#define ROLL_CAN_MIN        -180.0f
#define YAW_CAN_MAX          180.0f
#define YAW_CAN_MIN         -180.0f
#define QUATERNION_CAN_MAX     1.0f
#define QUATERNION_CAN_MIN    -1.0f

#define DM_IMU_L1_REG_ACTIVE_INTERVAL 0x0AU
#define DM_IMU_L1_REG_ACTIVE_REPORT   0x0BU
#define DM_IMU_L1_CMD_HEAD          0xCCU
#define DM_IMU_L1_CMD_TAIL          0xDDU
#define DM_IMU_L1_CMD_READ          0x00U
#define DM_IMU_L1_CMD_WRITE         0x01U
#define DM_IMU_L1_BOOT_DELAY_MS     300U
#define DM_IMU_L1_CMD_GAP_MS        10U

dm_imu_l1_data_t dm_imu_l1_data;

static knx_can_t *s_can;
static uint16_t s_can_id = DM_IMU_L1_DEFAULT_CAN_ID;

static float dm_imu_l1_uint_to_float(uint32_t value, float min, float max, uint8_t bits)
{
    float span = max - min;
    float scale = (float)((1UL << bits) - 1UL);
    return ((float)value * span / scale) + min;
}

static void dm_imu_l1_rx_dispatch(uint32_t std_id,
                                  const uint8_t *data,
                                  uint8_t len,
                                  void *user)
{
    (void)user;
    DM_IMU_L1_UpdateData(std_id, data, len);
}

static void dm_imu_l1_reset_data(void)
{
    memset(&dm_imu_l1_data, 0, sizeof(dm_imu_l1_data));
    dm_imu_l1_data.accel_scale = 1.0f;
    dm_imu_l1_data.g_norm = 9.81f;
}

static knx_status_t dm_imu_l1_send_write(uint8_t reg, const uint8_t *data, uint8_t len)
{
    if (s_can == NULL) {
        dm_imu_l1_data.last_status = KNX_NOT_READY;
        return KNX_NOT_READY;
    }
    if (len > 4U || (len != 0U && data == NULL)) {
        dm_imu_l1_data.last_status = KNX_INVALID_ARG;
        return KNX_INVALID_ARG;
    }

    uint8_t tx[DM_IMU_L1_CAN_DLC] = {0};
    tx[0] = DM_IMU_L1_CMD_HEAD;
    tx[1] = reg;
    tx[2] = DM_IMU_L1_CMD_WRITE;
    for (uint8_t i = 0; i < len; i++) {
        tx[3U + i] = data[i];
    }
    tx[7] = DM_IMU_L1_CMD_TAIL;

    knx_status_t status = knx_can_transmit_std(s_can, s_can_id, tx, DM_IMU_L1_CAN_DLC, 2U);
    dm_imu_l1_data.last_tx_id = s_can_id;
    dm_imu_l1_data.last_status = status;
    if (status == KNX_OK) {
        dm_imu_l1_data.tx_count++;
    } else {
        dm_imu_l1_data.error_count++;
    }
    return status;
}

static void dm_imu_l1_parse_accel(const uint8_t *data)
{
    uint16_t raw_x = ((uint16_t)data[3] << 8) | data[2];
    uint16_t raw_y = ((uint16_t)data[5] << 8) | data[4];
    uint16_t raw_z = ((uint16_t)data[7] << 8) | data[6];

    dm_imu_l1_data.accel[0] = dm_imu_l1_uint_to_float(raw_x, ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16U)
                            * dm_imu_l1_data.accel_scale;
    dm_imu_l1_data.accel[1] = dm_imu_l1_uint_to_float(raw_y, ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16U)
                            * dm_imu_l1_data.accel_scale;
    dm_imu_l1_data.accel[2] = dm_imu_l1_uint_to_float(raw_z, ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16U)
                            * dm_imu_l1_data.accel_scale;
    dm_imu_l1_data.temperature = (float)data[1];
    dm_imu_l1_data.data_flags |= DM_IMU_L1_FLAG_ACCEL;
}

static void dm_imu_l1_parse_gyro(const uint8_t *data)
{
    uint16_t raw_x = ((uint16_t)data[3] << 8) | data[2];
    uint16_t raw_y = ((uint16_t)data[5] << 8) | data[4];
    uint16_t raw_z = ((uint16_t)data[7] << 8) | data[6];

    dm_imu_l1_data.gyro[0] = dm_imu_l1_uint_to_float(raw_x, GYRO_CAN_MIN, GYRO_CAN_MAX, 16U)
                           - dm_imu_l1_data.gyro_offset[0];
    dm_imu_l1_data.gyro[1] = dm_imu_l1_uint_to_float(raw_y, GYRO_CAN_MIN, GYRO_CAN_MAX, 16U)
                           - dm_imu_l1_data.gyro_offset[1];
    dm_imu_l1_data.gyro[2] = dm_imu_l1_uint_to_float(raw_z, GYRO_CAN_MIN, GYRO_CAN_MAX, 16U)
                           - dm_imu_l1_data.gyro_offset[2];
    dm_imu_l1_data.data_flags |= DM_IMU_L1_FLAG_GYRO;
}

static void dm_imu_l1_parse_euler(const uint8_t *data)
{
    uint16_t raw_pitch = ((uint16_t)data[3] << 8) | data[2];
    uint16_t raw_yaw = ((uint16_t)data[5] << 8) | data[4];
    uint16_t raw_roll = ((uint16_t)data[7] << 8) | data[6];

    float pitch = dm_imu_l1_uint_to_float(raw_pitch, PITCH_CAN_MIN, PITCH_CAN_MAX, 16U);
    float yaw = dm_imu_l1_uint_to_float(raw_yaw, YAW_CAN_MIN, YAW_CAN_MAX, 16U);
    float roll = dm_imu_l1_uint_to_float(raw_roll, ROLL_CAN_MIN, ROLL_CAN_MAX, 16U);

    float dyaw = yaw - dm_imu_l1_data.yaw;
    if (dyaw > 180.0f) {
        dyaw -= 360.0f;
    } else if (dyaw < -180.0f) {
        dyaw += 360.0f;
    }

    float droll = roll - dm_imu_l1_data.roll;
    if (droll > 180.0f) {
        droll -= 360.0f;
    } else if (droll < -180.0f) {
        droll += 360.0f;
    }

    float dpitch = pitch - dm_imu_l1_data.pitch;
    if (dpitch > 90.0f) {
        dpitch -= 180.0f;
    } else if (dpitch < -90.0f) {
        dpitch += 180.0f;
    }

    dm_imu_l1_data.pitch = pitch;
    dm_imu_l1_data.yaw = yaw;
    dm_imu_l1_data.roll = roll;
    dm_imu_l1_data.pitch_total += dpitch;
    dm_imu_l1_data.yaw_total += dyaw;
    dm_imu_l1_data.roll_total += droll;
    dm_imu_l1_data.data_flags |= DM_IMU_L1_FLAG_EULER;
}

static void dm_imu_l1_parse_quaternion(const uint8_t *data)
{
    uint16_t raw_w = ((uint16_t)data[1] << 6) | ((uint16_t)(data[2] & 0xFCU) >> 2);
    uint16_t raw_x = ((uint16_t)(data[2] & 0x03U) << 12) | ((uint16_t)data[3] << 4)
                   | ((uint16_t)(data[4] & 0xF0U) >> 4);
    uint16_t raw_y = ((uint16_t)(data[4] & 0x0FU) << 10) | ((uint16_t)data[5] << 2)
                   | ((uint16_t)(data[6] & 0xC0U) >> 6);
    uint16_t raw_z = ((uint16_t)(data[6] & 0x3FU) << 8) | data[7];

    dm_imu_l1_data.q[0] = dm_imu_l1_uint_to_float(raw_w, QUATERNION_CAN_MIN, QUATERNION_CAN_MAX, 14U);
    dm_imu_l1_data.q[1] = dm_imu_l1_uint_to_float(raw_x, QUATERNION_CAN_MIN, QUATERNION_CAN_MAX, 14U);
    dm_imu_l1_data.q[2] = dm_imu_l1_uint_to_float(raw_y, QUATERNION_CAN_MIN, QUATERNION_CAN_MAX, 14U);
    dm_imu_l1_data.q[3] = dm_imu_l1_uint_to_float(raw_z, QUATERNION_CAN_MIN, QUATERNION_CAN_MAX, 14U);
    dm_imu_l1_data.data_flags |= DM_IMU_L1_FLAG_QUATERNION;
}

void DM_IMU_L1_AttachCAN(knx_can_t *can, uint16_t can_id)
{
    s_can = can;
    s_can_id = can_id;
}

knx_status_t DM_IMU_L1_Init(void)
{
    if (s_can == NULL) {
        dm_imu_l1_data.last_status = KNX_NOT_READY;
        return KNX_NOT_READY;
    }

    dm_imu_l1_reset_data();

    knx_status_t status = knx_can_set_rx_callback(s_can, dm_imu_l1_rx_dispatch, NULL);
    if (status != KNX_OK) {
        dm_imu_l1_data.last_status = status;
        return status;
    }

    status = knx_can_start(s_can);
    if (status != KNX_OK) {
        dm_imu_l1_data.last_status = status;
        (void)knx_health_report(KNX_HEALTH_SOURCE_DM_IMU_L1,
                                KNX_HEALTH_STATE_FAULT,
                                status,
                                0U,
                                dm_imu_l1_data.error_count);
        return status;
    }

    knx_delay_ms(DM_IMU_L1_BOOT_DELAY_MS);

    status = DM_IMU_L1_SetReportRate(DM_IMU_L1_RATE_1000HZ);
    if (status != KNX_OK) {
        return status;
    }

    knx_delay_ms(DM_IMU_L1_CMD_GAP_MS);

    return DM_IMU_L1_SetActiveReport(1U);
}

knx_status_t DM_IMU_L1_InitExternalRx(void)
{
    if (s_can == NULL) {
        dm_imu_l1_data.last_status = KNX_NOT_READY;
        return KNX_NOT_READY;
    }

    dm_imu_l1_reset_data();
    knx_delay_ms(DM_IMU_L1_BOOT_DELAY_MS);

    knx_status_t status = DM_IMU_L1_SetReportRate(DM_IMU_L1_RATE_1000HZ);
    if (status != KNX_OK) {
        (void)knx_health_report(KNX_HEALTH_SOURCE_DM_IMU_L1,
                                KNX_HEALTH_STATE_WARN,
                                status,
                                0U,
                                dm_imu_l1_data.error_count);
        return status;
    }

    knx_delay_ms(DM_IMU_L1_CMD_GAP_MS);
    status = DM_IMU_L1_SetActiveReport(1U);
    (void)knx_health_report(KNX_HEALTH_SOURCE_DM_IMU_L1,
                            (status == KNX_OK) ? KNX_HEALTH_STATE_STALE : KNX_HEALTH_STATE_WARN,
                            status,
                            0U,
                            dm_imu_l1_data.error_count);
    return status;
}

knx_status_t DM_IMU_L1_SetActiveReport(uint8_t enable)
{
    uint8_t data = (enable != 0U) ? 0x01U : 0x00U;
    return dm_imu_l1_send_write(DM_IMU_L1_REG_ACTIVE_REPORT, &data, 1U);
}

knx_status_t DM_IMU_L1_SetActiveIntervalMs(uint16_t interval_ms)
{
    if (interval_ms == 0U) {
        dm_imu_l1_data.last_status = KNX_INVALID_ARG;
        return KNX_INVALID_ARG;
    }

    uint8_t data[4];
    data[0] = (uint8_t)(interval_ms & 0xFFU);
    data[1] = (uint8_t)(interval_ms >> 8);
    data[2] = 0U;
    data[3] = 0U;
    return dm_imu_l1_send_write(DM_IMU_L1_REG_ACTIVE_INTERVAL, data, sizeof(data));
}

knx_status_t DM_IMU_L1_SetReportRate(dm_imu_l1_rate_t rate)
{
    uint16_t interval_ms;

    switch (rate) {
    case DM_IMU_L1_RATE_100HZ:
        interval_ms = 10U;
        break;
    case DM_IMU_L1_RATE_125HZ:
        interval_ms = 8U;
        break;
    case DM_IMU_L1_RATE_200HZ:
        interval_ms = 5U;
        break;
    case DM_IMU_L1_RATE_250HZ:
        interval_ms = 4U;
        break;
    case DM_IMU_L1_RATE_500HZ:
        interval_ms = 2U;
        break;
    case DM_IMU_L1_RATE_1000HZ:
        interval_ms = 1U;
        break;
    default:
        dm_imu_l1_data.last_status = KNX_INVALID_ARG;
        return KNX_INVALID_ARG;
    }

    return DM_IMU_L1_SetActiveIntervalMs(interval_ms);
}

knx_status_t DM_IMU_L1_RequestData(dm_imu_l1_data_type_t reg)
{
    if (s_can == NULL) {
        dm_imu_l1_data.last_status = KNX_NOT_READY;
        return KNX_NOT_READY;
    }

    uint8_t rid = (uint8_t)reg;
    if (rid < (uint8_t)DM_IMU_L1_DATA_ACCEL || rid > (uint8_t)DM_IMU_L1_DATA_QUATERNION) {
        dm_imu_l1_data.last_status = KNX_INVALID_ARG;
        return KNX_INVALID_ARG;
    }

    uint8_t tx[4];
    tx[0] = DM_IMU_L1_CMD_HEAD;
    tx[1] = rid;
    tx[2] = DM_IMU_L1_CMD_READ;
    tx[3] = DM_IMU_L1_CMD_TAIL;

    knx_status_t status = knx_can_transmit_std(s_can, s_can_id, tx, sizeof(tx), 2U);
    dm_imu_l1_data.last_tx_id = s_can_id;
    dm_imu_l1_data.last_status = status;
    if (status == KNX_OK) {
        dm_imu_l1_data.tx_count++;
    } else {
        dm_imu_l1_data.error_count++;
    }
    return status;
}

void DM_IMU_L1_UpdateData(uint32_t std_id, const uint8_t *data, uint8_t len)
{
    if (data == NULL || len < DM_IMU_L1_CAN_DLC) {
        return;
    }

    switch (data[0]) {
    case DM_IMU_L1_DATA_ACCEL:
        dm_imu_l1_parse_accel(data);
        break;
    case DM_IMU_L1_DATA_GYRO:
        dm_imu_l1_parse_gyro(data);
        break;
    case DM_IMU_L1_DATA_EULER:
        dm_imu_l1_parse_euler(data);
        break;
    case DM_IMU_L1_DATA_QUATERNION:
        dm_imu_l1_parse_quaternion(data);
        break;
    default:
        dm_imu_l1_data.error_count++;
        (void)knx_health_report(KNX_HEALTH_SOURCE_DM_IMU_L1,
                                KNX_HEALTH_STATE_WARN,
                                KNX_INVALID_ARG,
                                data[0],
                                dm_imu_l1_data.error_count);
        return;
    }

    dm_imu_l1_data.last_rx_id = std_id;
    dm_imu_l1_data.last_reg = data[0];
    dm_imu_l1_data.timestamp = knx_millis();
    dm_imu_l1_data.frame_count++;
    dm_imu_l1_data.rx_count++;
    dm_imu_l1_data.last_status = KNX_OK;
    (void)knx_health_report(KNX_HEALTH_SOURCE_DM_IMU_L1,
                            KNX_HEALTH_STATE_OK,
                            KNX_OK,
                            dm_imu_l1_data.data_flags,
                            dm_imu_l1_data.error_count);
}

uint8_t DM_IMU_L1_IsDataReady(void)
{
    return (dm_imu_l1_data.data_flags & DM_IMU_L1_FLAG_ALL_READY) == DM_IMU_L1_FLAG_ALL_READY;
}

void DM_IMU_L1_DebugOcto(Octolinker_Instance_t *octo, uint16_t base_id)
{
    if (octo == NULL) {
        return;
    }

    (void)Octolinker_SendF32(octo, base_id + 0U, dm_imu_l1_data.roll);
    (void)Octolinker_SendF32(octo, base_id + 1U, dm_imu_l1_data.pitch);
    (void)Octolinker_SendF32(octo, base_id + 2U, dm_imu_l1_data.yaw);
    (void)Octolinker_SendF32(octo, base_id + 3U, dm_imu_l1_data.yaw_total);
    (void)Octolinker_SendF32(octo, base_id + 4U, dm_imu_l1_data.temperature);
    (void)Octolinker_SendF32Array(octo, base_id + 5U, dm_imu_l1_data.accel, 3U);
    (void)Octolinker_SendF32Array(octo, base_id + 6U, dm_imu_l1_data.gyro, 3U);
    (void)Octolinker_SendF32Array(octo, base_id + 7U, dm_imu_l1_data.q, 4U);
    (void)Octolinker_SendU32(octo, base_id + 8U, dm_imu_l1_data.rx_count);
    (void)Octolinker_SendU32(octo, base_id + 9U, dm_imu_l1_data.tx_count);
    (void)Octolinker_SendU32(octo, base_id + 10U, dm_imu_l1_data.error_count);
    (void)Octolinker_SendU32(octo, base_id + 11U, dm_imu_l1_data.last_rx_id);
    (void)Octolinker_SendU8(octo, base_id + 12U, dm_imu_l1_data.last_reg);
    (void)Octolinker_SendU8(octo, base_id + 13U, dm_imu_l1_data.data_flags);
    (void)Octolinker_SendI32(octo, base_id + 14U, (int32_t)dm_imu_l1_data.last_status);
}
