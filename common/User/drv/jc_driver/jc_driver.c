#include "jc_driver.h"
#include "knx_health.h"
#include "knx_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stddef.h>

#define JC_DEFAULT_TIMEOUT_MS 20U

static knx_can_t *s_can = NULL;
static volatile uint8_t s_rx_flag = 0U;
static volatile uint8_t s_last_motor_id = 0U;
static volatile uint16_t s_last_reg = 0U;
static volatile JC_Reply s_last_reply;
static JC_Stats s_stats;
static uint32_t s_timeout_ms = JC_DEFAULT_TIMEOUT_MS;

static int32_t jc_sign_extend_24(uint32_t value)
{
    if ((value & 0x800000U) != 0U) {
        value |= 0xFF000000U;
    }
    return (int32_t)value;
}

static void jc_can_rx_dispatch(uint32_t std_id,
                               const uint8_t *data,
                               uint8_t len,
                               void *user)
{
    (void)user;
    JC_RxHandler(std_id, data, len);
}

void JC_AttachCAN(knx_can_t *can)
{
    s_can = can;
}

knx_status_t JC_Init(void)
{
    if (s_can == NULL) {
        s_stats.last_status = KNX_NOT_READY;
        return KNX_NOT_READY;
    }

    knx_status_t status = knx_can_set_rx_callback(s_can, jc_can_rx_dispatch, NULL);
    if (status != KNX_OK) {
        s_stats.last_status = status;
        return status;
    }

    status = knx_can_start(s_can);
    s_stats.last_status = status;
    return status;
}

knx_status_t JC_InitExternalRx(void)
{
    if (s_can == NULL) {
        s_stats.last_status = KNX_NOT_READY;
        return KNX_NOT_READY;
    }

    s_stats.last_status = KNX_OK;
    (void)knx_health_report(KNX_HEALTH_SOURCE_JC,
                            KNX_HEALTH_STATE_STALE,
                            KNX_OK,
                            0U,
                            s_stats.error_count + s_stats.timeout_count);
    return KNX_OK;
}

void JC_SetTimeoutMs(uint32_t timeout_ms)
{
    if (timeout_ms == 0U) {
        timeout_ms = 1U;
    }
    s_timeout_ms = timeout_ms;
}

uint32_t JC_GetTimeoutMs(void)
{
    return s_timeout_ms;
}

void JC_RxHandler(uint32_t can_id, const uint8_t *rx_data, uint8_t len)
{
    if (rx_data == NULL || len < JC_CAN_DLC) {
        return;
    }
    if (can_id < JC_RX_ID_BASE || can_id > (JC_RX_ID_BASE + JC_MOTOR_ID_MAX)) {
        return;
    }

    uint8_t motor_id = (uint8_t)(can_id - JC_RX_ID_BASE);
    if (motor_id != s_last_motor_id) {
        return;
    }

    s_last_reply.cmd = rx_data[0];
    s_last_reply.reg = (uint16_t)(((uint16_t)rx_data[1] << 8) | rx_data[2]);

    if (s_last_reply.cmd == 0x2AU) {
        uint32_t raw_pos = ((uint32_t)rx_data[1] << 16)
                         | ((uint32_t)rx_data[2] << 8)
                         | (uint32_t)rx_data[3];
        s_last_reply.position = jc_sign_extend_24(raw_pos);
        s_last_reply.speed = (int16_t)(((uint16_t)rx_data[4] << 8) | rx_data[5]);
        s_last_reply.current = (int16_t)(((uint16_t)rx_data[6] << 8) | rx_data[7]);
    } else if (s_last_reply.cmd == JC_CMD_READ_16BIT) {
        s_last_reply.current = (int16_t)(((uint16_t)rx_data[4] << 8) | rx_data[5]);
    } else if (s_last_reply.cmd == JC_CMD_READ_32BIT) {
        s_last_reply.position = (int32_t)(((uint32_t)rx_data[4] << 24)
                                        | ((uint32_t)rx_data[5] << 16)
                                        | ((uint32_t)rx_data[6] << 8)
                                        | (uint32_t)rx_data[7]);
    }

    s_stats.last_rx_id = can_id;
    s_stats.rx_count++;
    __asm volatile ("dsb" ::: "memory");  /* Ensure s_last_reply is visible before setting flag */
    s_rx_flag = 1U;
    (void)knx_health_report(KNX_HEALTH_SOURCE_JC,
                            KNX_HEALTH_STATE_OK,
                            KNX_OK,
                            motor_id,
                            s_stats.error_count + s_stats.timeout_count);
}

knx_status_t JC_Send(uint8_t motor_id, uint16_t reg, uint8_t cmd, uint64_t data)
{
    if (s_can == NULL) {
        s_stats.last_status = KNX_NOT_READY;
        return KNX_NOT_READY;
    }
    if (!JC_IsValidMotorID(motor_id)) {
        s_stats.error_count++;
        s_stats.last_status = KNX_INVALID_ARG;
        return KNX_INVALID_ARG;
    }

    uint8_t tx_data[JC_CAN_DLC] = {0};

    if (cmd == JC_CMD_PV || cmd == JC_CMD_PVT) {
        tx_data[0] = cmd;
        tx_data[1] = (uint8_t)((data >> 24) & 0xFFU);
        tx_data[2] = (uint8_t)((data >> 16) & 0xFFU);
        tx_data[3] = (uint8_t)((data >> 8) & 0xFFU);
        tx_data[4] = (uint8_t)(data & 0xFFU);
        tx_data[5] = (uint8_t)((data >> 40) & 0xFFU);
        tx_data[6] = (uint8_t)((data >> 32) & 0xFFU);
        if (cmd == JC_CMD_PVT) {
            tx_data[7] = (uint8_t)((data >> 48) & 0xFFU);
        }
    } else {
        tx_data[0] = cmd;
        tx_data[1] = (uint8_t)(reg >> 8);
        tx_data[2] = (uint8_t)(reg & 0xFFU);
        tx_data[3] = 0U;

        if (cmd == JC_CMD_WRITE_16BIT) {
            tx_data[4] = (uint8_t)((data >> 8) & 0xFFU);
            tx_data[5] = (uint8_t)(data & 0xFFU);
        } else if (cmd == JC_CMD_WRITE_32BIT) {
            tx_data[4] = (uint8_t)((data >> 24) & 0xFFU);
            tx_data[5] = (uint8_t)((data >> 16) & 0xFFU);
            tx_data[6] = (uint8_t)((data >> 8) & 0xFFU);
            tx_data[7] = (uint8_t)(data & 0xFFU);
        }
    }

    s_last_motor_id = motor_id;
    s_last_reg = reg;
    s_rx_flag = 0U;

    uint16_t tx_id = (uint16_t)(JC_TX_ID_BASE + motor_id);
    knx_status_t status = knx_can_transmit_std(s_can, tx_id, tx_data, JC_CAN_DLC, s_timeout_ms);
    s_stats.last_tx_id = tx_id;
    s_stats.last_status = status;
    if (status == KNX_OK) {
        s_stats.tx_count++;
    } else {
        s_stats.error_count++;
    }
    (void)knx_health_report(KNX_HEALTH_SOURCE_JC,
                            (status == KNX_OK) ? KNX_HEALTH_STATE_OK : KNX_HEALTH_STATE_WARN,
                            status,
                            motor_id,
                            s_stats.error_count + s_stats.timeout_count);
    return status;
}

knx_status_t JC_Receive(uint8_t motor_id, JC_Reply *reply, uint32_t timeout_ms)
{
    if (reply == NULL || !JC_IsValidMotorID(motor_id)) {
        s_stats.last_status = KNX_INVALID_ARG;
        return KNX_INVALID_ARG;
    }

    uint32_t start = knx_millis();
    while (s_rx_flag == 0U && (knx_millis() - start) < timeout_ms) {
        taskYIELD();
    }

    if (s_rx_flag != 0U) {
        taskENTER_CRITICAL();
        *reply = s_last_reply;
        taskEXIT_CRITICAL();
        s_stats.last_status = KNX_OK;
        return KNX_OK;
    }

    s_stats.timeout_count++;
    s_stats.last_status = KNX_TIMEOUT;
    (void)knx_health_report(KNX_HEALTH_SOURCE_JC,
                            KNX_HEALTH_STATE_WARN,
                            KNX_TIMEOUT,
                            motor_id,
                            s_stats.error_count + s_stats.timeout_count);
    return KNX_TIMEOUT;
}

float JC_ReadVoltage(uint8_t motor_id)
{
    if (JC_Send(motor_id, JC_REG_VOLTAGE, JC_CMD_READ_16BIT, 0U) != KNX_OK) {
        return -1.0f;
    }

    JC_Reply reply;
    if (JC_Receive(motor_id, &reply, s_timeout_ms) == KNX_OK && reply.cmd == JC_CMD_READ_16BIT) {
        return (float)(uint16_t)reply.current * 0.1f;
    }
    return -1.0f;
}

float JC_ReadBusCurrent(uint8_t motor_id)
{
    if (JC_Send(motor_id, JC_REG_BUS_CURRENT, JC_CMD_READ_16BIT, 0U) != KNX_OK) {
        return -1.0f;
    }

    JC_Reply reply;
    if (JC_Receive(motor_id, &reply, s_timeout_ms) == KNX_OK && reply.cmd == JC_CMD_READ_16BIT) {
        return (float)(uint16_t)reply.current * 0.01f;
    }
    return -1.0f;
}

float JC_ReadSpeed(uint8_t motor_id)
{
    if (JC_Send(motor_id, JC_REG_SPEED, JC_CMD_READ_32BIT, 0U) != KNX_OK) {
        return -1.0f;
    }

    JC_Reply reply;
    if (JC_Receive(motor_id, &reply, s_timeout_ms) == KNX_OK && reply.cmd == JC_CMD_READ_32BIT) {
        return (float)reply.position * 0.01f;
    }
    return -1.0f;
}

float JC_ReadPosition(uint8_t motor_id)
{
    if (JC_Send(motor_id, JC_REG_POSITION, JC_CMD_READ_32BIT, 0U) != KNX_OK) {
        return -1.0f;
    }

    JC_Reply reply;
    if (JC_Receive(motor_id, &reply, s_timeout_ms) == KNX_OK && reply.cmd == JC_CMD_READ_32BIT) {
        return (float)reply.position * 0.01f;
    }
    return -1.0f;
}

float JC_ReadDrvTemp(uint8_t motor_id)
{
    if (JC_Send(motor_id, JC_REG_DRV_TEMP, JC_CMD_READ_16BIT, 0U) != KNX_OK) {
        return -1.0f;
    }

    JC_Reply reply;
    if (JC_Receive(motor_id, &reply, s_timeout_ms) == KNX_OK && reply.cmd == JC_CMD_READ_16BIT) {
        return (float)(uint16_t)reply.current * 0.1f;
    }
    return -1.0f;
}

float JC_ReadMotTemp(uint8_t motor_id)
{
    if (JC_Send(motor_id, JC_REG_MOT_TEMP, JC_CMD_READ_16BIT, 0U) != KNX_OK) {
        return -1.0f;
    }

    JC_Reply reply;
    if (JC_Receive(motor_id, &reply, s_timeout_ms) == KNX_OK && reply.cmd == JC_CMD_READ_16BIT) {
        return (float)(uint16_t)reply.current * 0.1f;
    }
    return -1.0f;
}

uint32_t JC_ReadError(uint8_t motor_id)
{
    if (JC_Send(motor_id, JC_REG_ERROR, JC_CMD_READ_32BIT, 0U) != KNX_OK) {
        return 0xFFFFFFFFU;
    }

    JC_Reply reply;
    if (JC_Receive(motor_id, &reply, s_timeout_ms) == KNX_OK && reply.cmd == JC_CMD_READ_32BIT) {
        return (uint32_t)reply.position;
    }
    return 0xFFFFFFFFU;
}

knx_status_t JC_SetTorque(uint8_t motor_id, float torque_nm)
{
    int16_t value = (int16_t)(torque_nm * 100.0f);
    return JC_Send(motor_id, JC_REG_SET_TORQUE, JC_CMD_WRITE_16BIT, (uint16_t)value);
}

knx_status_t JC_SetSpeed(uint8_t motor_id, float speed_rpm)
{
    int32_t value = (int32_t)(speed_rpm * 100.0f);
    return JC_Send(motor_id, JC_REG_SET_SPEED, JC_CMD_WRITE_32BIT, (uint32_t)value);
}

knx_status_t JC_SetAbsPosition(uint8_t motor_id, float pos_deg)
{
    int32_t value = (int32_t)(pos_deg * 100.0f);
    return JC_Send(motor_id, JC_REG_SET_ABS_POS, JC_CMD_WRITE_32BIT, (uint32_t)value);
}

knx_status_t JC_SetRelPosition(uint8_t motor_id, float rel_deg)
{
    int32_t value = (int32_t)(rel_deg * 100.0f);
    return JC_Send(motor_id, JC_REG_SET_REL_POS, JC_CMD_WRITE_32BIT, (uint32_t)value);
}

knx_status_t JC_SetLowSpeed(uint8_t motor_id, float low_rpm)
{
    int16_t value = (int16_t)(low_rpm * 100.0f);
    return JC_Send(motor_id, JC_REG_SET_LOW_SPEED, JC_CMD_WRITE_16BIT, (uint16_t)value);
}

knx_status_t JC_SendPV(uint8_t motor_id, float pos_deg, float vel_rpm)
{
    int32_t pos = (int32_t)(pos_deg * 100.0f);
    int16_t vel = (int16_t)(vel_rpm * 100.0f);
    uint64_t data = ((uint64_t)(uint16_t)vel << 32) | (uint32_t)pos;
    return JC_Send(motor_id, 0U, JC_CMD_PV, data);
}

knx_status_t JC_SendPVT(uint8_t motor_id, float pos_deg, float vel_rpm, uint8_t torque_percent)
{
    if (torque_percent > 100U) {
        torque_percent = 100U;
    }

    int32_t pos = (int32_t)(pos_deg * 100.0f);
    int16_t vel = (int16_t)(vel_rpm * 100.0f);
    uint64_t data = ((uint64_t)torque_percent << 48)
                  | ((uint64_t)(uint16_t)vel << 32)
                  | (uint32_t)pos;
    return JC_Send(motor_id, 0U, JC_CMD_PVT, data);
}

knx_status_t JC_SwitchMode(uint8_t motor_id, JC_ControlMode mode)
{
    if (mode > JC_MODE_LOW_SPEED_HIGH_TORQUE) {
        s_stats.error_count++;
        s_stats.last_status = KNX_INVALID_ARG;
        return KNX_INVALID_ARG;
    }
    return JC_Send(motor_id, JC_REG_CONTROL_MODE, JC_CMD_WRITE_16BIT, (uint16_t)mode);
}

knx_status_t JC_Idle(uint8_t motor_id)
{
    return JC_Send(motor_id, JC_REG_IDLE_MODE, JC_CMD_WRITE_16BIT, 1U);
}

knx_status_t JC_Calibrate(uint8_t motor_id)
{
    return JC_Send(motor_id, JC_REG_CALIBRATE, JC_CMD_WRITE_16BIT, 1U);
}

uint16_t JC_ReadCalResult(uint8_t motor_id)
{
    if (JC_Send(motor_id, JC_REG_CAL_RESULT, JC_CMD_READ_16BIT, 0U) != KNX_OK) {
        return 0U;
    }

    JC_Reply reply;
    if (JC_Receive(motor_id, &reply, s_timeout_ms) == KNX_OK && reply.cmd == JC_CMD_READ_16BIT) {
        return (uint16_t)reply.current;
    }
    return 0U;
}

knx_status_t JC_EnterServo(uint8_t motor_id)
{
    return JC_Send(motor_id, JC_REG_ENTER_SERVO, JC_CMD_WRITE_16BIT, 1U);
}

knx_status_t JC_SaveParams(uint8_t motor_id)
{
    return JC_Send(motor_id, JC_REG_SAVE, JC_CMD_WRITE_16BIT, 1U);
}

knx_status_t JC_EraseParams(uint8_t motor_id)
{
    return JC_Send(motor_id, JC_REG_ERASE, JC_CMD_WRITE_16BIT, 1U);
}

knx_status_t JC_SetOrigin(uint8_t motor_id)
{
    return JC_Send(motor_id, JC_REG_SET_ORIGIN, JC_CMD_WRITE_16BIT, 1U);
}

knx_status_t JC_Reboot(uint8_t motor_id)
{
    return JC_Send(motor_id, JC_REG_REBOOT, JC_CMD_WRITE_16BIT, 1U);
}

void JC_GetLastReply(JC_Reply *reply)
{
    if (reply != NULL) {
        taskENTER_CRITICAL();
        *reply = s_last_reply;
        taskEXIT_CRITICAL();
    }
}

void JC_GetStats(JC_Stats *stats)
{
    if (stats != NULL) {
        *stats = s_stats;
    }
}

void JC_ParseError(uint32_t error_code, char *buf, uint16_t buf_size)
{
    if (buf == NULL || buf_size == 0U) {
        return;
    }

    if (error_code == JC_ERR_NONE) {
        (void)snprintf(buf, buf_size, "No error");
        return;
    }

    buf[0] = '\0';
    uint16_t len = 0U;

#define JC_APPEND_ERR(bit, text) \
    do { \
        if ((error_code & (bit)) != 0U && len < buf_size) { \
            int written = snprintf(buf + len, (size_t)(buf_size - len), "%s;", (text)); \
            if (written > 0) { \
                len = (uint16_t)(len + (uint16_t)written); \
                if (len >= buf_size) { \
                    len = (uint16_t)(buf_size - 1U); \
                } \
            } \
        } \
    } while (0)

    JC_APPEND_ERR(JC_ERR_OVER_VOLTAGE, "OverVolt");
    JC_APPEND_ERR(JC_ERR_UNDER_VOLTAGE, "UnderVolt");
    JC_APPEND_ERR(JC_ERR_OVER_CURRENT, "OverCurrent");
    JC_APPEND_ERR(JC_ERR_WAVE_OVERFLOW, "WaveOvfl");
    JC_APPEND_ERR(JC_ERR_SENSE_OVER_VOLTAGE, "SenseOvr");
    JC_APPEND_ERR(JC_ERR_SENSE_UNDER_VOLTAGE, "SenseUnd");
    JC_APPEND_ERR(JC_ERR_ENCODER_SPI_FAIL, "EncSPI");
    JC_APPEND_ERR(JC_ERR_ENCODER_TYPE_ERROR, "EncType");
    JC_APPEND_ERR(JC_ERR_HALL_NOT_FOUND, "HallNotFound");
    JC_APPEND_ERR(JC_ERR_ENCODER_NOT_FOUND, "EncNotFound");
    JC_APPEND_ERR(JC_ERR_CPR_ERROR, "CPRErr");
    JC_APPEND_ERR(JC_ERR_STATE_ERROR, "StateErr");
    JC_APPEND_ERR(JC_ERR_HALL_SIGNAL_ERROR, "HallSig");
    JC_APPEND_ERR(JC_ERR_SECOND_ENCODER_ERR, "Enc2");
    JC_APPEND_ERR(JC_ERR_JC2804_DRV_ERROR, "JC2804Drv");
    JC_APPEND_ERR(JC_ERR_MOS_OVER_TEMP, "MOSTemp");
    JC_APPEND_ERR(JC_ERR_MOT_OVER_TEMP, "MotTemp");
    JC_APPEND_ERR(JC_ERR_BUS_UNDER_VOLTAGE, "BusUnd");
    JC_APPEND_ERR(JC_ERR_BUS_OVER_VOLTAGE, "BusOvr");
    JC_APPEND_ERR(JC_ERR_CURRENT_OVERFLOW, "CurOvfl");

#undef JC_APPEND_ERR

    if (len > 0U && buf[len - 1U] == ';') {
        buf[len - 1U] = '\0';
    }
}

uint8_t JC_IsValidMotorID(uint8_t motor_id)
{
    return (motor_id >= JC_MOTOR_ID_MIN && motor_id <= JC_MOTOR_ID_MAX) ? 1U : 0U;
}
