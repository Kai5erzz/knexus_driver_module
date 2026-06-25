#ifndef JC_DRIVER_H
#define JC_DRIVER_H

#include "knx_can.h"
#include "knx_types.h"
#include <stdint.h>

#define JC_DRIVER_VERSION "3.0_knexus"

#define JC_FOC_MOTOR1_ID 1U
#define JC_FOC_MOTOR2_ID 2U

#define JC_CMD_READ_16BIT  0x4BU
#define JC_CMD_READ_32BIT  0x43U
#define JC_CMD_WRITE_16BIT 0x2BU
#define JC_CMD_WRITE_32BIT 0x23U
#define JC_CMD_PV          0x24U
#define JC_CMD_PVT         0x25U

#define JC_DEFAULT_MOTOR_ID 1U
#define JC_TX_ID_BASE       0x600U
#define JC_RX_ID_BASE       0x580U
#define JC_CAN_DLC          8U
#define JC_MOTOR_ID_MIN     1U
#define JC_MOTOR_ID_MAX     127U

#define JC_REG_HW_VERSION    0x0002U
#define JC_REG_FW_VERSION    0x0003U
#define JC_REG_VOLTAGE       0x0004U
#define JC_REG_BUS_CURRENT   0x0005U
#define JC_REG_SPEED         0x0006U
#define JC_REG_POSITION      0x0008U
#define JC_REG_DRV_TEMP      0x000AU
#define JC_REG_MOT_TEMP      0x000BU
#define JC_REG_ERROR         0x000CU

#define JC_REG_SET_TORQUE    0x0020U
#define JC_REG_SET_SPEED     0x0021U
#define JC_REG_SET_ABS_POS   0x0023U
#define JC_REG_SET_REL_POS   0x0025U
#define JC_REG_SET_LOW_SPEED 0x0027U

#define JC_REG_CONTROL_MODE  0x0060U

#define JC_REG_IDLE_MODE     0x00A0U
#define JC_REG_CALIBRATE     0x00A1U
#define JC_REG_ENTER_SERVO   0x00A2U
#define JC_REG_ERASE         0x00A3U
#define JC_REG_SAVE          0x00A4U
#define JC_REG_REBOOT        0x00A5U
#define JC_REG_SET_ORIGIN    0x00A6U
#define JC_REG_CAL_RESULT    0x00C2U

typedef enum {
    JC_MODE_TORQUE = 0,
    JC_MODE_SPEED = 1,
    JC_MODE_POS_TRAPEZOID = 2,
    JC_MODE_POS_FILTER = 3,
    JC_MODE_POS_DIRECT = 4,
    JC_MODE_LOW_SPEED_HIGH_TORQUE = 5,
} JC_ControlMode;

typedef enum {
    JC_ERR_NONE                = 0x000000U,
    JC_ERR_OVER_VOLTAGE        = 0x000001U,
    JC_ERR_UNDER_VOLTAGE       = 0x000002U,
    JC_ERR_OVER_CURRENT        = 0x000004U,
    JC_ERR_WAVE_OVERFLOW       = 0x000008U,
    JC_ERR_SENSE_OVER_VOLTAGE  = 0x000010U,
    JC_ERR_SENSE_UNDER_VOLTAGE = 0x000020U,
    JC_ERR_ENCODER_SPI_FAIL    = 0x000040U,
    JC_ERR_ENCODER_TYPE_ERROR  = 0x000080U,
    JC_ERR_HALL_NOT_FOUND      = 0x000100U,
    JC_ERR_ENCODER_NOT_FOUND   = 0x000200U,
    JC_ERR_CPR_ERROR           = 0x000400U,
    JC_ERR_STATE_ERROR         = 0x000800U,
    JC_ERR_HALL_SIGNAL_ERROR   = 0x008000U,
    JC_ERR_SECOND_ENCODER_ERR  = 0x020000U,
    JC_ERR_JC2804_DRV_ERROR    = 0x080000U,
    JC_ERR_MOS_OVER_TEMP       = 0x100000U,
    JC_ERR_MOT_OVER_TEMP       = 0x200000U,
    JC_ERR_BUS_UNDER_VOLTAGE   = 0x400000U,
    JC_ERR_BUS_OVER_VOLTAGE    = 0x800000U,
    JC_ERR_CURRENT_OVERFLOW    = 0x1000000U,
} JC_ErrorCode;

typedef struct {
    uint8_t cmd;
    uint16_t reg;
    int32_t position;
    int16_t speed;
    int16_t current;
    uint32_t error;
} JC_Reply;

typedef struct {
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t timeout_count;
    uint32_t error_count;
    uint32_t last_tx_id;
    uint32_t last_rx_id;
    knx_status_t last_status;
} JC_Stats;

void JC_AttachCAN(knx_can_t *can);
knx_status_t JC_Init(void);
knx_status_t JC_InitExternalRx(void);
void JC_SetTimeoutMs(uint32_t timeout_ms);
uint32_t JC_GetTimeoutMs(void);

knx_status_t JC_Send(uint8_t motor_id, uint16_t reg, uint8_t cmd, uint64_t data);
knx_status_t JC_Receive(uint8_t motor_id, JC_Reply *reply, uint32_t timeout_ms);
void JC_RxHandler(uint32_t can_id, const uint8_t *rx_data, uint8_t len);

float JC_ReadVoltage(uint8_t motor_id);
float JC_ReadBusCurrent(uint8_t motor_id);
float JC_ReadSpeed(uint8_t motor_id);
float JC_ReadPosition(uint8_t motor_id);
float JC_ReadDrvTemp(uint8_t motor_id);
float JC_ReadMotTemp(uint8_t motor_id);
uint32_t JC_ReadError(uint8_t motor_id);

knx_status_t JC_SetTorque(uint8_t motor_id, float torque_nm);
knx_status_t JC_SetSpeed(uint8_t motor_id, float speed_rpm);
knx_status_t JC_SetAbsPosition(uint8_t motor_id, float pos_deg);
knx_status_t JC_SetRelPosition(uint8_t motor_id, float rel_deg);
knx_status_t JC_SetLowSpeed(uint8_t motor_id, float low_rpm);

knx_status_t JC_SendPV(uint8_t motor_id, float pos_deg, float vel_rpm);
knx_status_t JC_SendPVT(uint8_t motor_id, float pos_deg, float vel_rpm, uint8_t torque_percent);

knx_status_t JC_SwitchMode(uint8_t motor_id, JC_ControlMode mode);
knx_status_t JC_Idle(uint8_t motor_id);
knx_status_t JC_Calibrate(uint8_t motor_id);
uint16_t JC_ReadCalResult(uint8_t motor_id);
knx_status_t JC_EnterServo(uint8_t motor_id);
knx_status_t JC_SaveParams(uint8_t motor_id);
knx_status_t JC_EraseParams(uint8_t motor_id);
knx_status_t JC_SetOrigin(uint8_t motor_id);
knx_status_t JC_Reboot(uint8_t motor_id);

void JC_GetLastReply(JC_Reply *reply);
void JC_GetStats(JC_Stats *stats);
void JC_ParseError(uint32_t error_code, char *buf, uint16_t buf_size);
uint8_t JC_IsValidMotorID(uint8_t motor_id);

#endif /* JC_DRIVER_H */
