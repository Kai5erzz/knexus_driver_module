#ifndef KNX_DJI_MOTOR_DEF_H
#define KNX_DJI_MOTOR_DEF_H

#include <stdint.h>

#define DJI_ENCODER_COUNTS_PER_ROUND 8192U
#define DJI_ENCODER_DEG_PER_COUNT    (360.0f / 8192.0f)
#define DJI_RPM_TO_DEG_PER_SEC       6.0f

typedef enum {
    DJI_MOTOR_STOP = 0,
    DJI_MOTOR_ENABLED = 1,
} dji_motor_working_state_t;

typedef enum {
    DJI_MOTOR_TYPE_NONE = 0,
    DJI_GM6020,
    DJI_M3508,
    DJI_M2006,
} dji_motor_type_t;

typedef struct {
    dji_motor_type_t motor_type;
    uint8_t can_bus_index;
    uint16_t tx_id;
    uint16_t rx_id;
} dji_motor_config_t;

#endif
