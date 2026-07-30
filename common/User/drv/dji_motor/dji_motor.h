#ifndef KNX_DJI_MOTOR_H
#define KNX_DJI_MOTOR_H

#include "knx_can_bus.h"
#include "motor_def.h"

#define DJI_MOTOR_COUNT_MAX 2U

typedef struct {
    float angle_single_round_deg;
    float speed_deg_per_s;
    float total_angle_deg;
    int32_t total_round;

    uint16_t ecd;
    uint16_t last_ecd;
    int16_t raw_speed_rpm;
    int16_t raw_current;
    float speed_rpm;
    float current;
    uint8_t temperature_c;
    uint32_t feedback_count;
    uint32_t last_feedback_ms;
} dji_motor_measure_t;

typedef int16_t (*dji_motor_control_fn)(const dji_motor_measure_t *measure);

typedef struct {
    knx_can_bus_id_t bus;
    dji_motor_measure_t measure;
    uint16_t tx_id;
    uint16_t rx_id;
    uint8_t message_slot;
    dji_motor_type_t motor_type;
    dji_motor_working_state_t state;
    dji_motor_control_fn control;
} dji_motor_object_t;

void dji_motor_init(void);
dji_motor_object_t *dji_motor_register(const dji_motor_config_t *config,
                                       dji_motor_control_fn control);
knx_status_t dji_motor_control(void);
void dji_motor_relax(dji_motor_object_t *motor);
void dji_motor_enable(dji_motor_object_t *motor);
void dji_motor_rx_handler(uint32_t std_id, const uint8_t *data,
                          uint8_t len, void *user);

#endif
