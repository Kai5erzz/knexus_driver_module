#include "dji_motor.h"
#include "knx_time.h"
#include <stddef.h>
#include <string.h>

#define DJI_SPEED_FILTER_ALPHA  0.85f
#define DJI_CURRENT_FILTER_ALPHA 0.90f

static dji_motor_object_t s_motors[DJI_MOTOR_COUNT_MAX];
static uint8_t s_motor_count;

void dji_motor_init(void)
{
    memset(s_motors, 0, sizeof(s_motors));
    s_motor_count = 0U;
}

static void decode_feedback(dji_motor_object_t *motor,
                            const uint8_t data[8])
{
    dji_motor_measure_t *m = &motor->measure;
    uint16_t ecd = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    int16_t raw_speed = (int16_t)(((uint16_t)data[2] << 8U) | data[3]);
    int16_t raw_current = (int16_t)(((uint16_t)data[4] << 8U) | data[5]);

    if (m->feedback_count == 0U) {
        m->last_ecd = ecd;
        m->speed_rpm = (float)raw_speed;
        m->current = (float)raw_current;
    } else {
        m->last_ecd = m->ecd;
        int32_t delta = (int32_t)ecd - (int32_t)m->last_ecd;
        if (delta > 4096) m->total_round--;
        else if (delta < -4096) m->total_round++;
        m->speed_rpm += DJI_SPEED_FILTER_ALPHA *
                        ((float)raw_speed - m->speed_rpm);
        m->current += DJI_CURRENT_FILTER_ALPHA *
                      ((float)raw_current - m->current);
    }

    m->ecd = ecd;
    m->raw_speed_rpm = raw_speed;
    m->raw_current = raw_current;
    m->angle_single_round_deg = DJI_ENCODER_DEG_PER_COUNT * (float)ecd;
    m->speed_deg_per_s = m->speed_rpm * DJI_RPM_TO_DEG_PER_SEC;
    m->total_angle_deg = (float)m->total_round * 360.0f +
                         m->angle_single_round_deg;
    m->temperature_c = data[6];
    m->feedback_count++;
    m->last_feedback_ms = knx_millis();
}

void dji_motor_rx_handler(uint32_t std_id, const uint8_t *data,
                          uint8_t len, void *user)
{
    (void)user;
    if (data == NULL || len < 7U) return;
    for (uint8_t i = 0U; i < s_motor_count; ++i) {
        if (s_motors[i].rx_id == std_id) {
            decode_feedback(&s_motors[i], data);
            return;
        }
    }
}

dji_motor_object_t *dji_motor_register(const dji_motor_config_t *config,
                                       dji_motor_control_fn control)
{
    if (config == NULL || control == NULL ||
        config->can_bus_index >= KNX_CAN_BUS_COUNT ||
        config->rx_id < 0x201U || config->rx_id > 0x204U ||
        s_motor_count >= DJI_MOTOR_COUNT_MAX) {
        return NULL;
    }

    dji_motor_object_t *motor = &s_motors[s_motor_count++];
    memset(motor, 0, sizeof(*motor));
    motor->bus = (knx_can_bus_id_t)config->can_bus_index;
    motor->tx_id = config->tx_id;
    motor->rx_id = config->rx_id;
    motor->message_slot = (uint8_t)(config->rx_id - 0x201U);
    motor->motor_type = config->motor_type;
    motor->control = control;
    motor->state = DJI_MOTOR_STOP;
    return motor;
}

knx_status_t dji_motor_control(void)
{
    if (s_motor_count == 0U) return KNX_NOT_READY;

    uint8_t frame[8] = {0};
    for (uint8_t i = 0U; i < s_motor_count; ++i) {
        dji_motor_object_t *motor = &s_motors[i];
        int16_t command = 0;
        if (motor->state == DJI_MOTOR_ENABLED) {
            command = motor->control(&motor->measure);
        }
        uint8_t offset = (uint8_t)(motor->message_slot * 2U);
        frame[offset] = (uint8_t)(((uint16_t)command >> 8U) & 0xFFU);
        frame[offset + 1U] = (uint8_t)((uint16_t)command & 0xFFU);
    }

    /*
     * This runs from the 1 ms high-priority control task.  Never wait for a
     * full CAN TX FIFO here: an unplugged C620 (no ACK) must not starve the
     * LED, debug and application tasks.
     */
    return knx_can_bus_try_send(s_motors[0].bus, s_motors[0].tx_id,
                                frame, sizeof(frame));
}

void dji_motor_relax(dji_motor_object_t *motor)
{
    if (motor != NULL) motor->state = DJI_MOTOR_STOP;
}

void dji_motor_enable(dji_motor_object_t *motor)
{
    if (motor != NULL) motor->state = DJI_MOTOR_ENABLED;
}
