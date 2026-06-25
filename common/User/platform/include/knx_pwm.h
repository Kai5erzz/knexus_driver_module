#ifndef KNX_PWM_H
#define KNX_PWM_H

#include "knx_types.h"

typedef struct {
    void     *timer;
    uint32_t  channel;
    uint32_t  arr;
} knx_pwm_channel_t;

knx_status_t knx_pwm_start(knx_pwm_channel_t *ch);
knx_status_t knx_pwm_stop(knx_pwm_channel_t *ch);
knx_status_t knx_pwm_set_duty(knx_pwm_channel_t *ch, float duty);
knx_status_t knx_pwm_set_compare(knx_pwm_channel_t *ch, uint32_t compare);
knx_status_t knx_pwm_get_period(const knx_pwm_channel_t *ch, uint32_t *period);
knx_status_t knx_pwm_get_counter(const knx_pwm_channel_t *ch, uint32_t *counter);
knx_status_t knx_pwm_start_compare(knx_pwm_channel_t *ch);

#endif /* KNX_PWM_H */
