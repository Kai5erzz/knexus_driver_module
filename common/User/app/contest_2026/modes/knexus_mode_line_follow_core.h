#ifndef KNEXUS_MODE_LINE_FOLLOW_CORE_H
#define KNEXUS_MODE_LINE_FOLLOW_CORE_H

#include "knx26_user.h"

void knexus_line_follow_core_init(void);
void knexus_line_follow_core_update(const struct knx26_context *context);
void knexus_line_follow_core_on_intersection(
    const knx_intersection_result_t *result);
void knexus_line_follow_core_on_peer_message(
    const knx_comm_message_t *message);
void knexus_line_follow_core_debug_octo(Octolinker_Instance_t *octo,
                                        uint16_t base_id);
void knexus_line_follow_core_debug_control_octo(
    Octolinker_Instance_t *octo, uint16_t base_id);
knx26_line_state_t knexus_line_follow_core_get_state(void);
float knexus_line_follow_core_get_accel_command_mps2(void);
float knexus_line_follow_core_get_distance_m(void);
uint32_t knexus_line_follow_core_get_elapsed_ms(void);
void knexus_line_follow_core_force_stop(void);
void knexus_line_follow_core_set_longitudinal_limits(
    float accel_limit_mps2, float decel_limit_mps2,
    float jerk_limit_mps3);
void knexus_line_follow_core_set_constant_speed(bool enabled);
void knx26_h_ball_control_update(void);

#endif
