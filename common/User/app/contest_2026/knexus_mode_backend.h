#ifndef KNEXUS_MODE_BACKEND_H
#define KNEXUS_MODE_BACKEND_H

#include "knx26_user.h"

/* 每个模式都实现同一组生命周期函数，由 knexus_mode.c 统一转发。 */
#define KNEXUS_DECLARE_MODE(name) \
    void name##_init(void); \
    void name##_update(const struct knx26_context *context); \
    void name##_on_intersection(const knx_intersection_result_t *result); \
    void name##_on_peer_message(const knx_comm_message_t *message); \
    void name##_debug_octo(Octolinker_Instance_t *octo, uint16_t base_id); \
    void name##_debug_control_octo(Octolinker_Instance_t *octo, uint16_t base_id)

KNEXUS_DECLARE_MODE(knexus_mode_line_follow);
KNEXUS_DECLARE_MODE(knexus_mode_intersection_sample);
KNEXUS_DECLARE_MODE(knexus_mode_pid_tune);
KNEXUS_DECLARE_MODE(knexus_mode_user);
KNEXUS_DECLARE_MODE(knexus_mode_board_test);
KNEXUS_DECLARE_MODE(knexus_mode_screw_test);
KNEXUS_DECLARE_MODE(knexus_mode_static_rod_angle);
KNEXUS_DECLARE_MODE(knexus_mode_line_follow_ball_center);
KNEXUS_DECLARE_MODE(knexus_mode_jc4310_link_center);
KNEXUS_DECLARE_MODE(knexus_mode_screw_offset_center);
KNEXUS_DECLARE_MODE(knexus_mode_six_menu);

void knexus_jc4310_external_target_set(float target_roll_deg, bool enabled);
void knexus_jc4310_external_target_set_ex(
    float target_roll_deg, bool enabled, bool add_motion_compensation);
void knexus_jc4310_motion_comp_request(bool enabled);
void knexus_jc4310_force_zero(void);
uint8_t knexus_jc4310_is_ready(void);
uint8_t knexus_jc4310_motion_comp_is_ready(void);
uint8_t knexus_jc4310_is_settled(void);
void knexus_jc4310_debug_compact_control_octo(
    Octolinker_Instance_t *octo);

#undef KNEXUS_DECLARE_MODE

#endif
