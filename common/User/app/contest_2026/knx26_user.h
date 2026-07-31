#ifndef KNX26_USER_H
#define KNX26_USER_H

#include "knx_comm.h"
#include "knx_intersection.h"
#include "octolinker.h"

struct knx26_context;

typedef enum {
    KNX26_LINE_IDLE = 0,
    KNX26_LINE_CAL_BLACK_DELAY,
    KNX26_LINE_CAL_BLACK_SAMPLE,
    KNX26_LINE_CAL_WHITE_DELAY,
    KNX26_LINE_CAL_WHITE_SAMPLE,
    KNX26_LINE_READY,
    KNX26_LINE_RUNNING,
    KNX26_LINE_STOPPED,
    KNX26_LINE_LOST,
    KNX26_LINE_CAL_FAULT,
    KNX26_LINE_START_CLEAR,
    KNX26_LINE_FINISH_APPROACH,
    KNX26_LINE_COMPLETE,
} knx26_line_state_t;

/* Hardware polarity and bench-safe PWM ceiling. */
extern float knx26_test_left_cmd_polarity;
extern float knx26_test_right_cmd_polarity;
extern float knx26_test_left_feedback_polarity;
extern float knx26_test_right_feedback_polarity;
extern float knx26_test_max_duty;

/* Line-follow parameters can be tuned from OctoLink/GDB. */
extern float knx26_line_speed_mps;
extern float knx26_line_kp;
extern float knx26_line_ki;
extern float knx26_line_kd;
extern float knx26_line_max_angular_radps;
extern float knx26_line_min_angular_radps;
extern float knx26_line_integral_limit;
extern float knx26_line_min_strength;
extern float knx26_line_recovery_angular_radps;

/* H题A线停车与小球目标，支持OctoLink/GDB在线调整。 */
extern float knexus_h_stop_after_mark_m;
extern float knexus_h_ball_target_cm;

/* PID 调参模式的运行期目标，可由 OctoLink/GDB 在线修改。 */
extern float knexus_tune_linear_speed_mps;
extern float knexus_tune_angular_speed_radps;

void knx26_user_init(void);
void knx26_user_update(const struct knx26_context *context);
void knx26_user_on_intersection(const knx_intersection_result_t *result);
void knx26_user_on_peer_message(const knx_comm_message_t *message);
void knx26_user_debug_octo(Octolinker_Instance_t *octo, uint16_t base_id);
void knx26_user_debug_control_octo(Octolinker_Instance_t *octo,
                                   uint16_t base_id);

#if (defined(KNEXUS_MODE_LINE_FOLLOW) || \
     defined(KNEXUS_MODE_LINE_FOLLOW_BALL_CENTER)) && \
    KNEXUS_H_TASK_ENABLE
void knx26_h_ball_control_update(void);
#endif

#endif
