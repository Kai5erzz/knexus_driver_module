#include "knexus_mode_backend.h"
#include "knexus_config.h"

typedef struct {
    void (*init)(void);
    void (*update)(const struct knx26_context *context);
    void (*on_intersection)(const knx_intersection_result_t *result);
    void (*on_peer_message)(const knx_comm_message_t *message);
    void (*debug_octo)(Octolinker_Instance_t *octo, uint16_t base_id);
    void (*debug_control_octo)(Octolinker_Instance_t *octo, uint16_t base_id);
} knexus_mode_ops_t;

#if defined(KNEXUS_MODE_LINE_FOLLOW)
#define KNEXUS_ACTIVE_MODE_PREFIX knexus_mode_line_follow
#elif defined(KNEXUS_MODE_INTERSECTION_SAMPLE)
#define KNEXUS_ACTIVE_MODE_PREFIX knexus_mode_intersection_sample
#elif defined(KNEXUS_MODE_PID_TUNE)
#define KNEXUS_ACTIVE_MODE_PREFIX knexus_mode_pid_tune
#elif defined(KNEXUS_MODE_BOARD_TEST)
#define KNEXUS_ACTIVE_MODE_PREFIX knexus_mode_board_test
#elif defined(KNEXUS_MODE_SCREW_TEST)
#define KNEXUS_ACTIVE_MODE_PREFIX knexus_mode_screw_test
#elif defined(KNEXUS_MODE_STATIC_ROD_ANGLE)
#define KNEXUS_ACTIVE_MODE_PREFIX knexus_mode_static_rod_angle
#elif defined(KNEXUS_MODE_LINE_FOLLOW_BALL_CENTER)
#define KNEXUS_ACTIVE_MODE_PREFIX knexus_mode_line_follow_ball_center
#else
#define KNEXUS_ACTIVE_MODE_PREFIX knexus_mode_user
#endif

#define KNEXUS_MODE_FN_(prefix, suffix) prefix##suffix
#define KNEXUS_MODE_FN(prefix, suffix) KNEXUS_MODE_FN_(prefix, suffix)

static const knexus_mode_ops_t s_mode = {
    .init = KNEXUS_MODE_FN(KNEXUS_ACTIVE_MODE_PREFIX, _init),
    .update = KNEXUS_MODE_FN(KNEXUS_ACTIVE_MODE_PREFIX, _update),
    .on_intersection = KNEXUS_MODE_FN(KNEXUS_ACTIVE_MODE_PREFIX, _on_intersection),
    .on_peer_message = KNEXUS_MODE_FN(KNEXUS_ACTIVE_MODE_PREFIX, _on_peer_message),
    .debug_octo = KNEXUS_MODE_FN(KNEXUS_ACTIVE_MODE_PREFIX, _debug_octo),
    .debug_control_octo = KNEXUS_MODE_FN(KNEXUS_ACTIVE_MODE_PREFIX, _debug_control_octo),
};

void knx26_user_init(void)
{
    s_mode.init();
}

void knx26_user_update(const struct knx26_context *context)
{
    s_mode.update(context);
}

void knx26_user_on_intersection(const knx_intersection_result_t *result)
{
    s_mode.on_intersection(result);
}

void knx26_user_on_peer_message(const knx_comm_message_t *message)
{
    s_mode.on_peer_message(message);
}

void knx26_user_debug_octo(Octolinker_Instance_t *octo, uint16_t base_id)
{
    s_mode.debug_octo(octo, base_id);
}

void knx26_user_debug_control_octo(Octolinker_Instance_t *octo,
                                   uint16_t base_id)
{
    s_mode.debug_control_octo(octo, base_id);
}
