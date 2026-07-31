#include "knexus_h_hooks.h"
#include "FreeRTOS.h"
#include "task.h"

static knexus_h_ball_request_t s_request;
static knexus_h_ball_status_t s_status;

__attribute__((weak)) void knexus_h_ball_user_init(void)
{
}

__attribute__((weak)) void knexus_h_ball_user_update(
    const knexus_h_ball_request_t *request,
    const struct knx26_context *context,
    float dt_s,
    knexus_h_ball_status_t *status)
{
    (void)dt_s;
    (void)context;
    if (request == NULL || status == NULL) return;
    status->implemented = false;
    status->ready = false;
    status->fault = false;
    status->target_position_cm = request->target_position_cm;
    status->measured_position_cm = 0.0f;
    status->control_output = 0.0f;
}

__attribute__((weak)) void knexus_h_display_time_ms(uint32_t elapsed_ms,
                                                     bool running,
                                                     bool completed)
{
    (void)elapsed_ms;
    (void)running;
    (void)completed;
}

__attribute__((weak)) void knexus_h_display_invalidate(void)
{
}

__attribute__((weak)) void knexus_h_display_set_limit_ms(uint32_t limit_ms)
{
    (void)limit_ms;
}

void knexus_h_ball_init(void)
{
    s_request = (knexus_h_ball_request_t){
        .mode = KNEXUS_H_BALL_DISABLED,
        .target_position_cm = 0.0f,
    };
    s_status = (knexus_h_ball_status_t){0};
    knexus_h_ball_user_init();
}

void knexus_h_ball_request_hold(float target_position_cm)
{
    taskENTER_CRITICAL();
    s_request.mode = KNEXUS_H_BALL_HOLD_POSITION;
    s_request.target_position_cm = target_position_cm;
    taskEXIT_CRITICAL();
}

void knexus_h_ball_request_static_sequence(void)
{
    taskENTER_CRITICAL();
    s_request.mode = KNEXUS_H_BALL_STATIC_SEQUENCE;
    taskEXIT_CRITICAL();
}

void knexus_h_ball_disable(void)
{
    taskENTER_CRITICAL();
    s_request.mode = KNEXUS_H_BALL_DISABLED;
    taskEXIT_CRITICAL();
}

void knexus_h_ball_update(const struct knx26_context *context, float dt_s)
{
    knexus_h_ball_request_t request;
    knexus_h_ball_status_t status;
    taskENTER_CRITICAL();
    request = s_request;
    status = s_status;
    taskEXIT_CRITICAL();

    status.target_position_cm = request.target_position_cm;
    knexus_h_ball_user_update(&request, context, dt_s, &status);

    taskENTER_CRITICAL();
    s_status = status;
    taskEXIT_CRITICAL();
}

void knexus_h_ball_snapshot(knexus_h_ball_status_t *out)
{
    if (out == NULL) return;
    taskENTER_CRITICAL();
    *out = s_status;
    taskEXIT_CRITICAL();
}
