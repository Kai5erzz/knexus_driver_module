#ifndef KNEXUS_H_HOOKS_H
#define KNEXUS_H_HOOKS_H

#include <stdbool.h>
#include <stdint.h>

struct knx26_context;

/* H题小球控制请求。底盘模式只发布目标，不接触摄像头、舵机或摆杆算法。 */
typedef enum {
    KNEXUS_H_BALL_DISABLED = 0,
    KNEXUS_H_BALL_HOLD_POSITION,
    KNEXUS_H_BALL_STATIC_SEQUENCE,
} knexus_h_ball_mode_t;

typedef struct {
    knexus_h_ball_mode_t mode;
    float target_position_cm;
} knexus_h_ball_request_t;

typedef struct {
    bool implemented;
    bool ready;
    bool fault;
    float target_position_cm;
    float measured_position_cm;
    float control_output;
} knexus_h_ball_status_t;

void knexus_h_ball_init(void);
void knexus_h_ball_request_hold(float target_position_cm);
void knexus_h_ball_request_static_sequence(void);
void knexus_h_ball_disable(void);
void knexus_h_ball_update(const struct knx26_context *context, float dt_s);
void knexus_h_ball_snapshot(knexus_h_ball_status_t *out);

/*
 * 小球组只需在自己的源文件中提供以下两个同名强符号，即可替换默认空实现：
 * - user_init：初始化摄像头位置输入、摆杆执行器和控制器；
 * - user_update：执行一次非阻塞球控，并填写状态。
 */
void knexus_h_ball_user_init(void);
void knexus_h_ball_user_update(const knexus_h_ball_request_t *request,
                               const struct knx26_context *context,
                               float dt_s,
                               knexus_h_ball_status_t *status);

/* 显示组可覆盖此弱钩子，将计时显示到不大于2英寸的屏幕。 */
void knexus_h_display_time_ms(uint32_t elapsed_ms,
                              bool running,
                              bool completed);

#endif
