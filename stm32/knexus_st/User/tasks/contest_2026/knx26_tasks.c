#include "knx26_tasks.h"
#include "knx26_app.h"
#include "knx26_config.h"
#include "knx26_user.h"
#include "knx_app.h"
#include "cmsis_os2.h"

/* 可由 GDB/OctoLink 读取的实时调度诊断。 */
volatile uint32_t knx26_diag_app_heartbeat;
volatile uint32_t knx26_diag_fast_heartbeat;
volatile uint32_t knx26_diag_control_heartbeat;
volatile uint32_t knx26_diag_track_heartbeat;
volatile uint32_t knx26_diag_perception_heartbeat;
volatile uint32_t knx26_diag_comm_heartbeat;
volatile uint32_t knx26_diag_debug_heartbeat;

volatile uint32_t knx26_diag_app_overrun;
volatile uint32_t knx26_diag_fast_overrun;
volatile uint32_t knx26_diag_control_overrun;
volatile uint32_t knx26_diag_track_overrun;
volatile uint32_t knx26_diag_perception_overrun;
volatile uint32_t knx26_diag_comm_overrun;
volatile uint32_t knx26_diag_debug_overrun;

volatile uint32_t knx26_diag_app_max_elapsed_ms;
volatile uint32_t knx26_diag_fast_max_elapsed_ms;
volatile uint32_t knx26_diag_control_max_elapsed_ms;
volatile uint32_t knx26_diag_track_max_elapsed_ms;
volatile uint32_t knx26_diag_perception_max_elapsed_ms;
volatile uint32_t knx26_diag_comm_max_elapsed_ms;
volatile uint32_t knx26_diag_debug_max_elapsed_ms;

/* 兼容已有全板测试的变量名。 */
volatile uint32_t knx26_fast_overrun_count;
volatile uint32_t knx26_fast_max_elapsed_ms;
volatile uint32_t knx26_task_create_error_mask;

static void wait_ready(void)
{
    while (!knx_app_is_initialized()) {
        osDelay(1U);
    }
}

static uint32_t start_periodic(uint32_t phase_ms)
{
    if (phase_ms != 0U) {
        osDelay(phase_ms);
    }
    return osKernelGetTickCount();
}

/*
 * osDelayUntil() 在截止时间已经过去时会立即返回。若继续沿用旧基准，
 * 高优先级任务会无休止追赶并饿死低优先级任务。这里统一记录超期、
 * 主动让出一个 tick，并从当前时刻重新建立周期相位。
 */
static void finish_periodic(uint32_t *wake,
                            uint32_t period_ms,
                            uint32_t started,
                            volatile uint32_t *overrun,
                            volatile uint32_t *max_elapsed_ms)
{
    uint32_t now = osKernelGetTickCount();
    uint32_t elapsed = now - started;
    if (elapsed > *max_elapsed_ms) {
        *max_elapsed_ms = elapsed;
    }

    uint32_t deadline = *wake + period_ms;
    if ((int32_t)(deadline - now) <= 0) {
        (*overrun)++;
        *wake = now + 1U;
        osDelayUntil(*wake);
        return;
    }

    *wake = deadline;
    osDelayUntil(deadline);
}

__attribute__((noreturn)) static void fast_task(void *argument)
{
    (void)argument;
    wait_ready();
    uint32_t wake = start_periodic(KNX26_FAST_PHASE_MS);
    for (;;) {
        uint32_t started = osKernelGetTickCount();
        knx26_diag_fast_heartbeat++;
        knx26_fast_update();
        finish_periodic(&wake, KNX26_FAST_PERIOD_MS, started,
                        &knx26_diag_fast_overrun,
                        &knx26_diag_fast_max_elapsed_ms);
        knx26_fast_overrun_count = knx26_diag_fast_overrun;
        knx26_fast_max_elapsed_ms = knx26_diag_fast_max_elapsed_ms;
    }
}

__attribute__((noreturn)) static void control_task(void *argument)
{
    (void)argument;
    wait_ready();
    uint32_t wake = start_periodic(KNX26_CONTROL_PHASE_MS);
    for (;;) {
        uint32_t started = osKernelGetTickCount();
        knx26_diag_control_heartbeat++;
        knx26_control_update((float)KNX26_CONTROL_PERIOD_MS * 0.001f);
#if defined(KNEXUS_MODE_LINE_FOLLOW) && KNEXUS_H_TASK_ENABLE
        knx26_h_ball_control_update();
#endif
        finish_periodic(&wake, KNX26_CONTROL_PERIOD_MS, started,
                        &knx26_diag_control_overrun,
                        &knx26_diag_control_max_elapsed_ms);
    }
}

__attribute__((noreturn)) static void track_task(void *argument)
{
    (void)argument;
    wait_ready();
    uint32_t wake = start_periodic(KNX26_TRACK_PHASE_MS);
    for (;;) {
        uint32_t started = osKernelGetTickCount();
        knx26_diag_track_heartbeat++;
        knx26_track_update();
        finish_periodic(&wake, KNX26_TRACK_PERIOD_MS, started,
                        &knx26_diag_track_overrun,
                        &knx26_diag_track_max_elapsed_ms);
    }
}

__attribute__((noreturn)) static void comm_task(void *argument)
{
    (void)argument;
    wait_ready();
    uint32_t wake = start_periodic(KNX26_COMM_PHASE_MS);
    for (;;) {
        uint32_t started = osKernelGetTickCount();
        knx26_diag_comm_heartbeat++;
        knx26_comm_update();
        finish_periodic(&wake, KNX26_COMM_PERIOD_MS, started,
                        &knx26_diag_comm_overrun,
                        &knx26_diag_comm_max_elapsed_ms);
    }
}

#if KNX26_INTERSECTION_EN
__attribute__((noreturn)) static void perception_task(void *argument)
{
    (void)argument;
    wait_ready();
    uint32_t wake = start_periodic(KNX26_PERCEPTION_PHASE_MS);
    for (;;) {
        uint32_t started = osKernelGetTickCount();
        knx26_diag_perception_heartbeat++;
        knx26_perception_update();
        finish_periodic(&wake, KNX26_PERCEPTION_PERIOD_MS, started,
                        &knx26_diag_perception_overrun,
                        &knx26_diag_perception_max_elapsed_ms);
    }
}
#endif

__attribute__((noreturn)) static void app_task(void *argument)
{
    (void)argument;
    knx_app_init();
    uint32_t wake = start_periodic(KNX26_APP_PHASE_MS);
    for (;;) {
        uint32_t started = osKernelGetTickCount();
        knx26_diag_app_heartbeat++;
        knx_app_loop();
        finish_periodic(&wake, KNX26_APP_PERIOD_MS, started,
                        &knx26_diag_app_overrun,
                        &knx26_diag_app_max_elapsed_ms);
    }
}

__attribute__((noreturn)) static void debug_task(void *argument)
{
    (void)argument;
    wait_ready();
    uint32_t wake = start_periodic(KNX26_DEBUG_PHASE_MS);
    for (;;) {
        uint32_t started = osKernelGetTickCount();
        knx26_diag_debug_heartbeat++;
        knx26_debug_update();
        finish_periodic(&wake, KNX26_DEBUG_PERIOD_MS, started,
                        &knx26_diag_debug_overrun,
                        &knx26_diag_debug_max_elapsed_ms);
    }
}

static void create_task(osThreadFunc_t entry, const char *name,
                        uint32_t stack, osPriority_t priority,
                        uint32_t error_bit)
{
    const osThreadAttr_t attr = {
        .name = name,
        .stack_size = stack,
        .priority = priority,
    };
    if (osThreadNew(entry, NULL, &attr) == NULL) {
        knx26_task_create_error_mask |= error_bit;
    }
}

void knx26_tasks_init(void)
{
    knx26_task_create_error_mask = 0U;

    /*
     * 优先级从高到低：硬实时快环 > 运动控制 > 灰度采样 > 策略 >
     * 通信 > 路口推理 > 调试。
     * 通信和串口输出都可能等待硬件，因此不能与电机/底盘控制同级。
     */
    create_task(app_task,     "knx26_app",   4096U, osPriorityAboveNormal, 0x01U);
    create_task(fast_task,    "knx26_fast",  3072U, osPriorityHigh,        0x02U);
    create_task(control_task, "knx26_ctrl",  4096U, osPriorityAboveNormal2,0x04U);
    create_task(track_task,   "knx26_track", 3072U, osPriorityAboveNormal1,0x08U);
    create_task(comm_task,    "knx26_comm",  3072U, osPriorityNormal1,     0x10U);
#if KNX26_INTERSECTION_EN
    create_task(perception_task, "knx26_percept", 4096U, osPriorityNormal, 0x20U);
#endif
    create_task(debug_task,   "knx26_debug", 3072U, osPriorityLow,         0x40U);
}
