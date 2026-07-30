#include "knx26_mspm0_tasks.h"
#include "knx26_app.h"
#include "knx26_config.h"
#include "knx26_user.h"
#include "FreeRTOS.h"
#include "task.h"

volatile uint32_t knx26_diag_app_heartbeat;
volatile uint32_t knx26_diag_fast_heartbeat;
volatile uint32_t knx26_diag_control_heartbeat;
volatile uint32_t knx26_diag_track_heartbeat;
volatile uint32_t knx26_diag_perception_heartbeat;
volatile uint32_t knx26_diag_comm_heartbeat;
volatile uint32_t knx26_diag_debug_heartbeat;
volatile uint32_t knx26_diag_app_init_result;
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

/* vTaskDelayUntil() returns immediately when a task has already missed its
 * deadline.  A permanently-overloaded high-priority task would therefore
 * loop without ever blocking and starve LED, track and OctoLink tasks.
 * After an overrun, deliberately block for one tick and restart the phase. */
static void delay_periodic(TickType_t *wake,
                           TickType_t period,
                           TickType_t started,
                           volatile uint32_t *overrun_count,
                           volatile uint32_t *max_elapsed_ms)
{
    TickType_t now = xTaskGetTickCount();
    uint32_t elapsed_ms = (uint32_t)((now - started) * portTICK_PERIOD_MS);
    if (elapsed_ms > *max_elapsed_ms) {
        *max_elapsed_ms = elapsed_ms;
    }

    if ((TickType_t)(now - *wake) >= period) {
        (*overrun_count)++;
        *wake = now;
        vTaskDelay(1U);
        *wake = xTaskGetTickCount();
        return;
    }

    vTaskDelayUntil(wake, period);
}

static TickType_t start_periodic(uint32_t phase_ms)
{
    if (phase_ms != 0U) {
        vTaskDelay(pdMS_TO_TICKS(phase_ms));
    }
    return xTaskGetTickCount();
}

static void wait_ready(void)
{
    while (!knx26_app_is_ready()) vTaskDelay(pdMS_TO_TICKS(1U));
}

static void app_task(void *arg)
{
    (void)arg;
    knx26_diag_app_init_result = (uint32_t)knx26_app_init();
    TickType_t wake = start_periodic(KNX26_APP_PHASE_MS);
    for (;;) {
        TickType_t started = xTaskGetTickCount();
        knx26_diag_app_heartbeat++;
        knx26_user_app_update();
        delay_periodic(&wake, pdMS_TO_TICKS(KNX26_APP_PERIOD_MS), started,
                       &knx26_diag_app_overrun,
                       &knx26_diag_app_max_elapsed_ms);
    }
}

static void fast_task(void *arg)
{
    (void)arg; wait_ready();
    TickType_t wake = start_periodic(KNX26_FAST_PHASE_MS);
    for (;;) {
        TickType_t started = xTaskGetTickCount();
        knx26_diag_fast_heartbeat++;
        knx26_fast_update();
        delay_periodic(&wake, pdMS_TO_TICKS(KNX26_FAST_PERIOD_MS), started,
                       &knx26_diag_fast_overrun,
                       &knx26_diag_fast_max_elapsed_ms);
    }
}

static void control_task(void *arg)
{
    (void)arg; wait_ready();
    TickType_t wake = start_periodic(KNX26_CONTROL_PHASE_MS);
    for (;;) {
        TickType_t started = xTaskGetTickCount();
        knx26_diag_control_heartbeat++;
        knx26_control_update((float)KNX26_CONTROL_PERIOD_MS * 0.001f);
#if defined(KNEXUS_MODE_LINE_FOLLOW) && KNEXUS_H_TASK_ENABLE
        knx26_h_ball_control_update();
#endif
        delay_periodic(&wake, pdMS_TO_TICKS(KNX26_CONTROL_PERIOD_MS), started,
                       &knx26_diag_control_overrun,
                       &knx26_diag_control_max_elapsed_ms);
    }
}

static void track_task(void *arg)
{
    (void)arg; wait_ready();
    TickType_t wake = start_periodic(KNX26_TRACK_PHASE_MS);
    for (;;) {
        TickType_t started = xTaskGetTickCount();
        knx26_diag_track_heartbeat++;
        knx26_track_update();
        delay_periodic(&wake, pdMS_TO_TICKS(KNX26_TRACK_PERIOD_MS), started,
                       &knx26_diag_track_overrun,
                       &knx26_diag_track_max_elapsed_ms);
    }
}

static void comm_task(void *arg)
{
    (void)arg; wait_ready();
    TickType_t wake = start_periodic(KNX26_COMM_PHASE_MS);
    for (;;) {
        TickType_t started = xTaskGetTickCount();
        knx26_diag_comm_heartbeat++;
        knx26_comm_update();
        delay_periodic(&wake, pdMS_TO_TICKS(KNX26_COMM_PERIOD_MS), started,
                       &knx26_diag_comm_overrun,
                       &knx26_diag_comm_max_elapsed_ms);
    }
}

#if KNX26_INTERSECTION_EN
static void perception_task(void *arg)
{
    (void)arg; wait_ready();
    TickType_t wake = start_periodic(KNX26_PERCEPTION_PHASE_MS);
    for (;;) {
        TickType_t started = xTaskGetTickCount();
        knx26_diag_perception_heartbeat++;
        knx26_perception_update();
        delay_periodic(&wake, pdMS_TO_TICKS(KNX26_PERCEPTION_PERIOD_MS),
                       started, &knx26_diag_perception_overrun,
                       &knx26_diag_perception_max_elapsed_ms);
    }
}
#endif

static void debug_task(void *arg)
{
    (void)arg; wait_ready();
    TickType_t wake = start_periodic(KNX26_DEBUG_PHASE_MS);
    for (;;) {
        TickType_t started = xTaskGetTickCount();
        knx26_diag_debug_heartbeat++;
        knx26_debug_update();
        delay_periodic(&wake, pdMS_TO_TICKS(KNX26_DEBUG_PERIOD_MS), started,
                       &knx26_diag_debug_overrun,
                       &knx26_diag_debug_max_elapsed_ms);
    }
}

knx_status_t knx26_mspm0_tasks_init(void)
{
    if (xTaskCreate(app_task, "knx26_app", 640U, NULL, tskIDLE_PRIORITY + 3U, NULL) != pdPASS) return KNX_ERROR;
    if (xTaskCreate(fast_task, "knx26_fast", 384U, NULL, tskIDLE_PRIORITY + 5U, NULL) != pdPASS) return KNX_ERROR;
    if (xTaskCreate(control_task, "knx26_ctrl", 640U, NULL, tskIDLE_PRIORITY + 4U, NULL) != pdPASS) return KNX_ERROR;
    if (xTaskCreate(track_task, "knx26_track", 384U, NULL, tskIDLE_PRIORITY + 3U, NULL) != pdPASS) return KNX_ERROR;
    if (xTaskCreate(comm_task, "knx26_comm", 384U, NULL, tskIDLE_PRIORITY + 2U, NULL) != pdPASS) return KNX_ERROR;
#if KNX26_INTERSECTION_EN
    if (xTaskCreate(perception_task, "knx26_percept", 384U, NULL, tskIDLE_PRIORITY + 2U, NULL) != pdPASS) return KNX_ERROR;
#endif
    if (xTaskCreate(debug_task, "knx26_debug", 384U, NULL, tskIDLE_PRIORITY + 1U, NULL) != pdPASS) return KNX_ERROR;
    return KNX_OK;
}
