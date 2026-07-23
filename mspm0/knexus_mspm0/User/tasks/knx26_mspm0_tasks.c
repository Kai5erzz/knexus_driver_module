#include "knx26_mspm0_tasks.h"
#include "knx26_app.h"
#include "knx26_config.h"
#include "FreeRTOS.h"
#include "task.h"

static void wait_ready(void)
{
    while (!knx26_app_is_ready()) vTaskDelay(pdMS_TO_TICKS(1U));
}

static void app_task(void *arg)
{
    (void)arg;
    (void)knx26_app_init();
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        knx26_user_app_update();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(KNX26_APP_PERIOD_MS));
    }
}

static void fast_task(void *arg)
{
    (void)arg; wait_ready(); TickType_t wake = xTaskGetTickCount();
    for (;;) { knx26_fast_update(); vTaskDelayUntil(&wake, pdMS_TO_TICKS(KNX26_FAST_PERIOD_MS)); }
}

static void control_task(void *arg)
{
    (void)arg; wait_ready(); TickType_t wake = xTaskGetTickCount();
    for (;;) {
        knx26_control_update((float)KNX26_CONTROL_PERIOD_MS * 0.001f);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(KNX26_CONTROL_PERIOD_MS));
    }
}

static void track_task(void *arg)
{
    (void)arg; wait_ready(); TickType_t wake = xTaskGetTickCount();
    for (;;) { knx26_track_update(); vTaskDelayUntil(&wake, pdMS_TO_TICKS(KNX26_TRACK_PERIOD_MS)); }
}

static void comm_task(void *arg)
{
    (void)arg; wait_ready(); TickType_t wake = xTaskGetTickCount();
    for (;;) { knx26_comm_update(); vTaskDelayUntil(&wake, pdMS_TO_TICKS(1U)); }
}

static void debug_task(void *arg)
{
    (void)arg; wait_ready(); TickType_t wake = xTaskGetTickCount();
    for (;;) { knx26_debug_update(); vTaskDelayUntil(&wake, pdMS_TO_TICKS(KNX26_DEBUG_PERIOD_MS)); }
}

knx_status_t knx26_mspm0_tasks_init(void)
{
    if (xTaskCreate(app_task, "knx26_app", 640U, NULL, tskIDLE_PRIORITY + 2U, NULL) != pdPASS) return KNX_ERROR;
    if (xTaskCreate(fast_task, "knx26_fast", 384U, NULL, tskIDLE_PRIORITY + 4U, NULL) != pdPASS) return KNX_ERROR;
    if (xTaskCreate(control_task, "knx26_ctrl", 640U, NULL, tskIDLE_PRIORITY + 3U, NULL) != pdPASS) return KNX_ERROR;
    if (xTaskCreate(track_task, "knx26_track", 384U, NULL, tskIDLE_PRIORITY + 2U, NULL) != pdPASS) return KNX_ERROR;
    if (xTaskCreate(comm_task, "knx26_comm", 384U, NULL, tskIDLE_PRIORITY + 3U, NULL) != pdPASS) return KNX_ERROR;
    if (xTaskCreate(debug_task, "knx26_debug", 384U, NULL, tskIDLE_PRIORITY + 1U, NULL) != pdPASS) return KNX_ERROR;
    return KNX_OK;
}
