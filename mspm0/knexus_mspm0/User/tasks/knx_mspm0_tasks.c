#include "knx_mspm0_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "knx_beep.h"
#include "knx_key.h"
#include "knx_led.h"
#include "knx_debug_config.h"
#include "knx_board.h"
#include "knx_mspm0_app.h"
#include "knx_project_config.h"
#if KNX_APP_CONTEST_2026
#include "knx26_mspm0_tasks.h"
#endif

#if (KNX_ACTIVE_TEST_MODE == KNX_ACTIVE_TEST_MODE_DRV8701E)
#include "test_drv8701e.h"
#elif (KNX_ACTIVE_TEST_MODE == KNX_ACTIVE_TEST_MODE_ENCODER)
#include "test_encoder.h"
#endif

#define HEARTBEAT_STACK_SIZE   128U
#define HEARTBEAT_PRIORITY     (tskIDLE_PRIORITY + 3U)

#define APP_STACK_SIZE         256U
#define APP_PRIORITY           (tskIDLE_PRIORITY + 2U)

#define BEEP_STACK_SIZE        128U
#define BEEP_PRIORITY          (tskIDLE_PRIORITY + 1U)

#define INIT_STACK_SIZE        512U
#define INIT_PRIORITY          (tskIDLE_PRIORITY + 2U)

#define CONTROL_STACK_SIZE     512U
#define CONTROL_PRIORITY       (tskIDLE_PRIORITY + 4U)

#define TEST_STACK_SIZE        512U
#define TEST_PRIORITY          (tskIDLE_PRIORITY + 4U)

static volatile uint8_t s_upper_initialized;

static void heartbeat_task(void *pvParameters)
{
    (void)pvParameters;
    for (;;) {
        knx_led_toggle(KNX_LED_0);
        vTaskDelay(pdMS_TO_TICKS(500U));
    }
}

static void app_task(void *pvParameters)
{
    (void)pvParameters;
    for (;;) {
        knx_key_update(10U);

        if (knx_key_just_pressed(KNX_KEY_0)) {
            knx_led_toggle(KNX_LED_1);
        }
        if (knx_key_just_pressed(KNX_KEY_1)) {
            knx_beep_beep(100U);
        }
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

static void beep_task(void *pvParameters)
{
    (void)pvParameters;
    for (;;) {
        knx_beep_update();
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

static void init_task(void *pvParameters)
{
    (void)pvParameters;

    if (knx_mspm0_upper_init() == KNX_OK) {
        s_upper_initialized = 1U;
        knx_led_set(KNX_LED_2, true);
    } else {
        knx_led_set(KNX_LED_2, false);
    }

    vTaskDelete(NULL);
}

static void control_task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t tick = 0U;

    for (;;) {
        if (s_upper_initialized != 0U) {
            (void)knx_mspm0_upper_fast_update();

            if ((tick % 10U) == 0U) {
                (void)knx_mspm0_upper_medium_update(0.01f);
            }
            if ((tick % 20U) == 0U) {
                (void)knx_mspm0_upper_slow_update();
            }
        }

        tick++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1U));
    }
}

/* ── Test mode task (1 ms for current loop) ── */
#if (KNX_ACTIVE_TEST_MODE != KNX_ACTIVE_TEST_MODE_NONE)
static void test_task(void *pvParameters)
{
    (void)pvParameters;

#if (KNX_ACTIVE_TEST_MODE == KNX_ACTIVE_TEST_MODE_DRV8701E)
    test_drv8701e_init(knx_board_get_octolinker());
#elif (KNX_ACTIVE_TEST_MODE == KNX_ACTIVE_TEST_MODE_ENCODER)
    test_encoder_init(knx_board_get_octolinker());
#endif

    for (;;) {
#if (KNX_ACTIVE_TEST_MODE == KNX_ACTIVE_TEST_MODE_DRV8701E)
        test_drv8701e_loop();
#elif (KNX_ACTIVE_TEST_MODE == KNX_ACTIVE_TEST_MODE_ENCODER)
        test_encoder_loop();
#endif
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}
#endif

knx_status_t knx_mspm0_tasks_init(void)
{
#if KNX_APP_CONTEST_2026
    return knx26_mspm0_tasks_init();
#else
    BaseType_t ok;

    ok = xTaskCreate(heartbeat_task, "heartbeat", HEARTBEAT_STACK_SIZE,
                     NULL, HEARTBEAT_PRIORITY, NULL);
    if (ok != pdPASS) return KNX_ERROR;

    ok = xTaskCreate(app_task, "app", APP_STACK_SIZE,
                     NULL, APP_PRIORITY, NULL);
    if (ok != pdPASS) return KNX_ERROR;

    ok = xTaskCreate(beep_task, "beep", BEEP_STACK_SIZE,
                     NULL, BEEP_PRIORITY, NULL);
    if (ok != pdPASS) return KNX_ERROR;

    ok = xTaskCreate(init_task, "init", INIT_STACK_SIZE,
                     NULL, INIT_PRIORITY, NULL);
    if (ok != pdPASS) return KNX_ERROR;

    ok = xTaskCreate(control_task, "control", CONTROL_STACK_SIZE,
                     NULL, CONTROL_PRIORITY, NULL);
    if (ok != pdPASS) return KNX_ERROR;

#if (KNX_ACTIVE_TEST_MODE != KNX_ACTIVE_TEST_MODE_NONE)
    ok = xTaskCreate(test_task, "test", TEST_STACK_SIZE,
                     NULL, TEST_PRIORITY, NULL);
    if (ok != pdPASS) return KNX_ERROR;
#endif

    return KNX_OK;
#endif
}
