#include "knx_telemetry_task.h"
#include "knx_app.h"
#include "knx_blackbox.h"
#include "knx_board.h"
#include "knx_debug_config.h"
#include "knx_gimbal_ctrl.h"
#include "knx_grayscale.h"
#include "knx_health.h"
#include "knx_key_debug.h"
#include "knx_param_host.h"
#include "knx_project_config.h"
#include "knx_safety.h"
#include "knx_telemetry.h"
#include "knx_vision.h"
#include "dm_imu_l1.h"
#include "octolinker.h"
#include "cmsis_os2.h"

#define KNX_TELEMETRY_TASK_STACK_SIZE  3072U
#define KNX_TELEMETRY_TASK_PRIORITY    osPriorityLow
#define KNX_TELEMETRY_TASK_PERIOD_MS   50U
#define KNX_TELEMETRY_COMPACT_BASE     280U

static osThreadId_t knx_telemetry_task_handle;

static void knx_telemetry_task_send_compact_gimbal(Octolinker_Instance_t *octo)
{
    if (octo == NULL) {
        return;
    }

    const uint16_t base = KNX_TELEMETRY_COMPACT_BASE;
    knx_health_record_t imu_health = {0};
    knx_health_record_t jc_health = {0};
    knx_blackbox_event_t latest_event = {0};
    knx_host_comm_stats_t host_stats = {0};

    (void)knx_health_get(KNX_HEALTH_SOURCE_DM_IMU_L1, &imu_health);
    (void)knx_health_get(KNX_HEALTH_SOURCE_JC, &jc_health);
    (void)knx_blackbox_latest(&latest_event);
    knx_vision_get_host_stats(&host_stats);

    (void)Octolinker_SendF32(octo, base + 0U, knx_gimbal_ctrl_state);
    (void)Octolinker_SendU8(octo, base + 1U, (uint8_t)knx_safety_get_level());
    (void)Octolinker_SendU32(octo, base + 2U, knx_safety_get_faults());
    (void)Octolinker_SendU32(octo, base + 3U, knx_safety_get_warnings());
    (void)Octolinker_SendF32(octo, base + 4U, dm_imu_l1_data.yaw_total);
    (void)Octolinker_SendF32(octo, base + 5U, dm_imu_l1_data.pitch);
    (void)Octolinker_SendF32(octo, base + 6U, knx_gimbal_ctrl_yaw_cmd_rpm);
    (void)Octolinker_SendF32(octo, base + 7U, knx_gimbal_ctrl_pitch_cmd_rpm);
    (void)Octolinker_SendF32(octo, base + 8U, knx_gimbal_ctrl_tracking_active);
    (void)Octolinker_SendF32(octo, base + 9U, knx_gimbal_ctrl_vision_valid);
    (void)Octolinker_SendF32(octo, base + 10U, knx_vision_err_x);
    (void)Octolinker_SendF32(octo, base + 11U, knx_vision_err_y);
    (void)Octolinker_SendF32(octo, base + 12U, knx_vision_conf);
    (void)Octolinker_SendU32(octo, base + 13U, knx_vision_age_ms());
    (void)Octolinker_SendU32(octo, base + 14U, knx_health_age_ms(KNX_HEALTH_SOURCE_DM_IMU_L1));
    (void)Octolinker_SendU32(octo, base + 15U, dm_imu_l1_data.rx_count);
    (void)Octolinker_SendI32(octo, base + 16U, imu_health.status);
    (void)Octolinker_SendI32(octo, base + 17U, jc_health.status);
    (void)Octolinker_SendI32(octo, base + 18U, knx_gimbal_ctrl_last_status);
    (void)Octolinker_SendU32(octo, base + 19U, knx_board_host_comm_rx_overflow_count());
    (void)Octolinker_SendU32(octo, base + 20U, knx_param_host_error_count);
    (void)Octolinker_SendU32(octo, base + 21U, latest_event.code);
    (void)Octolinker_SendI32(octo, base + 22U, latest_event.status);
    (void)Octolinker_SendU32(octo, base + 23U, knx_board_host_comm_rx_irq_count());
    (void)Octolinker_SendU32(octo, base + 24U, host_stats.rx_bytes);
    (void)Octolinker_SendU32(octo, base + 25U, host_stats.rx_frames);
    (void)Octolinker_SendU32(octo, base + 26U, host_stats.crc_errors);
    (void)Octolinker_SendU32(octo, base + 27U, host_stats.sync_losses);
    (void)Octolinker_SendU32(octo, base + 28U, host_stats.eof_errors);
    (void)Octolinker_SendU32(octo, base + 29U, knx_vision_payload_len_errors);
    (void)Octolinker_SendU32(octo, base + 30U, knx_board_host_comm_rx_restart_errors());
    (void)Octolinker_SendU32(octo, base + 31U, knx_board_host_comm_uart_error_count());
    (void)Octolinker_SendU32(octo, base + 32U, knx_board_host_comm_uart_error_code());
    (void)Octolinker_SendU32(octo, base + 33U, knx_board_host_comm_rx_last_byte());
    (void)Octolinker_SendU32(octo, base + 34U, knx_board_host_comm_uart_rx_state());
    (void)Octolinker_SendU32(octo, base + 35U, knx_board_host_comm_uart_start_status());
}

__attribute__((noreturn))
static void knx_telemetry_task_entry(void *argument)
{
    (void)argument;

    while (!knx_app_is_initialized()) {
        osDelay(1U);
    }

    Octolinker_Instance_t *octo = knx_board_get_octolinker();
    uint32_t next_wake = osKernelGetTickCount();

    for (;;) {
#if (KNX_MODULE_GIMBAL_EN) && \
    (KNX_ACTIVE_TEST_MODE == KNX_ACTIVE_TEST_MODE_NONE)
        knx_telemetry_task_send_compact_gimbal(octo);
#elif (KNX_ACTIVE_TEST_MODE != KNX_ACTIVE_TEST_MODE_STM32_BOARD_BRINGUP)
        (void)knx_telemetry_update();
        knx_key_debug_octo(octo);
        knx_grayscale_debug_octo(octo);
#endif

        next_wake += KNX_TELEMETRY_TASK_PERIOD_MS;
        osDelayUntil(next_wake);
    }
}

void knx_telemetry_task_init(void)
{
    const osThreadAttr_t telemetry_task_attributes = {
        .name = "knx_telemetry",
        .stack_size = KNX_TELEMETRY_TASK_STACK_SIZE,
        .priority = (osPriority_t)KNX_TELEMETRY_TASK_PRIORITY,
    };

    knx_telemetry_task_handle = osThreadNew(knx_telemetry_task_entry, NULL, &telemetry_task_attributes);
}
