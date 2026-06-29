#include "knx_comm_task.h"
#include "knx_app.h"
#include "knx_board.h"
#include "knx_vision.h"
#include "cmsis_os2.h"

#define KNX_COMM_TASK_STACK_SIZE  2048U
#define KNX_COMM_TASK_PRIORITY    osPriorityAboveNormal
#define KNX_COMM_TASK_PERIOD_MS   1U
#define KNX_COMM_RX_CHUNK         64U

static osThreadId_t knx_comm_task_handle;

__attribute__((noreturn))
static void knx_comm_task_entry(void *argument)
{
    (void)argument;

    while (!knx_app_is_initialized()) {
        osDelay(1U);
    }

    uint8_t rx_buf[KNX_COMM_RX_CHUNK];
    uint32_t next_wake = osKernelGetTickCount();

    for (;;) {
        uint16_t n;
        int max_rounds = 8;
        do {
            n = knx_board_host_comm_read(rx_buf, sizeof(rx_buf));
            if (n > 0U) {
                (void)knx_vision_feed_bytes(rx_buf, n);
            }
        } while (n == sizeof(rx_buf) && --max_rounds > 0);

        next_wake += KNX_COMM_TASK_PERIOD_MS;
        osDelayUntil(next_wake);
    }
}

void knx_comm_task_init(void)
{
    const osThreadAttr_t comm_task_attributes = {
        .name = "knx_comm",
        .stack_size = KNX_COMM_TASK_STACK_SIZE,
        .priority = (osPriority_t)KNX_COMM_TASK_PRIORITY,
    };

    knx_comm_task_handle = osThreadNew(knx_comm_task_entry, NULL, &comm_task_attributes);
}
