#include "ti_msp_dl_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "knx_mspm0_app.h"
#include "knx_mspm0_tasks.h"

int main(void)
{
    SYSCFG_DL_init();

    (void)knx_mspm0_app_init();
    (void)knx_mspm0_tasks_init();

    vTaskStartScheduler();

    for (;;) {
    }
}
