#include "knx_tasks.h"
#include "knx_project_config.h"
#if KNX_APP_CONTEST_2026
#include "knx26_tasks.h"
#else
#include "knx_app_task.h"
#include "knx_comm_task.h"
#include "knx_ctrl_task.h"
#include "knx_safety_task.h"
#include "knx_telemetry_task.h"
#include "knx_ui_task.h"
#endif

void knx_tasks_init(void)
{
#if KNX_APP_CONTEST_2026
    knx26_tasks_init();
#else
    knx_app_task_init();
    knx_comm_task_init();
    knx_safety_task_init();
    knx_ctrl_task_init();
    knx_ui_task_init();
    knx_telemetry_task_init();
#endif
}
