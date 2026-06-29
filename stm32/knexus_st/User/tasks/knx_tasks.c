#include "knx_tasks.h"
#include "knx_app_task.h"
#include "knx_comm_task.h"
#include "knx_ctrl_task.h"
#include "knx_safety_task.h"
#include "knx_telemetry_task.h"
#include "knx_ui_task.h"

void knx_tasks_init(void)
{
    knx_app_task_init();
    knx_comm_task_init();
    knx_safety_task_init();
    knx_ctrl_task_init();
    knx_ui_task_init();
    knx_telemetry_task_init();
}
