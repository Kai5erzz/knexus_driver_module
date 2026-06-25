/*
 * knx_telemetry_mspm0_stub.c
 * Minimal telemetry for MSPM0 — just forwards to DRV8701E debug output.
 * Avoids pulling in knx_motor/knx_imu/knx_drive/knx_sys/bmi088.
 */
#include "knx_telemetry.h"
#include "octolinker.h"
#include "drv8701e.h"
#include "knx_encoder.h"

static Octolinker_Instance_t *s_octo;

void knx_telemetry_set_octo(void *octo)
{
    s_octo = (Octolinker_Instance_t *)octo;
}

knx_status_t knx_telemetry_init(void)
{
    return KNX_OK;
}

knx_status_t knx_telemetry_update(void)
{
    if (s_octo == NULL) return KNX_ERROR;

    DRV8701E_DebugOcto(s_octo);
    return KNX_OK;
}
