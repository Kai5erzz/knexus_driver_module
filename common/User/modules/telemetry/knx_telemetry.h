#ifndef KNX_TELEMETRY_H
#define KNX_TELEMETRY_H

#include "knx_types.h"

/* Telemetry module — periodic debug output via OctoLink / VOFA+ */

void           knx_telemetry_set_octo(void *octo);
knx_status_t   knx_telemetry_init(void);
knx_status_t   knx_telemetry_update(void);           /* called periodically (20ms) */
knx_status_t   knx_telemetry_update_drive(void);

#endif /* KNX_TELEMETRY_H */
