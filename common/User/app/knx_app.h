#ifndef KNX_APP_H
#define KNX_APP_H

#include "knx_types.h"

/* Application entry points.
 * knx_app_init() — called once after CubeMX init.
 * knx_app_loop() — called repeatedly in the main while(1) loop.
 *                  Uses time-sliced scheduling internally (non-blocking). */

void knx_app_init(void);
void knx_app_loop(void);
bool knx_app_is_initialized(void);

#endif /* KNX_APP_H */
