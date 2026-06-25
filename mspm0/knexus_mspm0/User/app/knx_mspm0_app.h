#ifndef KNX_MSPM0_APP_H
#define KNX_MSPM0_APP_H

#include "knx_types.h"

knx_status_t knx_mspm0_app_init(void);
knx_status_t knx_mspm0_upper_init(void);
knx_status_t knx_mspm0_upper_fast_update(void);
knx_status_t knx_mspm0_upper_medium_update(float dt_s);
knx_status_t knx_mspm0_upper_slow_update(void);

#endif /* KNX_MSPM0_APP_H */
