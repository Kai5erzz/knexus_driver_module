#ifndef KNX26_CONFIG_H
#define KNX26_CONFIG_H

#include "knx_target_config.h"

#define KNX26_FAST_PERIOD_MS       1U
#define KNX26_CONTROL_PERIOD_MS    5U
#define KNX26_TRACK_PERIOD_MS     10U
#define KNX26_APP_PERIOD_MS       20U
#define KNX26_DEBUG_PERIOD_MS    100U
#define KNX26_CAN_HEARTBEAT_MS   100U
#define KNX26_LINK_HEARTBEAT_MS  250U
#define KNX26_OCTO_BASE          700U

#if defined(KNX_PLATFORM_STM32)
#define KNX26_LOCAL_NODE_ID        1U
#define KNX26_ROLE_MAIN_CONTROLLER 1
#else
#define KNX26_LOCAL_NODE_ID        2U
#define KNX26_ROLE_MAIN_CONTROLLER 0
#endif

#endif
