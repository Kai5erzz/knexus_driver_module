#ifndef KNX26_APP_H
#define KNX26_APP_H

#include "knx_can_bus.h"
#include "knx_chassis.h"
#include "knx_comm.h"
#include "knx_imu.h"
#include "knx_intersection.h"
#include "knx_track.h"

typedef struct knx26_context {
    uint32_t now_ms;
    bool ready;
    knx_chassis_state_t chassis;
    knx_imu_data_t imu;
    knx_track_state_t track;
    knx_intersection_result_t intersection;
    knx_can_bus_state_t can[KNX_CAN_BUS_COUNT];
    knx_comm_state_t links[KNX_COMM_CHANNEL_COUNT];
} knx26_context_t;

knx_status_t knx26_app_init(void);
bool knx26_app_is_ready(void);
void knx26_fast_update(void);
void knx26_control_update(float dt_s);
void knx26_track_update(void);
void knx26_comm_update(void);
void knx26_user_app_update(void);
void knx26_debug_update(void);
void knx26_context_snapshot(knx26_context_t *out);

knx_status_t knx26_attach_board_link(knx_host_comm_t *transport);
knx_status_t knx26_attach_peer_link(knx_host_comm_t *transport);

#endif
