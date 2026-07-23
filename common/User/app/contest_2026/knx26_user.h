#ifndef KNX26_USER_H
#define KNX26_USER_H

#include "knx_comm.h"
#include "knx_intersection.h"

struct knx26_context;

void knx26_user_init(void);
void knx26_user_update(const struct knx26_context *context);
void knx26_user_on_intersection(const knx_intersection_result_t *result);
void knx26_user_on_peer_message(const knx_comm_message_t *message);

#endif
