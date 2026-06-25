#ifndef KNX_PARAM_HOST_H
#define KNX_PARAM_HOST_H

#include "knx_host_comm.h"
#include "knx_types.h"
#include <stdint.h>

#define KNX_PARAM_HOST_MSG_GET_REQ     0x30U
#define KNX_PARAM_HOST_MSG_SET_REQ     0x31U
#define KNX_PARAM_HOST_MSG_RESET_REQ   0x32U
#define KNX_PARAM_HOST_MSG_INFO_REQ    0x33U

#define KNX_PARAM_HOST_MSG_VALUE_RSP   0xB0U
#define KNX_PARAM_HOST_MSG_INFO_RSP    0xB1U
#define KNX_PARAM_HOST_MSG_ACK_RSP     0xB2U

knx_status_t knx_param_host_attach(knx_host_comm_t *comm);

extern uint32_t knx_param_host_rx_count;
extern uint32_t knx_param_host_tx_count;
extern uint32_t knx_param_host_error_count;
extern int32_t knx_param_host_last_status;
extern uint16_t knx_param_host_last_id;
extern uint8_t knx_param_host_last_msg;

#endif /* KNX_PARAM_HOST_H */
