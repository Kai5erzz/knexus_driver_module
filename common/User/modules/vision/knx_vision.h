#ifndef KNX_VISION_H
#define KNX_VISION_H

#include "knx_host_comm.h"
#include "knx_types.h"
#include "octolinker.h"
#include <stdint.h>

#define KNX_VISION_MSG_ID      0x10U
#define KNX_VISION_PAYLOAD_LEN 13U

typedef struct {
    float err_x;
    float err_y;
    float conf;
    uint32_t timestamp_ms;
    uint32_t rx_count;
    uint8_t valid;
} knx_vision_target_t;

knx_status_t knx_vision_init(void);
void knx_vision_attach_host_comm(knx_host_comm_t *comm);
knx_status_t knx_vision_feed_bytes(const uint8_t *data, uint16_t len);

void knx_vision_snapshot(knx_vision_target_t *target);
uint8_t knx_vision_is_fresh(uint32_t timeout_ms, float min_conf);
uint32_t knx_vision_age_ms(void);

void knx_vision_get_host_stats(knx_host_comm_stats_t *stats);
void knx_vision_debug_octo(Octolinker_Instance_t *octo, uint16_t base_id);

extern float knx_vision_err_x;
extern float knx_vision_err_y;
extern float knx_vision_conf;
extern float knx_vision_valid;
extern float knx_vision_age_ms_f;
extern uint32_t knx_vision_rx_count;
extern uint32_t knx_vision_payload_len_errors;
extern uint32_t knx_vision_msg_id_errors;

#endif /* KNX_VISION_H */
