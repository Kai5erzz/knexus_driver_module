#include "knx_vision.h"
#include "knx_health.h"
#include "knx_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static knx_host_comm_t *s_host_comm;
static knx_vision_target_t s_target;

float knx_vision_err_x = 0.0f;
float knx_vision_err_y = 0.0f;
float knx_vision_conf = 0.0f;
float knx_vision_valid = 0.0f;
float knx_vision_age_ms_f = 0.0f;
uint32_t knx_vision_rx_count = 0U;
uint32_t knx_vision_payload_len_errors = 0U;
uint32_t knx_vision_msg_id_errors = 0U;

static float knx_vision_read_f32_le(const uint8_t *data)
{
    float value;
    memcpy(&value, data, sizeof(value));
    return value;
}

static uint32_t knx_vision_age_from_timestamp(uint32_t timestamp)
{
    if (timestamp == 0U) {
        return 0xFFFFFFFFU;
    }
    return knx_millis() - timestamp;
}

static void knx_vision_sync_mirrors_locked(void)
{
    knx_vision_err_x = s_target.err_x;
    knx_vision_err_y = s_target.err_y;
    knx_vision_conf = s_target.conf;
    knx_vision_rx_count = s_target.rx_count;
    knx_vision_valid = (float)s_target.valid;
    knx_vision_age_ms_f = (float)knx_vision_age_from_timestamp(s_target.timestamp_ms);
}

static void knx_vision_payload_callback(const uint8_t *payload,
                                        uint16_t len,
                                        void *user)
{
    (void)user;

    if (len == 0U || payload[0] != KNX_VISION_MSG_ID) {
        return;
    }

    if (len != KNX_VISION_PAYLOAD_LEN) {
        knx_vision_payload_len_errors++;
        (void)knx_health_report(KNX_HEALTH_SOURCE_VISION,
                                KNX_HEALTH_STATE_WARN,
                                KNX_INVALID_ARG,
                                len,
                                knx_vision_payload_len_errors + knx_vision_msg_id_errors);
        return;
    }

    taskENTER_CRITICAL();
    s_target.err_x = knx_vision_read_f32_le(&payload[1]);
    s_target.err_y = knx_vision_read_f32_le(&payload[5]);
    s_target.conf = knx_vision_read_f32_le(&payload[9]);
    s_target.timestamp_ms = knx_millis();
    s_target.rx_count++;
    s_target.valid = 1U;
    knx_vision_sync_mirrors_locked();
    taskEXIT_CRITICAL();

    (void)knx_health_report(KNX_HEALTH_SOURCE_VISION,
                            KNX_HEALTH_STATE_OK,
                            KNX_OK,
                            1U,
                            knx_vision_payload_len_errors + knx_vision_msg_id_errors);
}

knx_status_t knx_vision_init(void)
{
    taskENTER_CRITICAL();
    s_host_comm = NULL;
    memset(&s_target, 0, sizeof(s_target));
    knx_vision_payload_len_errors = 0U;
    knx_vision_msg_id_errors = 0U;
    knx_vision_sync_mirrors_locked();
    taskEXIT_CRITICAL();
    (void)knx_health_report(KNX_HEALTH_SOURCE_VISION,
                            KNX_HEALTH_STATE_STALE,
                            KNX_OK,
                            0U,
                            0U);
    return KNX_OK;
}

void knx_vision_attach_host_comm(knx_host_comm_t *comm)
{
    s_host_comm = comm;
    if (s_host_comm != NULL) {
        knx_host_comm_set_callback(s_host_comm, knx_vision_payload_callback, NULL);
        knx_host_comm_reset_stats(s_host_comm);
    }
}

knx_status_t knx_vision_feed_bytes(const uint8_t *data, uint16_t len)
{
    if (s_host_comm == NULL) {
        return KNX_NOT_READY;
    }
    return knx_host_comm_feed(s_host_comm, data, len);
}

void knx_vision_snapshot(knx_vision_target_t *target)
{
    if (target == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *target = s_target;
    knx_vision_sync_mirrors_locked();
    taskEXIT_CRITICAL();
}

uint32_t knx_vision_age_ms(void)
{
    uint32_t timestamp;
    taskENTER_CRITICAL();
    timestamp = s_target.timestamp_ms;
    taskEXIT_CRITICAL();

    if (timestamp == 0U) {
        return knx_vision_age_from_timestamp(timestamp);
    }
    return knx_vision_age_from_timestamp(timestamp);
}

uint8_t knx_vision_is_fresh(uint32_t timeout_ms, float min_conf)
{
    knx_vision_target_t target;
    knx_vision_snapshot(&target);
    if (target.valid == 0U || target.conf < min_conf) {
        return 0U;
    }
    return (knx_vision_age_ms() <= timeout_ms) ? 1U : 0U;
}

void knx_vision_get_host_stats(knx_host_comm_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    memset(stats, 0, sizeof(*stats));
    if (s_host_comm != NULL) {
        knx_host_comm_get_stats(s_host_comm, stats);
    }
}

void knx_vision_debug_octo(Octolinker_Instance_t *octo, uint16_t base_id)
{
    if (octo == NULL) {
        return;
    }

    knx_vision_target_t target;
    knx_host_comm_stats_t stats;
    knx_vision_snapshot(&target);
    knx_vision_get_host_stats(&stats);

    uint32_t age_ms = knx_vision_age_ms();
    knx_vision_age_ms_f = (float)age_ms;

    (void)Octolinker_SendF32(octo, base_id + 0U, target.err_x);
    (void)Octolinker_SendF32(octo, base_id + 1U, target.err_y);
    (void)Octolinker_SendF32(octo, base_id + 2U, target.conf);
    (void)Octolinker_SendF32(octo, base_id + 3U, (float)target.valid);
    (void)Octolinker_SendF32(octo, base_id + 4U, (float)age_ms);
    (void)Octolinker_SendU32(octo, base_id + 5U, target.rx_count);
    (void)Octolinker_SendU32(octo, base_id + 6U, knx_vision_payload_len_errors);
    (void)Octolinker_SendU32(octo, base_id + 7U, knx_vision_msg_id_errors);
    (void)Octolinker_SendU32(octo, base_id + 8U, stats.rx_frames);
    (void)Octolinker_SendU32(octo, base_id + 9U, stats.crc_errors);
    (void)Octolinker_SendU32(octo, base_id + 10U, stats.sync_losses);
}
