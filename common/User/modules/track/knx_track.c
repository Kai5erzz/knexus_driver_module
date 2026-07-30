#include "knx_track.h"
#include "knx_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static knx_track_state_t s_track;
static uint16_t s_black[KNX_GRAYSCALE_CH_NUM];
static uint16_t s_white[KNX_GRAYSCALE_CH_NUM];
static bool s_have_black;
static bool s_have_white;

void knx_track_init(void)
{
    memset(&s_track, 0, sizeof(s_track));
    s_have_black = false;
    s_have_white = false;
    knx_grayscale_init();
}

knx_status_t knx_track_update(void)
{
    knx_status_t status = knx_grayscale_update();
    if (status != KNX_OK) {
        /* Preserve the last complete frame and its timestamp.  Consumers can
         * tolerate an isolated ADC timeout and still detect a sustained loss
         * by checking the age of the last successful sample. */
        taskENTER_CRITICAL();
        s_track.last_status = status;
        taskEXIT_CRITICAL();
        return status;
    }

    knx_track_state_t next;
    taskENTER_CRITICAL();
    next = s_track;
    taskEXIT_CRITICAL();
    next.last_status = status;
    knx_grayscale_snapshot(&next.sensor);
    next.line_strength = 0.0f;
    for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
        next.line_strength += next.sensor.normalized[i];
    }
    next.line_lost = (next.line_strength < 0.15f);
    next.timestamp_ms = knx_millis();
    next.update_count++;
    taskENTER_CRITICAL();
    s_track = next;
    taskEXIT_CRITICAL();
    return KNX_OK;
}

static void apply_calibration_if_ready(void)
{
    if (s_have_black && s_have_white) {
        knx_grayscale_set_calibration(s_black, s_white);
    }
}

knx_status_t knx_track_capture_black(void)
{
    (void)knx_track_update();
    knx_track_state_t snapshot;
    knx_track_snapshot(&snapshot);
    memcpy(s_black, snapshot.sensor.raw, sizeof(s_black));
    s_have_black = true;
    apply_calibration_if_ready();
    return KNX_OK;
}

knx_status_t knx_track_capture_white(void)
{
    (void)knx_track_update();
    knx_track_state_t snapshot;
    knx_track_snapshot(&snapshot);
    memcpy(s_white, snapshot.sensor.raw, sizeof(s_white));
    s_have_white = true;
    apply_calibration_if_ready();
    return KNX_OK;
}

void knx_track_set_calibration(const uint16_t black[KNX_GRAYSCALE_CH_NUM],
                               const uint16_t white[KNX_GRAYSCALE_CH_NUM])
{
    if (black == NULL || white == NULL) return;
    memcpy(s_black, black, sizeof(s_black));
    memcpy(s_white, white, sizeof(s_white));
    s_have_black = true;
    s_have_white = true;
    apply_calibration_if_ready();
}

void knx_track_snapshot(knx_track_state_t *out)
{
    if (out == NULL) return;
    taskENTER_CRITICAL();
    *out = s_track;
    taskEXIT_CRITICAL();
}
