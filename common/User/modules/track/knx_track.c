#include "knx_track.h"
#include "knx_time.h"
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
    s_track.last_status = knx_grayscale_update();
    knx_grayscale_snapshot(&s_track.sensor);
    s_track.line_strength = 0.0f;
    for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
        s_track.line_strength += s_track.sensor.normalized[i];
    }
    s_track.line_lost = (s_track.line_strength < 0.15f);
    s_track.timestamp_ms = knx_millis();
    s_track.update_count++;
    return s_track.last_status;
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
    memcpy(s_black, s_track.sensor.raw, sizeof(s_black));
    s_have_black = true;
    apply_calibration_if_ready();
    return KNX_OK;
}

knx_status_t knx_track_capture_white(void)
{
    (void)knx_track_update();
    memcpy(s_white, s_track.sensor.raw, sizeof(s_white));
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
    if (out != NULL) *out = s_track;
}
