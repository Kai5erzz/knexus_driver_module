#include "test_color_track.h"
#include "knx_gimbal_ctrl.h"
#include "knx_time.h"
#include "knx_vision.h"
#include "octolinker.h"

#define TEST_COLOR_TRACK_TELEM_PERIOD_MS 50U
#define TEST_COLOR_TRACK_GIMBAL_BASE     280U
#define TEST_COLOR_TRACK_VISION_BASE     320U

static Octolinker_Instance_t *s_octo;
static uint32_t s_telem_tick;

void test_color_track_init(void *octo)
{
    s_octo = (Octolinker_Instance_t *)octo;
    s_telem_tick = 0U;
    knx_gimbal_ctrl_set_enable(1U);
    knx_gimbal_ctrl_restart();
}

void test_color_track_loop(void)
{
    uint32_t now = knx_millis();
    if (now - s_telem_tick < TEST_COLOR_TRACK_TELEM_PERIOD_MS) {
        return;
    }

    s_telem_tick = now;
    knx_gimbal_ctrl_debug_octo(s_octo, TEST_COLOR_TRACK_GIMBAL_BASE);
    knx_vision_debug_octo(s_octo, TEST_COLOR_TRACK_VISION_BASE);
}
