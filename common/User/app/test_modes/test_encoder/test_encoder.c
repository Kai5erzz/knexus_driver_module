/* test_encoder.c — Encoder test mode
 *
 * Platform-agnostic: uses only common interfaces.
 * Reads both encoders via Encoder_t driver, computes speed, outputs via OctoLink.
 */

#include "test_encoder.h"
#include "encoder.h"
#include "knx_encoder.h"
#include "knx_time.h"
#include "octolinker.h"
#include "knx_board.h"

static Octolinker_Instance_t *s_octo;

/* OctoLink variable IDs */
#define VAR_L_SPEED_RPM   0
#define VAR_L_SPEED_MPS   1
#define VAR_L_COUNT        2
#define VAR_R_SPEED_RPM   3
#define VAR_R_SPEED_MPS   4
#define VAR_R_COUNT        5

static uint32_t s_last_tx_tick;
#define TX_INTERVAL_MS    20U       /* 50 Hz OctoLink output */

/* Cumulative position (Encoder_CalcSpeed resets total_count every cycle) */
static int32_t s_l_position;
static int32_t s_r_position;
static uint16_t s_l_last_raw;
static uint16_t s_r_last_raw;

void test_encoder_init(void *octo)
{
    s_octo = (Octolinker_Instance_t *)octo;

    Encoder_AttachPorts(knx_board_get_encoder_left(),
                        knx_board_get_encoder_right());
    Encoder_Init();

    s_l_position  = 0;
    s_r_position  = 0;
    s_l_last_raw  = encoder_left.last_raw;
    s_r_last_raw  = encoder_right.last_raw;
    s_last_tx_tick = knx_millis();
}

/* Accumulate position from raw encoder delta (uint16 wrapping) */
static void update_position(void)
{
    uint16_t raw_l = 0, raw_r = 0;
    knx_encoder_read_raw(encoder_left.port,  &raw_l);
    knx_encoder_read_raw(encoder_right.port, &raw_r);

    int16_t diff_l = (int16_t)(raw_l - s_l_last_raw);
    int16_t diff_r = (int16_t)(raw_r - s_r_last_raw);

    s_l_position += (int32_t)diff_l * (int32_t)encoder_left.dir_sign;
    s_r_position += (int32_t)diff_r * (int32_t)encoder_right.dir_sign;

    s_l_last_raw = raw_l;
    s_r_last_raw = raw_r;
}

void test_encoder_loop(void)
{
    /* Update cumulative position (independent of speed calc) */
    update_position();

    /* Update speed (resets internal total_count) */
    Encoder_CalcSpeed(&encoder_left);
    Encoder_CalcSpeed(&encoder_right);

    /* OctoLink output at limited rate */
    uint32_t now = knx_millis();
    if (s_octo != NULL && (now - s_last_tx_tick) >= TX_INTERVAL_MS) {
        s_last_tx_tick = now;

        Octolinker_SendF32(s_octo, VAR_L_SPEED_RPM, encoder_left.speed_rpm);
        Octolinker_SendF32(s_octo, VAR_L_SPEED_MPS, encoder_left.speed_mps);
        Octolinker_SendI32(s_octo, VAR_L_COUNT,      s_l_position);

        Octolinker_SendF32(s_octo, VAR_R_SPEED_RPM, encoder_right.speed_rpm);
        Octolinker_SendF32(s_octo, VAR_R_SPEED_MPS, encoder_right.speed_mps);
        Octolinker_SendI32(s_octo, VAR_R_COUNT,      s_r_position);
    }
}
