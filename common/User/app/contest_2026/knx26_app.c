#include "knx26_app.h"
#include "knx26_config.h"
#include "knx26_user.h"
#include "knx_blackbox.h"
#include "knx_board.h"
#include "knx_health.h"
#include "knx_key.h"
#include "knx_beep.h"
#include "knx_led.h"
#include "knx_param.h"
#include "knx_safety.h"
#include "knx_sys.h"
#include "knx_telemetry.h"
#include "knx_time.h"
#include "octolinker.h"
#include <string.h>

static volatile bool s_ready;
static knx26_context_t s_context;
static uint32_t s_can_heartbeat_tick;
static uint32_t s_link_heartbeat_tick;
static uint32_t s_led_tick;

static void command_message_handler(knx_comm_channel_t channel,
                                    const knx_comm_message_t *message,
                                    void *user)
{
    (void)channel;
    (void)user;
    if (message == NULL || message->length != 9U) return;
    float linear_mps;
    float angular_radps;
    memcpy(&linear_mps, &message->payload[0], sizeof(float));
    memcpy(&angular_radps, &message->payload[4], sizeof(float));
    if (message->payload[8] == 0U) {
        (void)knx_chassis_disable();
    } else {
        (void)knx_chassis_enable();
        (void)knx_chassis_set_velocity(linear_mps, angular_radps);
    }
}

static void peer_message_handler(knx_comm_channel_t channel,
                                 const knx_comm_message_t *message,
                                 void *user)
{
    (void)channel;
    (void)user;
    knx26_user_on_peer_message(message);
}

static void attach_can_buses(void)
{
    knx_can_bus_init();
    knx_can_t *can1 = knx_board_get_can_bus(0U);
    knx_can_t *can2 = knx_board_get_can_bus(1U);
    if (can1 != NULL && knx_can_bus_attach(KNX_CAN_BUS_1, can1) == KNX_OK) {
        (void)knx_can_bus_start(KNX_CAN_BUS_1);
    }
    if (can2 != NULL && knx_can_bus_attach(KNX_CAN_BUS_2, can2) == KNX_OK) {
        (void)knx_can_bus_start(KNX_CAN_BUS_2);
    }
}

knx_status_t knx26_app_init(void)
{
    memset(&s_context, 0, sizeof(s_context));
    s_ready = false;

    knx_health_init();
    knx_blackbox_init();
    knx_param_init();
    (void)knx_sys_init();
    if (knx_chassis_init() != KNX_OK) return KNX_ERROR;
    if (knx_imu_init() != KNX_OK) return KNX_ERROR;
    knx_track_init();
    knx_intersection_init();
    attach_can_buses();

    knx_comm_init(KNX26_LOCAL_NODE_ID);
#if defined(KNX_PLATFORM_STM32)
    (void)knx_comm_attach(KNX_COMM_HOST, knx_board_get_host_comm());
#endif
    (void)knx_comm_subscribe(KNX_COMM_HOST, KNX_COMM_MSG_COMMAND,
                             command_message_handler, NULL);
    (void)knx_comm_subscribe(KNX_COMM_BOARD, KNX_COMM_MSG_COMMAND,
                             command_message_handler, NULL);
    (void)knx_comm_subscribe(KNX_COMM_PEER, KNX_COMM_MSG_USER,
                             peer_message_handler, NULL);

    knx_telemetry_set_octo(knx_board_get_octolinker());
    (void)knx_telemetry_init();
    (void)knx_safety_init();
    knx26_user_init();
    s_can_heartbeat_tick = knx_millis();
    s_link_heartbeat_tick = s_can_heartbeat_tick;
    s_led_tick = s_can_heartbeat_tick;
    s_ready = true;
    return KNX_OK;
}

bool knx26_app_is_ready(void)
{
    return s_ready;
}

void knx26_fast_update(void)
{
    if (!s_ready) return;
    (void)knx_safety_update();
    if (knx_safety_get_level() == KNX_SAFETY_LEVEL_FAULT) {
        (void)knx_chassis_stop();
    } else {
        (void)knx_chassis_update_fast();
    }
    knx_beep_update();
}

void knx26_control_update(float dt_s)
{
    if (!s_ready) return;
    (void)knx_imu_update();
    if (knx_safety_get_level() != KNX_SAFETY_LEVEL_FAULT) {
        (void)knx_chassis_update_control(dt_s);
    }
}

void knx26_track_update(void)
{
    if (!s_ready) return;
    (void)knx_track_update();
    knx_track_state_t track;
    knx_track_snapshot(&track);
    (void)knx_intersection_update(&track);
    knx_intersection_result_t result;
    knx_intersection_snapshot(&result);
    if (result.fresh) {
        knx26_user_on_intersection(&result);
        knx_intersection_clear_fresh();
    }
}

static void send_can_heartbeats(uint32_t now)
{
    if (now - s_can_heartbeat_tick < KNX26_CAN_HEARTBEAT_MS) return;
    s_can_heartbeat_tick = now;
    uint8_t payload[8] = {
        'K', 'N', 'X', '2', KNX26_LOCAL_NODE_ID,
        (uint8_t)(now & 0xFFU), (uint8_t)((now >> 8) & 0xFFU), 0U
    };
    (void)knx_can_bus_send(KNX_CAN_BUS_1, 0x601U, payload, sizeof(payload));
    (void)knx_can_bus_send(KNX_CAN_BUS_2, 0x602U, payload, sizeof(payload));
}

static void send_link_heartbeats(uint32_t now)
{
    if (now - s_link_heartbeat_tick < KNX26_LINK_HEARTBEAT_MS) return;
    s_link_heartbeat_tick = now;
    uint8_t heartbeat[4] = {
        KNX26_LOCAL_NODE_ID,
        (uint8_t)s_ready,
        (uint8_t)knx_safety_get_level(),
        0U,
    };
    (void)knx_comm_send(KNX_COMM_HOST, KNX_COMM_MSG_HEARTBEAT,
                        0xFFU, heartbeat, sizeof(heartbeat));
    (void)knx_comm_send(KNX_COMM_BOARD, KNX_COMM_MSG_HEARTBEAT,
                        0xFFU, heartbeat, sizeof(heartbeat));
    (void)knx_comm_send(KNX_COMM_PEER, KNX_COMM_MSG_HEARTBEAT,
                        0xFFU, heartbeat, sizeof(heartbeat));
}

void knx26_comm_update(void)
{
    if (!s_ready) return;
    uint32_t now = knx_millis();
#if defined(KNX_PLATFORM_STM32)
    uint8_t bytes[64];
    for (uint8_t round = 0U; round < 4U; ++round) {
        uint16_t count = knx_board_host_comm_read(bytes, sizeof(bytes));
        if (count == 0U) break;
        (void)knx_host_comm_feed(knx_board_get_host_comm(), bytes, count);
    }
#else
    extern void knx_can_mspm0_poll(void);
    knx_can_mspm0_poll();
#endif
    knx_comm_update(now);
    send_can_heartbeats(now);
    send_link_heartbeats(now);
}

void knx26_user_app_update(void)
{
    if (!s_ready) return;
    knx_key_update(KNX26_APP_PERIOD_MS);
    if (knx_key_just_pressed(KNX_KEY_1)) {
        (void)knx_chassis_disable();
        knx_beep_beep(300U);
    }
    (void)knx_sys_update();
    knx26_context_snapshot(&s_context);
    knx26_user_update(&s_context);

    uint32_t now = knx_millis();
    if (now - s_led_tick >= 500U) {
        s_led_tick = now;
        knx_led_toggle(KNX_LED_0);
    }
}

void knx26_context_snapshot(knx26_context_t *out)
{
    if (out == NULL) return;
    knx26_context_t snapshot = {0};
    snapshot.now_ms = knx_millis();
    snapshot.ready = s_ready;
    knx_chassis_snapshot(&snapshot.chassis);
    knx_imu_snapshot(&snapshot.imu);
    knx_track_snapshot(&snapshot.track);
    knx_intersection_snapshot(&snapshot.intersection);
    for (uint8_t i = 0U; i < KNX_CAN_BUS_COUNT; ++i)
        knx_can_bus_snapshot((knx_can_bus_id_t)i, &snapshot.can[i]);
    for (uint8_t i = 0U; i < KNX_COMM_CHANNEL_COUNT; ++i)
        knx_comm_snapshot((knx_comm_channel_t)i, &snapshot.links[i]);
    *out = snapshot;
}

void knx26_debug_update(void)
{
    if (!s_ready) return;
    knx26_context_snapshot(&s_context);
    Octolinker_Instance_t *octo = knx_board_get_octolinker();
    if (octo == NULL) return;
    uint16_t b = KNX26_OCTO_BASE;
    (void)Octolinker_SendU32(octo, b + 0U, s_context.now_ms);
    (void)Octolinker_SendU8(octo, b + 1U, s_context.chassis.enabled);
    (void)Octolinker_SendF32(octo, b + 2U, s_context.chassis.drive.measured_linear_mps);
    (void)Octolinker_SendF32(octo, b + 3U, s_context.chassis.drive.measured_angular_radps);
    (void)Octolinker_SendF32(octo, b + 4U, s_context.chassis.left_motor.current_speed);
    (void)Octolinker_SendF32(octo, b + 5U, s_context.chassis.right_motor.current_speed);
    (void)Octolinker_SendF32(octo, b + 6U, s_context.imu.yaw_total);
    (void)Octolinker_SendF32(octo, b + 7U, s_context.imu.temperature);
    (void)Octolinker_SendF32(octo, b + 8U, s_context.track.sensor.line_error);
    (void)Octolinker_SendU8(octo, b + 9U, s_context.track.line_lost);
    (void)Octolinker_SendU8(octo, b + 10U, (uint8_t)s_context.intersection.type);
    (void)Octolinker_SendF32(octo, b + 11U, s_context.intersection.confidence);
    (void)Octolinker_SendU32(octo, b + 12U, s_context.can[0].tx_ok);
    (void)Octolinker_SendU32(octo, b + 13U, s_context.can[0].tx_error);
    (void)Octolinker_SendU32(octo, b + 14U, s_context.can[1].tx_ok);
    (void)Octolinker_SendU32(octo, b + 15U, s_context.can[1].tx_error);
    (void)Octolinker_SendU8(octo, b + 16U, (uint8_t)knx_safety_get_level());
}

knx_status_t knx26_attach_board_link(knx_host_comm_t *transport)
{
    return knx_comm_attach(KNX_COMM_BOARD, transport);
}

knx_status_t knx26_attach_peer_link(knx_host_comm_t *transport)
{
    return knx_comm_attach(KNX_COMM_PEER, transport);
}
