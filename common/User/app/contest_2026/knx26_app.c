#include "knx26_app.h"
#include "knx26_config.h"
#include "knx26_user.h"
#include "knx_blackbox.h"
#include "knx_board.h"
#include "knx_health.h"
#include "knx_imu.h"
#include "knx_key.h"
#include "knx_beep.h"
#include "knx_led.h"
#include "knx_param.h"
#include "knx_port.h"
#include "knx_safety.h"
#include "knx_sys.h"
#include "knx_telemetry.h"
#include "knx_time.h"
#include "octolinker.h"
#include "track_sensor.h"
#include "bmi088.h"
#if defined(KNEXUS_MODE_BOARD_TEST) || defined(KNEXUS_MODE_SCREW_TEST)
#include "knx_dji_motor_ctrl.h"
#endif
#if defined(KNX_PLATFORM_MSPM0)
#include "ti_msp_dl_config.h"
#include <ti/driverlib/driverlib.h>
#endif
#include <string.h>
#include <math.h>

static volatile bool s_ready;
static knx26_context_t s_context;
static uint32_t s_can_heartbeat_tick;
static uint32_t s_link_heartbeat_tick;
static uint32_t s_led_tick;
static uint32_t s_perception_track_update;
#if KNX26_OCTO_DEBUG_EN
static uint8_t s_debug_diag_divider;
#endif
volatile uint32_t knx26_diag_init_stage;
volatile uint32_t knx26_diag_init_error;

/* STM32 与 MSPM0 调度器使用同名诊断量，便于统一由 OctoLink 验证。 */
extern volatile uint32_t knx26_diag_app_heartbeat;
extern volatile uint32_t knx26_diag_fast_heartbeat;
extern volatile uint32_t knx26_diag_control_heartbeat;
extern volatile uint32_t knx26_diag_track_heartbeat;
extern volatile uint32_t knx26_diag_perception_heartbeat;
extern volatile uint32_t knx26_diag_comm_heartbeat;
extern volatile uint32_t knx26_diag_debug_heartbeat;
extern volatile uint32_t knx26_diag_app_overrun;
extern volatile uint32_t knx26_diag_fast_overrun;
extern volatile uint32_t knx26_diag_control_overrun;
extern volatile uint32_t knx26_diag_track_overrun;
extern volatile uint32_t knx26_diag_perception_overrun;
extern volatile uint32_t knx26_diag_comm_overrun;
extern volatile uint32_t knx26_diag_debug_overrun;
extern volatile uint32_t knx26_diag_app_max_elapsed_ms;
extern volatile uint32_t knx26_diag_fast_max_elapsed_ms;
extern volatile uint32_t knx26_diag_control_max_elapsed_ms;
extern volatile uint32_t knx26_diag_track_max_elapsed_ms;
extern volatile uint32_t knx26_diag_perception_max_elapsed_ms;
extern volatile uint32_t knx26_diag_comm_max_elapsed_ms;
extern volatile uint32_t knx26_diag_debug_max_elapsed_ms;

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
    knx26_diag_init_stage = 1U;
    knx26_diag_init_error = 0U;
    memset(&s_context, 0, sizeof(s_context));
    s_ready = false;

    knx_health_init();
    knx_blackbox_init();
    knx_param_init();
    (void)knx_sys_init();
    knx26_diag_init_stage = 2U;
    if (knx_chassis_init() != KNX_OK) {
        knx26_diag_init_error = 2U;
        return KNX_ERROR;
    }
    knx26_diag_init_stage = 3U;
    if (knx_imu_init() != KNX_OK) {
        knx26_diag_init_error = 3U;
        return KNX_ERROR;
    }
    knx26_diag_init_stage = 4U;
    knx_track_init();
#if KNX26_INTERSECTION_EN
    knx_intersection_init();
#endif
    attach_can_buses();
    knx26_diag_init_stage = 5U;

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
    knx26_diag_init_stage = 6U;

    knx_telemetry_set_octo(knx_board_get_octolinker());
    (void)knx_telemetry_init();
    (void)knx_safety_init();
    knx26_diag_init_stage = 7U;
    knx26_user_init();
    knx26_diag_init_stage = 8U;
    s_can_heartbeat_tick = knx_millis();
    s_link_heartbeat_tick = s_can_heartbeat_tick;
    s_led_tick = s_can_heartbeat_tick;
    s_perception_track_update = 0U;
    s_ready = true;
    knx26_diag_init_stage = 9U;
    return KNX_OK;
}

bool knx26_app_is_ready(void)
{
    return s_ready;
}

void knx26_fast_update(void)
{
    if (!s_ready) return;
#if KNX26_MOTOR_PIN_TEST
    /* Bench motor mode: once KEY0 sets the hardware PWM registers, no
     * attitude, encoder, chassis or safety state is allowed to clear them. */
    knx_beep_update();
    return;
#elif defined(KNEXUS_MODE_BOARD_TEST)
    /* 验收时底板可能被翻转测量。只在该模式绕过姿态安全停车；电机仍需
     * KEY0 长按解锁，并受30%占空比硬限制和KEY1急停约束。 */
    (void)knx_chassis_update_fast();
    knx_dji_motor_ctrl_update((float)KNX26_FAST_PERIOD_MS * 0.001f);
    /* 正常模式由 knx_safety_update() 喂狗；验收模式绕过安全判定后必须
     * 在这里显式喂狗，否则STM32会在IWDG超时后周期性复位。 */
    knx_port_watchdog_refresh();
    knx_beep_update();
    return;
#elif defined(KNEXUS_MODE_SCREW_TEST)
    /* 丝杆测试只驱动FDCAN2上的C620。按键状态机在10 ms任务中给目标，
     * 1 kHz任务负责速度斜坡、PID和CAN电流帧；不启动底盘轮电机。 */
    knx_dji_motor_ctrl_update((float)KNX26_FAST_PERIOD_MS * 0.001f);
    knx_port_watchdog_refresh();
    knx_beep_update();
    return;
#else
    (void)knx_safety_update();
    if (knx_safety_get_level() == KNX_SAFETY_LEVEL_FAULT) {
        (void)knx_chassis_stop();
    } else {
        (void)knx_chassis_update_fast();
    }
    knx_beep_update();
#endif
}

void knx26_control_update(float dt_s)
{
    if (!s_ready) return;
#if KNX26_MOTOR_PIN_TEST
    (void)dt_s;
    return;
#else
    knx_chassis_state_t motion_state;
    knx_chassis_snapshot(&motion_state);
    bool vehicle_stationary = !motion_state.enabled ||
        (fabsf(motion_state.drive.left_target_mps) < 0.01f &&
         fabsf(motion_state.drive.right_target_mps) < 0.01f &&
         fabsf(motion_state.drive.measured_linear_mps) < 0.02f &&
         fabsf(motion_state.drive.measured_angular_radps) < 0.03f &&
         fabsf(motion_state.left_motor.duty) < 0.03f &&
         fabsf(motion_state.right_motor.duty) < 0.03f);
    knx_imu_set_stationary_hint(vehicle_stationary);
    (void)knx_imu_update();
#if defined(KNEXUS_MODE_BOARD_TEST)
    (void)knx_chassis_update_control(dt_s);
#else
    if (knx_safety_get_level() != KNX_SAFETY_LEVEL_FAULT) {
        (void)knx_chassis_update_control(dt_s);
    }
#endif
#endif
}

void knx26_track_update(void)
{
    if (!s_ready) return;
    (void)knx_track_update();
}

void knx26_perception_update(void)
{
#if !KNX26_INTERSECTION_EN
    return;
#else
    if (!s_ready) return;
    knx_track_state_t track;
    knx_track_snapshot(&track);
    if (track.update_count == 0U ||
        track.update_count == s_perception_track_update) {
        return;
    }
    s_perception_track_update = track.update_count;
    (void)knx_intersection_update(&track);
    knx_intersection_result_t result;
    knx_intersection_snapshot(&result);
    if (result.fresh) {
        knx26_user_on_intersection(&result);
        knx_intersection_clear_fresh();
    }
#endif
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
    snapshot.ready = s_ready;
    knx_chassis_snapshot(&snapshot.chassis);
    knx_imu_snapshot(&snapshot.imu);
    knx_track_snapshot(&snapshot.track);
    knx_intersection_snapshot(&snapshot.intersection);
    for (uint8_t i = 0U; i < KNX_CAN_BUS_COUNT; ++i)
        knx_can_bus_snapshot((knx_can_bus_id_t)i, &snapshot.can[i]);
    for (uint8_t i = 0U; i < KNX_COMM_CHANNEL_COUNT; ++i)
        knx_comm_snapshot((knx_comm_channel_t)i, &snapshot.links[i]);
    /* Capture time last.  A higher-priority track update may run while the
     * snapshot is being assembled; taking now_ms first could make the copied
     * track timestamp appear to be in the future and underflow its age. */
    snapshot.now_ms = knx_millis();
    *out = snapshot;
}

void knx26_debug_update(void)
{
#if !KNX26_OCTO_DEBUG_EN
    return;
#else
    if (!s_ready) return;
    /* The app task owns s_context.  Using it here as a second shared output
     * buffer can tear the app task's snapshot while it evaluates freshness. */
    knx26_context_t debug_context;
    knx26_context_snapshot(&debug_context);
    Octolinker_Instance_t *octo = knx_board_get_octolinker();
    if (octo == NULL) return;
#if KNEXUS_DEBUG_LINE_DATA_ENABLE
    for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
        (void)Octolinker_SendF32(octo, (uint16_t)(KNX26_OCTO_BASE + i),
                                 debug_context.track.sensor.normalized[i]);
    }
    (void)Octolinker_SendF32(octo, 725U,
                             debug_context.track.sensor.line_error);
    (void)Octolinker_SendF32(octo, 726U,
                             debug_context.track.line_strength);
    (void)Octolinker_SendF32(octo, 727U,
                             debug_context.chassis.drive.measured_linear_mps);
    (void)Octolinker_SendF32(octo, 728U,
                             debug_context.chassis.drive.measured_angular_radps);
    (void)Octolinker_SendU8(octo, 729U,
                            debug_context.track.sensor.is_calibrated);
#endif
    /* 每个模式都可使用50 Hz控制调试通道；不再依赖巡线调试宏。 */
    knx26_user_debug_control_octo(octo, 748U);

    if (++s_debug_diag_divider < 5U) return;
    s_debug_diag_divider = 0U;

    /* 10 Hz 调度诊断：心跳用于核对实际频率，overrun 必须长期保持不增长。 */
    (void)Octolinker_SendU32(octo, 780U, knx26_diag_app_heartbeat);
    (void)Octolinker_SendU32(octo, 781U, knx26_diag_fast_heartbeat);
    (void)Octolinker_SendU32(octo, 782U, knx26_diag_control_heartbeat);
    (void)Octolinker_SendU32(octo, 783U, knx26_diag_track_heartbeat);
    (void)Octolinker_SendU32(octo, 784U, knx26_diag_comm_heartbeat);
    (void)Octolinker_SendU32(octo, 785U, knx26_diag_debug_heartbeat);
    (void)Octolinker_SendU32(octo, 786U, knx26_diag_app_overrun);
    (void)Octolinker_SendU32(octo, 787U, knx26_diag_fast_overrun);
    (void)Octolinker_SendU32(octo, 788U, knx26_diag_control_overrun);
    (void)Octolinker_SendU32(octo, 789U, knx26_diag_track_overrun);
    (void)Octolinker_SendU32(octo, 790U, knx26_diag_comm_overrun);
    (void)Octolinker_SendU32(octo, 791U, knx26_diag_debug_overrun);
    (void)Octolinker_SendU32(octo, 792U, knx26_diag_app_max_elapsed_ms);
    (void)Octolinker_SendU32(octo, 793U, knx26_diag_fast_max_elapsed_ms);
    (void)Octolinker_SendU32(octo, 794U, knx26_diag_control_max_elapsed_ms);
    (void)Octolinker_SendU32(octo, 795U, knx26_diag_track_max_elapsed_ms);
    (void)Octolinker_SendU32(octo, 796U, knx26_diag_comm_max_elapsed_ms);
    (void)Octolinker_SendU32(octo, 797U, knx26_diag_debug_max_elapsed_ms);
    (void)Octolinker_SendU32(octo, 798U, knx26_diag_perception_heartbeat);
    (void)Octolinker_SendU32(octo, 799U, knx26_diag_perception_overrun);
    (void)Octolinker_SendU32(octo, 800U,
                             knx26_diag_perception_max_elapsed_ms);

#if KNEXUS_DEBUG_IMU_DATA_ENABLE
    BMI088_DebugOcto(octo);
    (void)Octolinker_SendF32(octo, 44U, debug_context.imu.yaw_total);
    (void)Octolinker_SendF32(octo, 45U, debug_context.imu.gyro[2]);
    (void)Octolinker_SendF32(octo, 46U,
                             debug_context.imu.gyro_runtime_bias[2]);
    (void)Octolinker_SendU8(octo, 47U,
                            (uint8_t)debug_context.imu.zero_drift_stationary);
    (void)Octolinker_SendU32(octo, 48U,
                             debug_context.imu.zero_drift_dwell_ms);
    (void)Octolinker_SendU8(octo, 49U,
                            (uint8_t)debug_context.imu.zero_drift_temp_ready);
    (void)Octolinker_SendF32(octo, 50U, imu_data.gyro[2]);
    (void)Octolinker_SendF32(octo, 51U, debug_context.imu.gyro_norm);
    (void)Octolinker_SendF32(octo, 52U, knx_imu_zero_drift_enable);
    (void)Octolinker_SendF32(octo, 53U, knx_imu_zero_drift_gyro_enter);
    (void)Octolinker_SendF32(octo, 54U, knx_imu_zero_drift_accel_enter);
    (void)Octolinker_SendF32(octo, 55U, knx_imu_zero_drift_bias_tau_s);
    (void)Octolinker_SendF32(octo, 56U,
                             knx_imu_zero_drift_default_bias_z);
    (void)Octolinker_SendU32(octo, 57U,
                             knx_imu_zero_drift_confirm_ms);
#endif

#if KNEXUS_DEBUG_LINE_DATA_ENABLE
    (void)Octolinker_SendU32(octo, 708U, debug_context.now_ms);
    (void)Octolinker_SendI32(octo, 709U, (int32_t)debug_context.track.last_status);
    (void)Octolinker_SendU32(octo, 710U,
                             (debug_context.now_ms >= debug_context.track.timestamp_ms)
                                 ? (debug_context.now_ms - debug_context.track.timestamp_ms)
                                 : 0U);
    (void)Octolinker_SendI32(octo, 711U, track_scan_last_status);
    (void)Octolinker_SendU32(octo, 712U, track_scan_last_completed_channel);
    (void)Octolinker_SendU32(octo, 713U, track_scan_ok_count);
    (void)Octolinker_SendU32(octo, 714U, track_scan_error_count);
#endif
    (void)Octolinker_SendU8(octo, 715U, (uint8_t)debug_context.chassis.enabled);
    (void)Octolinker_SendI32(octo, 716U,
                             (int32_t)debug_context.chassis.last_status);
    (void)Octolinker_SendU8(octo, 717U,
                            (uint8_t)knx_safety_get_level());
    (void)Octolinker_SendU32(octo, 718U, knx_safety_get_faults());
    (void)Octolinker_SendU32(octo, 719U, knx_safety_get_warnings());
#if defined(KNX_PLATFORM_MSPM0)
    extern volatile uint32_t knx_mspm0_reset_cause;
    extern volatile uint32_t knx26_diag_app_overrun;
    extern volatile uint32_t knx26_diag_fast_overrun;
    extern volatile uint32_t knx26_diag_control_overrun;
    extern volatile uint32_t knx26_diag_track_overrun;
    extern volatile uint32_t knx_pwm_mspm0_tima0_enabled_mask;
    (void)Octolinker_SendU32(octo, 720U, knx_mspm0_reset_cause);
    (void)Octolinker_SendU32(octo, 721U, knx26_diag_app_overrun);
    (void)Octolinker_SendU32(octo, 722U, knx26_diag_fast_overrun);
    (void)Octolinker_SendU32(octo, 723U, knx26_diag_control_overrun);
    (void)Octolinker_SendU32(octo, 724U, knx26_diag_track_overrun);
    /* Heater PWM proof: requested duty alone is insufficient when a motor
     * shares TIMA0.  Export the live CC2/timer/output registers so OctoLink
     * can prove that PB17 is actually being driven continuously. */
    (void)Octolinker_SendU32(octo, 37U,
        DL_TimerA_getCaptureCompareValue(PWM_HEATER_INST, DL_TIMER_CC_2_INDEX));
    (void)Octolinker_SendU8(octo, 38U,
        (uint8_t)DL_TimerA_isRunning(PWM_HEATER_INST));
    (void)Octolinker_SendU32(octo, 39U, PWM_HEATER_INST->COMMONREGS.ODIS);
    (void)Octolinker_SendU32(octo, 40U, PWM_HEATER_INST->COMMONREGS.CCPD);
    (void)Octolinker_SendU32(octo, 41U,
        knx_pwm_mspm0_tima0_enabled_mask);
    (void)Octolinker_SendU32(octo, 42U,
        DL_TimerA_getCaptureCompareValue(PWM_L_INST, DL_TIMER_CC_0_INDEX));
#endif
#if KNEXUS_DEBUG_PID_DATA_ENABLE
    (void)Octolinker_SendF32(octo, 752U,
                             debug_context.chassis.drive.left_target_mps);
    (void)Octolinker_SendF32(octo, 753U,
                             debug_context.chassis.drive.right_target_mps);
    (void)Octolinker_SendF32(octo, 754U,
                             debug_context.chassis.left_motor.current_speed);
    (void)Octolinker_SendF32(octo, 755U,
                             debug_context.chassis.right_motor.current_speed);
    (void)Octolinker_SendF32(octo, 756U,
                             debug_context.chassis.left_motor.duty);
    (void)Octolinker_SendF32(octo, 757U,
                             debug_context.chassis.right_motor.duty);
#endif
#if KNEXUS_DEBUG_LINE_DATA_ENABLE
    (void)Octolinker_SendU8(octo, 758U,
                            debug_context.track.sensor.digital_byte);
    (void)Octolinker_SendU8(octo, 759U,
                            (uint8_t)debug_context.track.line_lost);
    (void)Octolinker_SendF32(octo, 761U, knx26_line_ki);
    (void)Octolinker_SendF32(octo, 762U,
                             knx26_line_min_angular_radps);
    (void)Octolinker_SendF32(octo, 763U,
                             knx26_line_integral_limit);
    (void)Octolinker_SendF32(octo, 764U,
                             knx26_line_min_strength);
    (void)Octolinker_SendF32(octo, 765U,
                             knx26_line_recovery_angular_radps);

    for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
        (void)Octolinker_SendU16(octo, (uint16_t)(770U + i),
                                 debug_context.track.sensor.raw[i]);
    }
#endif
    knx26_user_debug_octo(octo, 730U);
#endif
}

knx_status_t knx26_attach_board_link(knx_host_comm_t *transport)
{
    return knx_comm_attach(KNX_COMM_BOARD, transport);
}

knx_status_t knx26_attach_peer_link(knx_host_comm_t *transport)
{
    return knx_comm_attach(KNX_COMM_PEER, transport);
}
