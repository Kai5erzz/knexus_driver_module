#include "knx_dji_motor_ctrl.h"
#include "dji_motor.h"
#include "knexus_config.h"
#include "knx_can_bus.h"
#include "knx_time.h"
#include "PID.h"

static pid_obj_t *s_speed_pid;
static dji_motor_object_t *s_motor;
static float s_target_rpm;
static float s_target_used_rpm;
static bool s_initialized;
static bool s_enabled;
static knx_status_t s_last_tx_status = KNX_NOT_READY;
static uint32_t s_last_tx_attempt_ms;
static uint32_t s_stop_flush_deadline_ms;
static float s_max_current_cmd = KNEXUS_DJI_PID_MAX_CURRENT_CMD;
static float s_current_feedforward_cmd;
static float s_current_feedforward_min_rpm;
static float s_target_slew_rpmps = KNEXUS_DJI_TARGET_SLEW_RPM_PER_S;

volatile int32_t dji_init_status = KNX_NOT_READY;
volatile uint32_t dji_tx_count;
volatile uint32_t dji_tx_error_count;
volatile uint32_t dji_tx_busy_count;
volatile uint32_t dji_rx_count;
volatile int32_t dji_pid_output;

static float limit_abs(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static int16_t m3508_speed_control(const dji_motor_measure_t *measure)
{
    if (measure == NULL || s_speed_pid == NULL) return 0;
    float output = pid_calculate(s_speed_pid, measure->speed_rpm,
                                 s_target_used_rpm);
    if (s_current_feedforward_cmd > 0.0f &&
        s_target_used_rpm >= s_current_feedforward_min_rpm) {
        output += s_current_feedforward_cmd;
    } else if (s_current_feedforward_cmd > 0.0f &&
               s_target_used_rpm <= -s_current_feedforward_min_rpm) {
        output -= s_current_feedforward_cmd;
    }
    output = limit_abs(output, s_max_current_cmd);
    dji_pid_output = (int32_t)output;
    return (int16_t)output;
}

static void dji_feedback_handler(uint32_t std_id, const uint8_t *data,
                                 uint8_t len, void *user)
{
    dji_motor_rx_handler(std_id, data, len, user);
    dji_rx_count++;
}

knx_status_t knx_dji_motor_ctrl_init(void)
{
    if (s_initialized) return KNX_OK;

    dji_motor_init();
    pid_config_t pid_config = {
        .Kp = KNEXUS_DJI_SPEED_KP,
        .Ki = KNEXUS_DJI_SPEED_KI,
        .Kd = KNEXUS_DJI_SPEED_KD,
        .MaxOut = KNEXUS_DJI_PID_MAX_CURRENT_CMD,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,
        .IntegralLimit = KNEXUS_DJI_PID_INTEGRAL_LIMIT,
        .CoefA = 0.0f,
        .CoefB = 0.0f,
        .Output_LPF_RC = 0.0f,
        .Derivative_LPF_RC = 0.0f,
    };
    s_speed_pid = pid_register(&pid_config);
    if (s_speed_pid == NULL) {
        dji_init_status = KNX_ERROR;
        return KNX_ERROR;
    }

    dji_motor_config_t motor_config = {
        .motor_type = DJI_M3508,
        .can_bus_index = KNEXUS_DJI_CAN_BUS_INDEX,
        .tx_id = KNEXUS_DJI_MOTOR_TX_ID,
        .rx_id = KNEXUS_DJI_MOTOR_RX_ID,
    };
    s_motor = dji_motor_register(&motor_config, m3508_speed_control);
    if (s_motor == NULL) {
        dji_init_status = KNX_ERROR;
        return KNX_ERROR;
    }

    knx_status_t status = knx_can_bus_subscribe(
        (knx_can_bus_id_t)KNEXUS_DJI_CAN_BUS_INDEX,
        KNEXUS_DJI_MOTOR_RX_ID, KNEXUS_DJI_MOTOR_RX_ID,
        dji_feedback_handler, NULL);
    if (status != KNX_OK) {
        dji_init_status = status;
        return status;
    }

    s_target_rpm = 0.0f;
    s_target_used_rpm = 0.0f;
    s_max_current_cmd = KNEXUS_DJI_PID_MAX_CURRENT_CMD;
    s_current_feedforward_cmd = 0.0f;
    s_current_feedforward_min_rpm = 0.0f;
    s_target_slew_rpmps = KNEXUS_DJI_TARGET_SLEW_RPM_PER_S;
    s_last_tx_attempt_ms = knx_millis();
    s_enabled = false;
    s_initialized = true;
    dji_init_status = KNX_OK;
    dji_tx_count = 0U;
    dji_tx_error_count = 0U;
    dji_tx_busy_count = 0U;
    dji_rx_count = 0U;
    dji_pid_output = 0;
    dji_motor_relax(s_motor);
    return KNX_OK;
}

void knx_dji_motor_ctrl_update(float dt_s)
{
    if (!s_initialized || s_motor == NULL) return;
    uint32_t now = knx_millis();
    if (!s_enabled) {
        /* C620保持最后一条电流命令。stop()的首次零帧若遇到TX FIFO busy，
         * 必须由1 kHz任务继续重试，否则会出现软件已停而电机继续加速。 */
        bool flush_active = (int32_t)(s_stop_flush_deadline_ms - now) > 0;
        if (flush_active &&
            (now - s_last_tx_attempt_ms) >=
                KNEXUS_DJI_STOP_FLUSH_PERIOD_MS) {
            s_last_tx_attempt_ms = now;
            s_last_tx_status = dji_motor_control();
            if (s_last_tx_status == KNX_OK) dji_tx_count++;
            else {
                dji_tx_error_count++;
                if (s_last_tx_status == KNX_BUSY ||
                    s_last_tx_status == KNX_TIMEOUT) {
                    dji_tx_busy_count++;
                }
            }
        }
        return;
    }
    if (dt_s <= 0.0f) dt_s = 0.001f;

    float max_step = s_target_slew_rpmps * dt_s;
    float delta = s_target_rpm - s_target_used_rpm;
    if (delta > max_step) delta = max_step;
    if (delta < -max_step) delta = -max_step;
    s_target_used_rpm += delta;

    /*
     * A powered C620 publishes 0x201 feedback without requiring a command.
     * If that feedback is absent/stale, keep only a low-rate probe stream.
     * This makes an intentionally unplugged 3508 a normal degraded state,
     * rather than allowing repeated no-ACK traffic to dominate FDCAN.
     */
    bool feedback_fresh =
        (s_motor->measure.feedback_count != 0U) &&
        ((now - s_motor->measure.last_feedback_ms) <=
         KNEXUS_DJI_FEEDBACK_TIMEOUT_MS);
    if (!feedback_fresh &&
        (now - s_last_tx_attempt_ms) < KNEXUS_DJI_OFFLINE_TX_PERIOD_MS) {
        return;
    }
    s_last_tx_attempt_ms = now;

    s_last_tx_status = dji_motor_control();
    if (s_last_tx_status == KNX_OK) dji_tx_count++;
    else {
        dji_tx_error_count++;
        if (s_last_tx_status == KNX_BUSY ||
            s_last_tx_status == KNX_TIMEOUT) {
            dji_tx_busy_count++;
        }
    }
}

void knx_dji_motor_ctrl_set_target(float target_rpm)
{
    s_target_rpm = limit_abs(target_rpm, KNEXUS_DJI_MAX_TARGET_RPM);
}

knx_status_t knx_dji_motor_ctrl_set_speed_pid(float kp, float ki, float kd,
                                              float integral_limit,
                                              float max_current_cmd)
{
    if (!s_initialized || s_speed_pid == NULL ||
        kp < 0.0f || ki < 0.0f || kd < 0.0f ||
        integral_limit < 0.0f || max_current_cmd <= 0.0f ||
        max_current_cmd > 16384.0f) {
        return KNX_INVALID_ARG;
    }

    s_speed_pid->Kp = kp;
    s_speed_pid->Ki = ki;
    s_speed_pid->Kd = kd;
    s_speed_pid->IntegralLimit = integral_limit;
    s_speed_pid->MaxOut = max_current_cmd;
    s_max_current_cmd = max_current_cmd;
    pid_clear(s_speed_pid);
    return KNX_OK;
}

knx_status_t knx_dji_motor_ctrl_set_current_feedforward(
    float current_cmd, float min_target_rpm)
{
    if (!s_initialized || current_cmd < 0.0f ||
        current_cmd > s_max_current_cmd || min_target_rpm < 0.0f) {
        return KNX_INVALID_ARG;
    }
    s_current_feedforward_cmd = current_cmd;
    s_current_feedforward_min_rpm = min_target_rpm;
    return KNX_OK;
}

knx_status_t knx_dji_motor_ctrl_set_target_slew(float slew_rpmps)
{
    if (!s_initialized || slew_rpmps <= 0.0f) {
        return KNX_INVALID_ARG;
    }
    s_target_slew_rpmps = slew_rpmps;
    return KNX_OK;
}

void knx_dji_motor_ctrl_stop(void)
{
    s_target_rpm = 0.0f;
    s_target_used_rpm = 0.0f;
    s_enabled = false;
    s_last_tx_attempt_ms = knx_millis();
    s_stop_flush_deadline_ms =
        s_last_tx_attempt_ms + KNEXUS_DJI_STOP_FLUSH_MS;
    dji_pid_output = 0;
    if (s_speed_pid != NULL) pid_clear(s_speed_pid);
    dji_motor_relax(s_motor);
    if (s_initialized && s_motor != NULL) {
        s_last_tx_status = dji_motor_control();
        if (s_last_tx_status == KNX_OK) dji_tx_count++;
        else {
            dji_tx_error_count++;
            if (s_last_tx_status == KNX_BUSY ||
                s_last_tx_status == KNX_TIMEOUT) {
                dji_tx_busy_count++;
            }
        }
    }
}

void knx_dji_motor_ctrl_enable(void)
{
    if (!s_initialized || s_motor == NULL) return;
    s_last_tx_attempt_ms = knx_millis();
    s_stop_flush_deadline_ms = 0U;
    s_enabled = true;
    dji_motor_enable(s_motor);
}

void knx_dji_motor_ctrl_snapshot(knx_dji_motor_state_t *out)
{
    if (out == NULL) return;
    *out = (knx_dji_motor_state_t){0};
    out->initialized = s_initialized;
    out->enabled = s_enabled;
    out->target_rpm = s_target_rpm;
    out->target_used_rpm = s_target_used_rpm;
    out->target_slew_rpmps = s_target_slew_rpmps;
    out->pid_output = (int16_t)dji_pid_output;
    out->tx_ok = dji_tx_count;
    out->tx_error = dji_tx_error_count;
    out->tx_busy = dji_tx_busy_count;
    out->rx_count = dji_rx_count;
    out->init_status = (knx_status_t)dji_init_status;
    out->last_tx_status = s_last_tx_status;
    if (s_motor != NULL) {
        out->speed_rpm = s_motor->measure.speed_rpm;
        out->current = s_motor->measure.current;
        out->angle_deg = s_motor->measure.total_angle_deg;
        out->temperature_c = s_motor->measure.temperature_c;
        out->feedback_age_ms = (s_motor->measure.feedback_count == 0U)
                                   ? 0xFFFFFFFFU
                                   : knx_millis() - s_motor->measure.last_feedback_ms;
    } else {
        out->feedback_age_ms = 0xFFFFFFFFU;
    }
}
