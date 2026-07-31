#include "knx_ball_motion_comp.h"
#include "knexus_config.h"
#include <math.h>

#define RAD_TO_DEG 57.2957795131f
#define DEG_TO_RAD 0.01745329252f

/* Runtime variables intentionally remain writable from GDB/OctoLink.  The
 * BMI088 forward mapping starts unknown; the encoder remains the safe
 * acceleration source until the online correlation learner identifies it. */
int32_t knexus_ball_accel_axis = KNEXUS_BALL_ACCEL_AXIS_DEFAULT;
float knexus_ball_accel_axis_sign =
    KNEXUS_BALL_ACCEL_AXIS_SIGN_DEFAULT;
float knexus_ball_roll_gravity_sign =
    KNEXUS_BALL_ROLL_GRAVITY_SIGN_DEFAULT;
float knexus_ball_compensation_enable =
    KNEXUS_BALL_COMPENSATION_ENABLE_DEFAULT;
float knexus_ball_accel_compensation_gain =
    KNEXUS_BALL_ACCEL_COMPENSATION_GAIN;
float knexus_ball_drag_s_inv = KNEXUS_BALL_DRAG_S_INV_DEFAULT;

static knx_ball_motion_comp_state_t s_state;
static float s_bias_sum[3];
static float s_mapping_cross[3];
static float s_mapping_axis_energy[3];
static float s_mapping_encoder_energy;
static float s_stationary_time_s;
static float s_motion_time_s;
static float s_last_target_roll_deg;
static bool s_filter_initialized;
static bool s_predicting_last;

static float clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void chassis_gravity_vector(float roll_deg, float pitch_deg,
                                   float gravity_mps2[3])
{
    const float roll = roll_deg * DEG_TO_RAD;
    const float pitch = pitch_deg * DEG_TO_RAD;
    gravity_mps2[0] = -KNEXUS_BALL_GRAVITY_MPS2 * sinf(pitch);
    gravity_mps2[1] = KNEXUS_BALL_GRAVITY_MPS2 * sinf(roll) * cosf(pitch);
    gravity_mps2[2] = KNEXUS_BALL_GRAVITY_MPS2 * cosf(roll) * cosf(pitch);
}

static void reset_mapping_learning(void)
{
    s_mapping_cross[0] = 0.0f;
    s_mapping_cross[1] = 0.0f;
    s_mapping_cross[2] = 0.0f;
    s_mapping_axis_energy[0] = 0.0f;
    s_mapping_axis_energy[1] = 0.0f;
    s_mapping_axis_energy[2] = 0.0f;
    s_mapping_encoder_energy = 0.0f;
    s_state.mapping_sample_count = 0U;
}

void knx_ball_motion_comp_init(void)
{
    s_state = (knx_ball_motion_comp_state_t){0};
    s_bias_sum[0] = 0.0f;
    s_bias_sum[1] = 0.0f;
    s_bias_sum[2] = 0.0f;
    reset_mapping_learning();
    s_stationary_time_s = 0.0f;
    s_motion_time_s = 0.0f;
    s_last_target_roll_deg = 0.0f;
    s_filter_initialized = false;
    s_predicting_last = false;
}

void knx_ball_motion_comp_update(const float chassis_accel_mps2[3],
                                 float chassis_roll_deg,
                                 float chassis_pitch_deg,
                                 bool chassis_imu_valid,
                                 float rod_roll_deg,
                                 float chassis_encoder_accel_mps2,
                                 bool chassis_encoder_valid,
                                 float chassis_command_accel_mps2,
                                 bool stationary_for_bias,
                                 bool chassis_stationary,
                                 bool compensation_requested, float dt_s)
{
    if (chassis_accel_mps2 == NULL) return;
    if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.01f;

    float gravity_mps2[3];
    chassis_gravity_vector(chassis_roll_deg, chassis_pitch_deg,
                           gravity_mps2);
    for (uint8_t i = 0U; i < 3U; ++i) {
        s_state.raw_accel_mps2[i] = chassis_accel_mps2[i];
        s_state.gravity_compensated_accel_mps2[i] =
            chassis_accel_mps2[i] - gravity_mps2[i];
    }
    s_state.chassis_imu_ready = chassis_imu_valid;
    s_state.chassis_encoder_accel_mps2 = chassis_encoder_accel_mps2;
    s_state.chassis_command_accel_mps2 = chassis_command_accel_mps2;

    bool axis_valid =
        knexus_ball_accel_axis >= KNX_BALL_ACCEL_AXIS_X &&
        knexus_ball_accel_axis <= KNX_BALL_ACCEL_AXIS_Z;
    bool roll_sign_valid = fabsf(knexus_ball_roll_gravity_sign) > 0.5f;
    bool axis_sign_valid = fabsf(knexus_ball_accel_axis_sign) > 0.5f;
    s_state.chassis_imu_mapping_ready = axis_valid && axis_sign_valid;
    /* The encoder acceleration is always a safe fallback, therefore rod
     * pre-leveling does not depend on the BMI088 forward-axis learning. */
    s_state.mapping_ready = roll_sign_valid;

    if (!s_state.bias_ready && stationary_for_bias && chassis_imu_valid) {
        for (uint8_t i = 0U; i < 3U; ++i) {
            s_bias_sum[i] += s_state.gravity_compensated_accel_mps2[i];
        }
        s_state.bias_sample_count++;
        if (s_state.bias_sample_count >= KNEXUS_BALL_BIAS_SAMPLES) {
            float inv_count = 1.0f / (float)s_state.bias_sample_count;
            for (uint8_t i = 0U; i < 3U; ++i) {
                s_state.bias_mps2[i] = s_bias_sum[i] * inv_count;
            }
            s_state.bias_ready = true;
        }
    }

    for (uint8_t i = 0U; i < 3U; ++i) {
        s_state.gravity_compensated_accel_mps2[i] -= s_state.bias_mps2[i];
    }

#if KNEXUS_BALL_CHASSIS_IMU_AUTO_MAP_ENABLE
    if (!s_state.chassis_imu_mapping_ready && s_state.bias_ready &&
        chassis_imu_valid && !chassis_stationary &&
        fabsf(chassis_encoder_accel_mps2) >=
            KNEXUS_BALL_CHASSIS_IMU_MAP_MIN_ACCEL_MPS2) {
        for (uint8_t i = 0U; i < 3U; ++i) {
            float axis_accel = s_state.gravity_compensated_accel_mps2[i];
            s_mapping_cross[i] += axis_accel * chassis_encoder_accel_mps2;
            s_mapping_axis_energy[i] += axis_accel * axis_accel;
        }
        s_mapping_encoder_energy +=
            chassis_encoder_accel_mps2 * chassis_encoder_accel_mps2;
        s_state.mapping_sample_count++;

        if (s_state.mapping_sample_count >=
            KNEXUS_BALL_CHASSIS_IMU_MAP_SAMPLES) {
            float best_correlation = 0.0f;
            int32_t best_axis = KNX_BALL_ACCEL_AXIS_UNKNOWN;
            for (uint8_t i = 0U; i < 3U; ++i) {
                float denominator = sqrtf(s_mapping_axis_energy[i] *
                                          s_mapping_encoder_energy);
                float correlation = denominator > 0.0001f
                    ? fabsf(s_mapping_cross[i]) / denominator : 0.0f;
                if (correlation > best_correlation) {
                    best_correlation = correlation;
                    best_axis = (int32_t)i;
                }
            }
            if (best_axis != KNX_BALL_ACCEL_AXIS_UNKNOWN &&
                best_correlation >=
                    KNEXUS_BALL_CHASSIS_IMU_MAP_MIN_CORRELATION) {
                knexus_ball_accel_axis = best_axis;
                knexus_ball_accel_axis_sign =
                    s_mapping_cross[best_axis] >= 0.0f ? 1.0f : -1.0f;
                s_state.chassis_imu_mapping_ready = true;
            } else {
                reset_mapping_learning();
            }
        }
    }
#endif

    float selected = chassis_encoder_valid ? chassis_encoder_accel_mps2
                                           : 0.0f;
    if (s_state.chassis_imu_mapping_ready && chassis_imu_valid &&
        s_state.bias_ready) {
        uint8_t axis = (uint8_t)knexus_ball_accel_axis;
        selected = knexus_ball_accel_axis_sign *
                   s_state.gravity_compensated_accel_mps2[axis];
    }
    s_state.selected_specific_force_mps2 = selected;

    float tau_s = 1.0f / (6.28318530718f *
                          KNEXUS_BALL_ACCEL_LPF_HZ);
    float alpha = dt_s / (tau_s + dt_s);
    if (!s_filter_initialized) {
        s_state.filtered_specific_force_mps2 = selected;
        s_filter_initialized = true;
    } else {
        s_state.filtered_specific_force_mps2 +=
            alpha * (selected - s_state.filtered_specific_force_mps2);
    }

    float imu_weight = (s_state.chassis_imu_mapping_ready &&
                        chassis_imu_valid && s_state.bias_ready)
        ? (chassis_encoder_valid ? KNEXUS_BALL_CHASSIS_IMU_BLEND_WEIGHT
                                 : 1.0f)
        : 0.0f;
    imu_weight = clampf(imu_weight, 0.0f, 1.0f);
    float sensor_chassis_accel =
        imu_weight * s_state.filtered_specific_force_mps2 +
        (1.0f - imu_weight) *
            (chassis_encoder_valid ? chassis_encoder_accel_mps2 : 0.0f);
    float command_weight = chassis_encoder_valid
        ? clampf(KNEXUS_BALL_COMMAND_ACCEL_WEIGHT, 0.0f, 1.0f)
        : 0.0f;
    float fused_chassis_accel =
        command_weight * chassis_command_accel_mps2 +
        (1.0f - command_weight) * sensor_chassis_accel;

    float motion_probe_mps2 =
        (s_state.chassis_imu_mapping_ready && chassis_imu_valid &&
         s_state.bias_ready)
            ? fabsf(s_state.filtered_specific_force_mps2)
            : fabsf(chassis_encoder_accel_mps2);
    bool motion_detected = motion_probe_mps2 >=
        KNEXUS_BALL_MOTION_UNLOCK_ACCEL_MPS2;
    bool command_motion_detected = chassis_encoder_valid &&
        fabsf(chassis_command_accel_mps2) >=
            KNEXUS_BALL_COMMAND_MOTION_ACCEL_MPS2;
    bool settle_detected = chassis_stationary &&
        !command_motion_detected &&
        motion_probe_mps2 <= KNEXUS_BALL_STATIONARY_IMU_ACCEL_MPS2;

    if (!compensation_requested) {
        /* Pre-leveling must never be interpreted as chassis motion. */
        s_motion_time_s = 0.0f;
        s_stationary_time_s =
            (float)KNEXUS_BALL_STATIONARY_HOLD_MS * 0.001f;
        s_state.chassis_stationary_locked = true;
    } else if (command_motion_detected) {
        /* The jerk-limited chassis request is known before the vehicle has
         * built measurable speed, so start tilting without sensor latency. */
        s_motion_time_s = 0.0f;
        s_stationary_time_s = 0.0f;
        s_state.chassis_stationary_locked = false;
    } else if (motion_detected) {
        s_motion_time_s += dt_s;
        s_stationary_time_s = 0.0f;
        if (s_motion_time_s >=
            (float)KNEXUS_BALL_MOTION_UNLOCK_HOLD_MS * 0.001f) {
            s_state.chassis_stationary_locked = false;
        }
    } else {
        s_motion_time_s = 0.0f;
        if (settle_detected) {
            s_stationary_time_s += dt_s;
        } else {
            s_stationary_time_s = 0.0f;
        }
    }
    if (!chassis_stationary && chassis_encoder_valid) {
        s_stationary_time_s = 0.0f;
        s_state.chassis_stationary_locked = false;
    }
    if (s_stationary_time_s >=
        (float)KNEXUS_BALL_STATIONARY_HOLD_MS * 0.001f) {
        s_state.chassis_stationary_locked = true;
    }
    s_state.chassis_accel_mps2 = s_state.chassis_stationary_locked
        ? 0.0f : fused_chassis_accel;

    s_state.compensation_active =
        compensation_requested && s_state.bias_ready &&
        s_state.mapping_ready && knexus_ball_compensation_enable > 0.5f;

    float target_roll_deg = 0.0f;
    if (s_state.compensation_active) {
        float denominator = knexus_ball_roll_gravity_sign *
                            KNEXUS_BALL_GRAVITY_MPS2;
        float sine_target =
            -knexus_ball_accel_compensation_gain *
            s_state.chassis_accel_mps2 / denominator;
        sine_target = clampf(sine_target, -0.95f, 0.95f);
        target_roll_deg = asinf(sine_target) * RAD_TO_DEG;
    }
    s_state.feedforward_roll_deg = target_roll_deg;

    float limited_target = clampf(
        target_roll_deg, KNEXUS_BALL_ROLL_TARGET_MIN_DEG,
        KNEXUS_BALL_ROLL_TARGET_MAX_DEG);
    s_state.target_limited = fabsf(limited_target - target_roll_deg) > 0.001f;
    float previous_target_roll_deg = s_last_target_roll_deg;
    float max_step = KNEXUS_BALL_ROLL_TARGET_SLEW_DPS * dt_s;
    float step = limited_target - s_last_target_roll_deg;
    step = clampf(step, -max_step, max_step);
    s_last_target_roll_deg += step;
    s_state.target_roll_deg = s_last_target_roll_deg;
    s_state.target_roll_rate_dps =
        (s_last_target_roll_deg - previous_target_roll_deg) / dt_s;

    bool predict = compensation_requested && s_state.bias_ready &&
                   s_state.mapping_ready;
    if (!predict) {
        s_state.predicted_ball_accel_mps2 = 0.0f;
        s_state.predicted_ball_velocity_mps = 0.0f;
        s_state.predicted_ball_position_m = 0.0f;
    } else {
        if (!s_predicting_last) {
            s_state.predicted_ball_velocity_mps = 0.0f;
            s_state.predicted_ball_position_m = 0.0f;
        }
        float pipe_specific_force = s_state.chassis_accel_mps2 +
            knexus_ball_roll_gravity_sign * KNEXUS_BALL_GRAVITY_MPS2 *
                sinf(rod_roll_deg * DEG_TO_RAD);
        s_state.predicted_ball_accel_mps2 =
            -KNEXUS_BALL_ROLLING_FACTOR * pipe_specific_force -
            knexus_ball_drag_s_inv * s_state.predicted_ball_velocity_mps;
        s_state.predicted_ball_velocity_mps +=
            s_state.predicted_ball_accel_mps2 * dt_s;
        s_state.predicted_ball_position_m +=
            s_state.predicted_ball_velocity_mps * dt_s;
        s_state.predicted_ball_velocity_mps = clampf(
            s_state.predicted_ball_velocity_mps,
            -KNEXUS_BALL_PREDICT_VELOCITY_LIMIT_MPS,
            KNEXUS_BALL_PREDICT_VELOCITY_LIMIT_MPS);
        s_state.predicted_ball_position_m = clampf(
            s_state.predicted_ball_position_m,
            -KNEXUS_BALL_PREDICT_POSITION_LIMIT_M,
            KNEXUS_BALL_PREDICT_POSITION_LIMIT_M);
    }
    s_predicting_last = predict;
}

void knx_ball_motion_comp_snapshot(knx_ball_motion_comp_state_t *out)
{
    if (out != NULL) *out = s_state;
}
