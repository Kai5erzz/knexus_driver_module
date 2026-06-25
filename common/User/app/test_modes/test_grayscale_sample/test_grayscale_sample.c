/**
 * @file    test_grayscale_sample.c
 * @author  kaiser
 * @version V2.1.0
 * @date    2026-06-07
 * @brief   路口样本采集程序 (非阻塞状态机)
 *
 * 状态机:
 *   IDLE → (KEY0) → CAL_BLACK_DELAY → CAL_BLACK_SAMPLE →
 *   CAL_WHITE_DELAY → CAL_WHITE_SAMPLE → CAL_DONE →
 *   WAIT_SAMPLE → (KEY1) → SAMPLING → (到达 20cm) → SAMPLE_DONE
 *
 * SAMPLE_DONE 状态:
 *   - KEY0: 通过 Octolinker_SendLiteBitMatrix() 输出 64×8 bit matrix (VAR_ID 100, 64 bytes)
 *   - KEY1: 清空矩阵, 重新开始采样
 *
 * 采样策略:
 *   运动过程中按编码器等距离触发 (next_sample_pos),
 *   将 8 路归一化灰度 [0.0, 1.0] → [0, 255] 存入矩阵行.
 *   到达后不足行用最后一行补齐.
 *
 * LED 指示:
 *   LED1 - 校准进度: 快闪(校准中) → 常亮(校准完成)
 *   LED2 - 运动状态: 亮(前进中) → 灭(到达/停止)
 */

#include "test_grayscale_sample.h"
#include "knx_grayscale.h"
#include "knx_drive.h"
#include "knx_motor.h"
#include "knx_imu.h"
#include "knx_key.h"
#include "knx_led.h"
#include "knx_beep.h"
#include "knx_sys.h"
#include "knx_time.h"
#include "encoder.h"
#include "track_sensor.h"
#include "octolinker.h"
#include "track_tinycnn.h"
#include <string.h>

/* ==================== 状态定义 ==================== */

typedef enum {
    STATE_IDLE = 0,             /**< 等待 KEY0 开始校准 */
    STATE_CAL_BLACK_DELAY,      /**< 提示放黑线, 延时 2 秒 */
    STATE_CAL_BLACK_SAMPLE,     /**< 采样黑线 (每通道取 min) */
    STATE_CAL_WHITE_DELAY,      /**< 提示放白底, 延时 2 秒 */
    STATE_CAL_WHITE_SAMPLE,     /**< 采样白底 (每通道取 max) */
    STATE_CAL_DONE,             /**< 校准完成, 写入参数 */
    STATE_WAIT_SAMPLE,          /**< 等待 KEY1 开始采样运行 */
    STATE_SAMPLING,             /**< 前进中, 按距离触发灰度采样 */
    STATE_SAMPLE_DONE,          /**< 到达 20cm, 等待输出/重采 */
} sample_state_t;

/* ==================== 参数 ==================== */

#define TARGET_DISTANCE_M       0.20f   /**< 目标前进距离 20cm */
#define POSITION_REACHED_TH     0.02f   /**< 位置到达阈值 2cm */
#define SPEED_SETTLED_TH        0.15f   /**< 速度稳定阈值 m/s */
#define SETTLE_DELAY_MS         200     /**< 到达后等待稳定时间 ms */
#define CAL_DELAY_MS            2000    /**< 校准前延时 (放置传感器) */
#define CAL_SAMPLES             100     /**< 每次校准采样次数 */

/* 采样矩阵参数 */
#define SAMPLE_ROWS             64      /**< 矩阵行数 (采样点数) */
#define SAMPLE_INTERVAL_M       (TARGET_DISTANCE_M / SAMPLE_ROWS)  /**< 每行对应距离 */

/* 角度环参数 (IMU 直接反馈) */
#define ANGLE_KP                0.3f
#define ANGLE_KI                0.05f
#define ANGLE_KD                0.1f
#define ANGLE_MAX_OUT           1.0f    /**< 最大角速度修正 rad/s */

/* 位置环参数 (编码器反馈) */
#define POS_KP                  3.0f
#define POS_KI                  0.0f
#define POS_KD                  0.0f
#define POS_MAX_OUT             0.5f    /**< 最大线速度 m/s */

/* OctoLink 矩阵输出 var_id (64×8 bit matrix, 64 bytes, 一次发完) */
#define VAR_ID_SAMPLE_MATRIX    100
#define VAR_ID_NN_PRED_CLASS    120
#define VAR_ID_NN_CONFIDENCE    121
#define VAR_ID_NN_LOGIT_BASE    122
#define VAR_ID_NN_PROB_BASE     130

#define TRACK_CLASS_CROSS_COMPLEX  0U
#define TRACK_CLASS_NORMAL         1U
#define NN_CROSS_CONF_TH           0.95f

_Static_assert(TRACK_NN_ROWS == SAMPLE_ROWS, "TRACK_NN_ROWS must match SAMPLE_ROWS");
_Static_assert(TRACK_NN_COLS == KNX_GRAYSCALE_CH_NUM, "TRACK_NN_COLS must match grayscale channel count");

/* ==================== 私有数据 ==================== */

static Octolinker_Instance_t *s_octo;
static sample_state_t s_state;
static uint32_t s_ctrl_tick;
static uint32_t s_settle_tick;
static uint32_t s_move_start_ms;

/* 校准用临时数据 */
static uint32_t s_cal_tick;             /**< 校准阶段起始时刻 */
static uint16_t s_cal_sample_cnt;       /**< 当前采样计数 */
static uint32_t s_imu_div;              /**< IMU 分频计数 (2ms → 500Hz) */
static uint16_t s_cal_min[KNX_GRAYSCALE_CH_NUM];
static uint16_t s_cal_max[KNX_GRAYSCALE_CH_NUM];

/* 运动控制状态 */
static float s_target_yaw_deg;          /**< 按下 KEY1 瞬间的 IMU yaw (度) */
static float s_pos_err_sum;             /**< 位置环积分 */
static float s_angle_err_sum;           /**< 角度环积分 */
static float s_last_pos_err;            /**< 位置环上次误差 */
static float s_last_angle_err;          /**< 角度环上次误差 */
static float s_angle_corr;              /**< 角度环输出, 在 IMU 更新时刷新 */

/* 采样矩阵 (bit matrix: 每行 1 byte, bit0=ch0 ... bit7=ch7) */
static uint8_t  s_sample_matrix[SAMPLE_ROWS];
static uint8_t  s_sample_row;           /**< 当前已采行数 */
static float    s_next_sample_pos;      /**< 下次采样对应的里程阈值 (m) */
static bool     s_matrix_sent;          /**< 本次矩阵是否已发送 */
static knx_status_t s_last_matrix_send_status; /**< 最近一次矩阵发送结果 */

/* NN inference keeps the original packed matrix intact and expands it on demand. */
static uint8_t s_nn_input[TRACK_NN_ROWS][TRACK_NN_COLS];
static track_tinycnn_result_t s_nn_result;
static bool s_nn_valid;

/* ==================== LED 辅助 ==================== */

static void led_calibrating_blink(void)
{
    static uint32_t blink_tick = 0;
    static bool led_on = false;
    uint32_t now = knx_millis();
    if (now - blink_tick >= 100U) {
        blink_tick = now;
        led_on = !led_on;
        knx_led_set(KNX_LED_1, led_on);
    }
}

/* ==================== 前向声明 ==================== */

static float wrap_angle(float a);
static void save_sample_row(void);
static void fill_remaining_rows(void);
static void run_nn_inference(void);
static void send_nn_result(void);
static bool is_cross_complex_detected(void);

/* ==================== 采样矩阵输出 ==================== */

static void send_sample_matrix(void)
{
    if (s_octo == NULL) return;

    /* 一次性发送 64×8 bit matrix (64 bytes payload) */
    s_last_matrix_send_status = Octolinker_SendLiteBitMatrix(
        s_octo,
        VAR_ID_SAMPLE_MATRIX,
        s_sample_matrix,
        SAMPLE_ROWS,
        KNX_GRAYSCALE_CH_NUM
    );
}

static void build_nn_input_from_bit_matrix(void)
{
    for (uint8_t r = 0; r < TRACK_NN_ROWS; r++) {
        uint8_t packed = s_sample_matrix[r];
        for (uint8_t c = 0; c < TRACK_NN_COLS; c++) {
            uint8_t mask = (uint8_t)(1U << c);
            s_nn_input[r][c] = ((packed & mask) != 0U) ? 255U : 0U;
        }
    }
}

static void run_nn_inference(void)
{
    build_nn_input_from_bit_matrix();
    track_tinycnn_predict_u8(s_nn_input, &s_nn_result);
    s_nn_valid = true;
}

static void send_nn_result(void)
{
    if (s_octo == NULL || !s_nn_valid) return;

    Octolinker_SendLiteU8(s_octo, VAR_ID_NN_PRED_CLASS, s_nn_result.pred_class);
    Octolinker_SendLiteF32(s_octo, VAR_ID_NN_CONFIDENCE, s_nn_result.confidence);

    for (uint8_t i = 0; i < TRACK_NN_CLASSES; i++) {
        Octolinker_SendLiteF32(s_octo, (uint8_t)(VAR_ID_NN_LOGIT_BASE + i), s_nn_result.logits[i]);
    }

    for (uint8_t i = 0; i < TRACK_NN_CLASSES; i++) {
        Octolinker_SendLiteF32(s_octo, (uint8_t)(VAR_ID_NN_PROB_BASE + i), s_nn_result.probs[i]);
    }
}

static bool is_cross_complex_detected(void)
{
    return s_nn_valid &&
           s_nn_result.pred_class == TRACK_CLASS_CROSS_COMPLEX &&
           s_nn_result.confidence >= NN_CROSS_CONF_TH;
}

/* ==================== 采样辅助 ==================== */

static void save_sample_row(void)
{
    knx_grayscale_data_t gray;
    knx_grayscale_snapshot(&gray);
    s_sample_matrix[s_sample_row] = gray.digital_byte;
}

static void fill_remaining_rows(void)
{
    if (s_sample_row >= SAMPLE_ROWS) return;

    uint8_t src_val = (s_sample_row > 0U) ? s_sample_matrix[s_sample_row - 1U] : 0U;

    while (s_sample_row < SAMPLE_ROWS) {
        s_sample_matrix[s_sample_row] = src_val;
        s_sample_row++;
    }
}

/* ==================== 校准状态处理 ==================== */

static void enter_cal_black_delay(void)
{
    s_cal_tick = knx_millis();
    s_state = STATE_CAL_BLACK_DELAY;
}

static void handle_cal_black_delay(void)
{
    led_calibrating_blink();
    if (knx_millis() - s_cal_tick >= CAL_DELAY_MS) {
        /* 延时结束, 开始采样黑线 */
        for (uint8_t i = 0; i < KNX_GRAYSCALE_CH_NUM; i++) {
            s_cal_min[i] = 65535;
        }
        s_cal_sample_cnt = 0;
        s_state = STATE_CAL_BLACK_SAMPLE;
    }
}

static void handle_cal_black_sample(void)
{
    led_calibrating_blink();

    TrackSensor_Scan();
    for (uint8_t i = 0; i < KNX_GRAYSCALE_CH_NUM; i++) {
        if (track_raw[i] < s_cal_min[i]) {
            s_cal_min[i] = track_raw[i];
        }
    }
    s_cal_sample_cnt++;

    if (s_cal_sample_cnt >= CAL_SAMPLES) {
        /* 黑线采样完成, 响一声, 进入白底延时 */
        knx_beep_beep(150);
        s_cal_tick = knx_millis();
        s_state = STATE_CAL_WHITE_DELAY;
    }
}

static void handle_cal_white_delay(void)
{
    led_calibrating_blink();
    if (knx_millis() - s_cal_tick >= CAL_DELAY_MS) {
        /* 延时结束, 开始采样白底 */
        for (uint8_t i = 0; i < KNX_GRAYSCALE_CH_NUM; i++) {
            s_cal_max[i] = 0;
        }
        s_cal_sample_cnt = 0;
        s_state = STATE_CAL_WHITE_SAMPLE;
    }
}

static void handle_cal_white_sample(void)
{
    led_calibrating_blink();

    TrackSensor_Scan();
    for (uint8_t i = 0; i < KNX_GRAYSCALE_CH_NUM; i++) {
        if (track_raw[i] > s_cal_max[i]) {
            s_cal_max[i] = track_raw[i];
        }
    }
    s_cal_sample_cnt++;

    if (s_cal_sample_cnt >= CAL_SAMPLES) {
        /* 白底采样完成, 响一声 */
        knx_beep_beep(150);
        s_state = STATE_CAL_DONE;
    }
}

static void handle_cal_done(void)
{
    /* 将采样结果写入灰度模块 */
    knx_grayscale_set_calibration(s_cal_min, s_cal_max);

    /* LED1 常亮, 表示校准完成 */
    knx_led_on(KNX_LED_1);
    s_state = STATE_WAIT_SAMPLE;
}

/* ==================== 运动和采样状态处理 ==================== */

static float wrap_angle(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static void enter_sampling(void)
{
    /* 锁定当前 IMU yaw 作为目标航向 */
    knx_imu_data_t imu;
    knx_imu_snapshot(&imu);
    s_target_yaw_deg = imu.yaw;

    /* 重置编码器里程和 PID 状态 */
    Encoder_Reset(&encoder_left);
    Encoder_Reset(&encoder_right);
    s_pos_err_sum = 0.0f;
    s_angle_err_sum = 0.0f;
    s_last_pos_err = 0.0f;
    s_last_angle_err = 0.0f;
    s_angle_corr = 0.0f;

    /* 清空采样矩阵, 重置采样位置 */
    memset(s_sample_matrix, 0, sizeof(s_sample_matrix));
    s_sample_row = 0;
    s_next_sample_pos = 0.0f;
    s_matrix_sent = false;
    s_last_matrix_send_status = KNX_OK;
    memset(s_nn_input, 0, sizeof(s_nn_input));
    memset(&s_nn_result, 0, sizeof(s_nn_result));
    s_nn_valid = false;

    knx_sys_set_mode(KNX_RUN_MODE_SPEED);
    knx_led_on(KNX_LED_2);
    s_move_start_ms = knx_millis();
    s_settle_tick = 0U;
    s_state = STATE_SAMPLING;
}

static void handle_wait_sample(void)
{
    if (knx_key_just_pressed(KNX_KEY_1)) {
        enter_sampling();
    }
}

static void handle_sampling(void)
{
    /* 位置反馈: 左右编码器平均里程 */
    float position = 0.5f * (encoder_left.position_m + encoder_right.position_m);
    float pos_err = TARGET_DISTANCE_M - position;

    /* 安全超时: 10 秒 */
    if (knx_millis() - s_move_start_ms > 10000U) {
        fill_remaining_rows();
        knx_motor_set_target(0.0f, 0.0f);
        knx_led_off(KNX_LED_2);
        s_state = STATE_SAMPLE_DONE;
        return;
    }

    /* ---- 等距离灰度采样 (next_sample_pos 方式) ---- */
    while (s_sample_row < SAMPLE_ROWS && position >= s_next_sample_pos) {
        save_sample_row();
        s_sample_row++;
        s_next_sample_pos = (float)s_sample_row * SAMPLE_INTERVAL_M;
    }

    /* ---- 到达判定 ---- */
    float pos_err_abs = pos_err;
    if (pos_err_abs < 0.0f) pos_err_abs = -pos_err_abs;
    bool reached = (pos_err_abs < POSITION_REACHED_TH);

    if (reached) {
        knx_motor_set_target(0.0f, 0.0f);
        if (s_settle_tick == 0U) {
            s_settle_tick = knx_millis();
        } else if (knx_millis() - s_settle_tick >= SETTLE_DELAY_MS) {
            fill_remaining_rows();
            run_nn_inference();
            knx_led_off(KNX_LED_2);
            send_nn_result();
            knx_beep_beep(200);
            s_state = STATE_SAMPLE_DONE;
        }
        return;
    } else {
        s_settle_tick = 0U;
    }

    /* ---- 位置环 1kHz (编码器) ---- */
    s_pos_err_sum += pos_err * 0.001f;
    if (s_pos_err_sum > POS_MAX_OUT) s_pos_err_sum = POS_MAX_OUT;
    if (s_pos_err_sum < -POS_MAX_OUT) s_pos_err_sum = -POS_MAX_OUT;
    float d_pos = (pos_err - s_last_pos_err) / 0.001f;
    s_last_pos_err = pos_err;
    float base_speed = POS_KP * pos_err + POS_KI * s_pos_err_sum + POS_KD * d_pos;
    if (base_speed > POS_MAX_OUT) base_speed = POS_MAX_OUT;
    if (base_speed < -POS_MAX_OUT) base_speed = -POS_MAX_OUT;

    /* ---- 角度环 500Hz (IMU, 与 imu_update 同步) ---- */
    if (s_imu_div == 0U) {  /* IMU 刚更新过 */
        knx_imu_data_t imu;
        knx_imu_snapshot(&imu);
        float angle_err = wrap_angle(s_target_yaw_deg - imu.yaw);

        s_angle_err_sum += angle_err * 0.002f;  /* dt = 2ms = IMU 周期 */
        if (s_angle_err_sum > ANGLE_MAX_OUT) s_angle_err_sum = ANGLE_MAX_OUT;
        if (s_angle_err_sum < -ANGLE_MAX_OUT) s_angle_err_sum = -ANGLE_MAX_OUT;
        float d_angle = (angle_err - s_last_angle_err) / 0.002f;
        s_last_angle_err = angle_err;
        s_angle_corr = ANGLE_KP * angle_err + ANGLE_KI * s_angle_err_sum + ANGLE_KD * d_angle;
        if (s_angle_corr > ANGLE_MAX_OUT) s_angle_corr = ANGLE_MAX_OUT;
        if (s_angle_corr < -ANGLE_MAX_OUT) s_angle_corr = -ANGLE_MAX_OUT;
    }
    /* 非 IMU 更新周期, s_angle_corr 保持上次值 */

    /* ---- 合并输出 ---- */
    float left_speed  = base_speed - s_angle_corr;
    float right_speed = base_speed + s_angle_corr;
    knx_motor_set_target(left_speed, right_speed);
}

static void handle_sample_done(void)
{
    /* KEY0: 发送本次采样矩阵 */
    if (knx_key_just_pressed(KNX_KEY_0)) {
        send_sample_matrix();
        send_nn_result();
        s_matrix_sent = true;
        knx_beep_beep(100);
    }

    /* KEY1: 清空矩阵, 重新开始采样 */
    if (knx_key_just_pressed(KNX_KEY_1)) {
        enter_sampling();
    }
}

/* ==================== 接口实现 ==================== */

void test_grayscale_sample_init(void *octo)
{
    s_octo = (Octolinker_Instance_t *)octo;
    s_state = STATE_IDLE;
    s_ctrl_tick = 0U;
    s_settle_tick = 0U;
    s_move_start_ms = 0U;
    s_cal_tick = 0U;
    s_cal_sample_cnt = 0U;
    s_imu_div = 0U;
    s_angle_corr = 0.0f;

    /* 采样矩阵 */
    memset(s_sample_matrix, 0, sizeof(s_sample_matrix));
    s_sample_row = 0;
    s_next_sample_pos = 0.0f;
    s_matrix_sent = false;
    s_last_matrix_send_status = KNX_OK;
    memset(s_nn_input, 0, sizeof(s_nn_input));
    memset(&s_nn_result, 0, sizeof(s_nn_result));
    s_nn_valid = false;

    knx_grayscale_init();
    knx_drive_reset_odometry();
    knx_led_off(KNX_LED_1);
    knx_led_off(KNX_LED_2);
}

void test_grayscale_sample_loop(void)
{
    uint32_t now = knx_millis();

    /* ---- 控制回路 1ms ---- */
    if (now - s_ctrl_tick >= 1U) {
        s_ctrl_tick = now;

        /* IMU 更新 500Hz (每 2ms 一次) — 所有状态都需要 */
        s_imu_div++;
        if (s_imu_div >= 2U) {
            s_imu_div = 0U;
            knx_imu_update();
        }

        switch (s_state) {
        case STATE_IDLE:
            if (knx_key_just_pressed(KNX_KEY_0)) {
                enter_cal_black_delay();
            }
            knx_motor_set_target(0.0f, 0.0f);
            knx_motor_update();
            break;

        case STATE_CAL_BLACK_DELAY:
            handle_cal_black_delay();
            break;

        case STATE_CAL_BLACK_SAMPLE:
            handle_cal_black_sample();
            break;

        case STATE_CAL_WHITE_DELAY:
            handle_cal_white_delay();
            break;

        case STATE_CAL_WHITE_SAMPLE:
            handle_cal_white_sample();
            break;

        case STATE_CAL_DONE:
            handle_cal_done();
            break;

        case STATE_WAIT_SAMPLE:
            handle_wait_sample();
            knx_motor_set_target(0.0f, 0.0f);
            knx_motor_update();
            break;

        case STATE_SAMPLING:
            /* 先更新灰度再采样, 确保 snapshot 拿到最新数据 */
            knx_grayscale_update();
            handle_sampling();
            knx_motor_update();
            break;

        case STATE_SAMPLE_DONE:
            handle_sample_done();
            knx_motor_set_target(0.0f, 0.0f);
            knx_motor_update();
            break;
        }
    }
}
