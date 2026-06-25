/**
 * @file    drv8701e.h
 * @author  kaiser
 * @version V5.0.0
 * @date    2026-06-02
 * @brief   DRV8701E 电机驱动 + ADC 电流采样封装
 *
 * 硬件连接 (PHASE/EN 模式):
 *   左电机 (MOTOR1):
 *     - EN/PWM: PA15 (TIM2_CH1) - PWM控制速度
 *     - PHASE:  PB0  - GPIO控制方向
 *     - Current sense: PA4 (ADC1 Rank2, CH18)
 *   右电机 (MOTOR2):
 *     - EN/PWM: PA2  (TIM2_CH3) - PWM控制速度
 *     - PHASE:  PB11 - GPIO控制方向
 *     - Current sense: PC0 (ADC1 Rank1, CH10)
 *
 * v5.0 变更:
 *   - ADC 电流采样迁移到 knx_adc platform 抽象
 *   - drv8701e.h 不再 include HAL 头文件
 *   - Current_Start/Poll use the platform ADC abstraction
 */

#ifndef __DRV8701E_H
#define __DRV8701E_H

#include "knx_pwm.h"
#include "knx_gpio.h"
#include "knx_adc.h"
#include "octolinker.h"

/* ==================== 方向宏定义 ==================== */
#define MOTOR1_DIR_SIGN      1
#define MOTOR2_DIR_SIGN     -1
#define MOTOR1_TARGET_SPEED_SIGN   1
#define MOTOR2_TARGET_SPEED_SIGN  1
#define MOTOR1_TARGET_CURRENT_SIGN  1
#define MOTOR2_TARGET_CURRENT_SIGN -1

/* ==================== ADC 电流采样配置 ==================== */
#define DRV8701E_ADC_VREF_MV           3300.0f
#define DRV8701E_ADC_MAX_VALUE         65535.0f
#define DRV8701E_RSENSE_MOHM           10.0f
#define DRV8701E_CSA_GAIN              20.0f
#define DRV8701E_CURRENT_VALID_CNT_WINDOW  200

/* ==================== 低通滤波器配置 ==================== */
#define DRV8701E_LPF_CUTOFF_HZ        10.0f   /* 电流滤波截止频率 */
#define DRV8701E_LPF_SAMPLE_HZ        1000.0f

/* ==================== Platform-backed 端口描述 ==================== */
typedef struct {
    knx_pwm_channel_t pwm;        /* PWM 输出通道 */
    knx_gpio_t        phase_gpio; /* PHASE 方向引脚 */
    knx_adc_channel_t current_adc; /* 电流采样 ADC 通道 */
    uint8_t           invert;     /* PHASE 电平反转 */
} drv8701e_port_t;

/* ==================== 电流采样结构体 ==================== */
typedef struct {
    uint16_t raw_left;
    uint16_t raw_right;
    float voltage_left_mv;
    float voltage_right_mv;
    float current_left_a;
    float current_right_a;
    float current_left_filtered_a;
    float current_right_filtered_a;
    uint8_t valid;
    uint32_t sample_count;
    uint32_t invalid_count;
    uint32_t timeout_count;
    uint32_t tim_cnt_when_read;
    uint32_t tim_arr;
    uint32_t adc_restart_count;
    uint8_t  adc_alive;
} DRV8701E_CurrentSample_t;

/* ==================== 电机句柄结构体 ==================== */
typedef struct {
    const drv8701e_port_t *port;    /* platform-backed 端口 (NULL=未 attach) */
    int8_t dir_sign;                /* 方向符号: 1 或 -1 */
    int8_t target_speed_sign;       /* 速度目标符号: 1 或 -1 */
    int32_t duty;                   /* 当前占空比 (有符号, 原始 compare 值) */
} DRV8701E_Motor_t;

/* ==================== 速度环 PID 控制器 ==================== */
typedef struct {
    float kp;
    float ki;
    float kd;
    float target;
    float target_used;
    float integral;
    float prev_error;
    float feedforward;
    float correction;
    float output;
    float out_min;
    float out_max;
    float integral_max;
    uint32_t last_ramp_tick_ms;
    void *rm_pid;
} DRV8701E_VelocityCtrl_t;

typedef struct {
    float kp;
    float ki;
    float target;
    float integral;
    float output;
    float out_min;
    float out_max;
    float integral_max;
    void *rm_pid;
} DRV8701E_CurrentCtrl_t;

/* ==================== 全局实例 ==================== */
extern DRV8701E_Motor_t motor_left;
extern DRV8701E_Motor_t motor_right;
extern DRV8701E_CurrentSample_t drv8701e_current;
extern DRV8701E_VelocityCtrl_t vc_left;
extern DRV8701E_VelocityCtrl_t vc_right;
extern DRV8701E_CurrentCtrl_t cc_left;
extern DRV8701E_CurrentCtrl_t cc_right;
extern float drv8701e_velocity_kv_cnt_per_mps;
extern float drv8701e_velocity_duty_deadzone_cnt;
extern float drv8701e_velocity_acc_limit_mps2;
extern float drv8701e_current_sample_fraction;
extern float drv8701e_current_sample_min_cnt;
extern float drv8701e_current_sample_default_cnt;
extern uint32_t drv8701e_current_sample_compare_cnt;

/* ==================== API 函数 ==================== */

void DRV8701E_Init(void);

void DRV8701E_AttachPorts(const drv8701e_port_t *left,
                          const drv8701e_port_t *right);

void DRV8701E_AttachCurrentSampleTrigger(const knx_pwm_channel_t *trigger);

void DRV8701E_SetDuty(DRV8701E_Motor_t *motor, int32_t duty);
void DRV8701E_SetDutyNorm(DRV8701E_Motor_t *motor, float duty_norm);

void DRV8701E_Stop(DRV8701E_Motor_t *motor);
void DRV8701E_StopAll(void);

/**
 * @brief  启动 ADC 电流采样 (使用 port 中的 current_adc)
 * @note   必须在 AttachPorts 之后调用
 */
void DRV8701E_Current_Start(void);

/**
 * @brief  轮询 ADC 电流采样 (使用 knx_adc_read_raw)
 * @param  timeout_ms: 每次 ADC 读取的超时 (ms)
 */
void DRV8701E_Current_Poll(uint32_t timeout_ms);

void DRV8701E_Current_CalibrateZero(uint16_t samples, uint32_t timeout_ms);

void DRV8701E_DebugOcto(Octolinker_Instance_t *octo);
void DRV8701E_VelocityInit(void);
void DRV8701E_VelocityControl(DRV8701E_VelocityCtrl_t *vc,
                               DRV8701E_Motor_t *motor,
                               float speed_actual);
void DRV8701E_CurrentCtrlInit(void);
void DRV8701E_CurrentControl(DRV8701E_CurrentCtrl_t *cc,
                             DRV8701E_Motor_t *motor,
                             float current_actual);

#endif /* __DRV8701E_H */
