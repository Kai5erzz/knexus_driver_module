/**
 * @file    bmi088.c
 * @author  kaiser
 * @version V5.0.0
 * @date    2026-06-02
 * @brief   BMI088 SPI驱动实现 (平台抽象�?
 *
 * V5.0 修改:
 *   - Removed direct platform header dependencies
 *   - SPI 传输通过 knx_spi_transmit_receive 逐字节完�? *   - CS 控制通过 knx_spi_cs_low / knx_spi_cs_high
 *   - BMI088_Init 接收两个 knx_spi_t* (accel / gyro)
 *   - 保留全部寄存器配置、dummy byte 读时序、校准逻辑
 */

#include "bmi088.h"
#include "knx_time.h"
#include "PID.h"
#include <math.h>

/* ==================== 私有变量 ==================== */

static knx_spi_t *spi_accel;
static knx_spi_t *spi_gyro;

static float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
static float BMI088_GYRO_SEN  = BMI088_GYRO_2000_SEN;

/* accel 连续�?帧阈�?*/
#define ACCEL_ZERO_THRESHOLD  5

DM_IMU_Data_t imu_data;

/* 诊断计数 */
static uint32_t bmi088_init_count = 0;
static uint32_t bmi088_read_count = 0;
static uint32_t bmi088_debug_count = 0;

/* ==================== 加热温控 (PI) ==================== */

/* 调参全局变量, Octo/GDB 可直接修�?*/
float bmi088_heater_enable   = 1.0f;
float bmi088_temp_target_c   = 40.0f;
float bmi088_temp_kp         = 0.05f;
float bmi088_temp_ki         = 0.03f;
float bmi088_heater_duty_max = 0.30f;   /* 最�?30% 占空�? 保守防过�?*/
float bmi088_heater_duty     = 0.0f;    /* 当前输出 (只读) */

static knx_pwm_channel_t s_heater_pwm;
static uint8_t s_heater_attached = 0;
static float s_heater_integral = 0.0f;
static uint32_t s_heater_last_ms = 0;
static pid_obj_t *s_heater_pid = NULL;

/* ==================== SPI 底层 (带错误计�? ==================== */

static knx_status_t bmi088_spi_rw(knx_spi_t *spi, uint8_t tx, uint8_t *rx)
{
    knx_status_t st = knx_spi_transmit_receive(spi, &tx, rx, 1, 5);
    if (st != KNX_OK) {
        imu_data.spi_err_cnt++;
    }
    return st;
}

static uint8_t s_spi_frame_error;  /* 当前帧 SPI 错误标志 (BMI088_Read 每帧复位) */

static uint8_t bmi088_rw_byte(knx_spi_t *spi, uint8_t tx)
{
    uint8_t rx = 0;
    knx_status_t st = bmi088_spi_rw(spi, tx, &rx);
    if (st != KNX_OK) {
        /* SPI 超时/错误: 标记本帧数据不可信 */
        s_spi_frame_error = 1;
    }
    return rx;
}

/* ==================== 寄存器读写函�?==================== */

static void bmi088_accel_read_reg(uint8_t reg, uint8_t *data)
{
    knx_spi_cs_low(spi_accel);
    bmi088_rw_byte(spi_accel, reg | 0x80);
    bmi088_rw_byte(spi_accel, 0x55);  /* accel dummy byte */
    *data = bmi088_rw_byte(spi_accel, 0x55);
    knx_spi_cs_high(spi_accel);
}

static void bmi088_accel_write_reg(uint8_t reg, uint8_t val)
{
    knx_spi_cs_low(spi_accel);
    bmi088_rw_byte(spi_accel, reg);
    bmi088_rw_byte(spi_accel, val);
    knx_spi_cs_high(spi_accel);
}

static void bmi088_accel_read_multi(uint8_t reg, uint8_t *buf, uint8_t len)
{
    knx_spi_cs_low(spi_accel);
    bmi088_rw_byte(spi_accel, reg | 0x80);
    bmi088_rw_byte(spi_accel, 0x55);  /* accel dummy byte */
    for (uint8_t i = 0; i < len; i++)
        buf[i] = bmi088_rw_byte(spi_accel, 0x55);
    knx_spi_cs_high(spi_accel);
}

static void bmi088_gyro_read_reg(uint8_t reg, uint8_t *data)
{
    knx_spi_cs_low(spi_gyro);
    bmi088_rw_byte(spi_gyro, reg | 0x80);
    *data = bmi088_rw_byte(spi_gyro, 0x55);
    knx_spi_cs_high(spi_gyro);
}

static void bmi088_gyro_write_reg(uint8_t reg, uint8_t val)
{
    knx_spi_cs_low(spi_gyro);
    bmi088_rw_byte(spi_gyro, reg);
    bmi088_rw_byte(spi_gyro, val);
    knx_spi_cs_high(spi_gyro);
}

static void bmi088_gyro_read_multi(uint8_t reg, uint8_t *buf, uint8_t len)
{
    knx_spi_cs_low(spi_gyro);
    bmi088_rw_byte(spi_gyro, reg | 0x80);
    for (uint8_t i = 0; i < len; i++)
        buf[i] = bmi088_rw_byte(spi_gyro, 0x55);
    knx_spi_cs_high(spi_gyro);
}

/* ==================== 加速度计初始化 (�?readback 校验) ==================== */

static uint8_t bmi088_accel_init(void)
{
    uint8_t res;

    /* 读取 CHIP_ID (两次, 确保 SPI 就绪) */
    bmi088_accel_read_reg(BMI088_ACC_CHIP_ID, &res);
    knx_delay_ms(1);
    bmi088_accel_read_reg(BMI088_ACC_CHIP_ID, &res);
    knx_delay_ms(1);

    /* 软复�?*/
    bmi088_accel_write_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    knx_delay_ms(80);

    /* 复位后重新读�?CHIP_ID */
    bmi088_accel_read_reg(BMI088_ACC_CHIP_ID, &res);
    knx_delay_ms(1);
    bmi088_accel_read_reg(BMI088_ACC_CHIP_ID, &res);
    knx_delay_ms(1);

    if (res != BMI088_ACC_CHIP_ID_VALUE)
        return 0xFF;  /* CHIP_ID 不匹�?*/

    /* �?PWR_CTRL, 等待稳定后再写后续寄存器 */
    bmi088_accel_write_reg(BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON);
    knx_delay_ms(10);  /* 电源稳定延时 */

    /* 校验 PWR_CTRL */
    bmi088_accel_read_reg(BMI088_ACC_PWR_CTRL, &res);
    if (res != BMI088_ACC_ENABLE_ACC_ON)
        return 0x01;  /* PWR_CTRL 写入失败 */

    /* �?PWR_CONF (active mode) */
    bmi088_accel_write_reg(BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE);
    knx_delay_ms(1);
    bmi088_accel_read_reg(BMI088_ACC_PWR_CONF, &res);
    if (res != BMI088_ACC_PWR_ACTIVE_MODE)
        return 0x02;  /* PWR_CONF 写入失败 */

    /* �?ACC_CONF */
    uint8_t acc_conf_val = BMI088_ACC_CONF_MUST_SET | BMI088_ACC_NORMAL | BMI088_ACC_800_HZ;
    bmi088_accel_write_reg(BMI088_ACC_CONF, acc_conf_val);
    knx_delay_ms(1);
    bmi088_accel_read_reg(BMI088_ACC_CONF, &res);
    if (res != acc_conf_val)
        return 0x03;  /* ACC_CONF 写入失败 */

    /* �?ACC_RANGE */
    bmi088_accel_write_reg(BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G);
    knx_delay_ms(1);
    bmi088_accel_read_reg(BMI088_ACC_RANGE, &res);
    if (res != BMI088_ACC_RANGE_6G)
        return 0x04;  /* ACC_RANGE 写入失败 */

    /* 写中断配�?(非关�? 不做 readback) */
    bmi088_accel_write_reg(BMI088_INT1_IO_CTRL,
                           BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW);
    knx_delay_ms(1);
    bmi088_accel_write_reg(BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY);
    knx_delay_ms(1);

    return 0;  /* 成功 */
}

/* ==================== 陀螺仪初始�?(�?readback 校验) ==================== */

static uint8_t bmi088_gyro_init(void)
{
    uint8_t res;

    /* 读取 CHIP_ID */
    bmi088_gyro_read_reg(BMI088_GYRO_CHIP_ID, &res);
    knx_delay_ms(1);
    bmi088_gyro_read_reg(BMI088_GYRO_CHIP_ID, &res);
    knx_delay_ms(1);

    /* 软复�?*/
    bmi088_gyro_write_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    knx_delay_ms(80);

    /* 复位后重新读�?CHIP_ID */
    bmi088_gyro_read_reg(BMI088_GYRO_CHIP_ID, &res);
    knx_delay_ms(1);
    bmi088_gyro_read_reg(BMI088_GYRO_CHIP_ID, &res);
    knx_delay_ms(1);

    if (res != BMI088_GYRO_CHIP_ID_VALUE)
        return 0xFF;

    /* �?GYRO_RANGE */
    bmi088_gyro_write_reg(BMI088_GYRO_RANGE, BMI088_GYRO_2000);
    knx_delay_ms(1);
    bmi088_gyro_read_reg(BMI088_GYRO_RANGE, &res);
    if (res != BMI088_GYRO_2000)
        return 0x01;

    /* �?GYRO_BANDWIDTH */
    uint8_t bw_val = BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_SET;
    bmi088_gyro_write_reg(BMI088_GYRO_BANDWIDTH, bw_val);
    knx_delay_ms(1);
    bmi088_gyro_read_reg(BMI088_GYRO_BANDWIDTH, &res);
    if (res != bw_val)
        return 0x02;

    /* �?LPM1 (normal mode) */
    bmi088_gyro_write_reg(BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE);
    knx_delay_ms(1);
    bmi088_gyro_read_reg(BMI088_GYRO_LPM1, &res);
    if (res != BMI088_GYRO_NORMAL_MODE)
        return 0x03;

    /* �?GYRO_CTRL (DRDY enable) */
    bmi088_gyro_write_reg(BMI088_GYRO_CTRL, BMI088_DRDY_ON);
    knx_delay_ms(1);
    bmi088_gyro_read_reg(BMI088_GYRO_CTRL, &res);
    if (res != BMI088_DRDY_ON)
        return 0x04;

    /* 写中断配�?(非关�? */
    bmi088_gyro_write_reg(BMI088_GYRO_INT3_INT4_IO_CONF,
                          BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW);
    knx_delay_ms(1);
    bmi088_gyro_write_reg(BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3);
    knx_delay_ms(1);

    return 0;  /* 成功 */
}

/* ==================== 鲁棒校准 ==================== */

#define CALI_SAMPLES       3000   /* 每次校准采样�?*/
#define CALI_MAX_RETRY     5      /* 最大重试次�?*/
#define GYRO_DIFF_THRESH   0.1f   /* 陀�?max-min 阈�?(rad/s) */
#define G_NORM_DIFF_THRESH 0.3f   /* 加速度模�?max-min 阈�?(m/s^2) */
#define G_NORM_ERR_THRESH  0.5f   /* 加速度模值偏�?9.8 阈�?*/

static void bmi088_calibrate(uint8_t do_accel, uint8_t do_gyro)
{
    uint8_t buf[8];
    int16_t raw;

    imu_data.accel_scale = 1.0f;
    imu_data.cali_success = 0;
    imu_data.cali_retry = 0;
    for (uint8_t j = 0; j < 3; j++) imu_data.gyro_diff[j] = 0.0f;

    knx_delay_ms(100);

    for (uint8_t attempt = 0; attempt < CALI_MAX_RETRY; attempt++) {
        imu_data.cali_retry = attempt + 1;

        float g_norm_acc = 0.0f;
        float gyro_off[3] = {0.0f};
        float gyro_max[3] = {-1e30f, -1e30f, -1e30f};
        float gyro_min[3] = { 1e30f,  1e30f,  1e30f};
        float g_norm_max = -1e30f;
        float g_norm_min =  1e30f;
        uint8_t aborted = 0;

        for (uint16_t i = 0; i < CALI_SAMPLES; i++) {
            if (do_accel) {
                bmi088_accel_read_multi(BMI088_ACCEL_XOUT_L, buf, 6);
                raw = (int16_t)((buf[1] << 8) | buf[0]);
                float ax = raw * BMI088_ACCEL_SEN;
                raw = (int16_t)((buf[3] << 8) | buf[2]);
                float ay = raw * BMI088_ACCEL_SEN;
                raw = (int16_t)((buf[5] << 8) | buf[4]);
                float az = raw * BMI088_ACCEL_SEN;
                float gn = sqrtf(ax * ax + ay * ay + az * az);
                g_norm_acc += gn;
                if (gn > g_norm_max) g_norm_max = gn;
                if (gn < g_norm_min) g_norm_min = gn;

                /* 提前检�? 加速度模值抖动过�?*/
                if ((g_norm_max - g_norm_min) > G_NORM_DIFF_THRESH) {
                    aborted = 1;
                    break;
                }
            }

            if (do_gyro) {
                bmi088_gyro_read_multi(BMI088_GYRO_CHIP_ID, buf, 8);
                if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
                    raw = (int16_t)((buf[3] << 8) | buf[2]);
                    float gx = raw * BMI088_GYRO_SEN;
                    gyro_off[0] += gx;
                    if (gx > gyro_max[0]) gyro_max[0] = gx;
                    if (gx < gyro_min[0]) gyro_min[0] = gx;

                    raw = (int16_t)((buf[5] << 8) | buf[4]);
                    float gy = raw * BMI088_GYRO_SEN;
                    gyro_off[1] += gy;
                    if (gy > gyro_max[1]) gyro_max[1] = gy;
                    if (gy < gyro_min[1]) gyro_min[1] = gy;

                    raw = (int16_t)((buf[7] << 8) | buf[6]);
                    float gz = raw * BMI088_GYRO_SEN;
                    gyro_off[2] += gz;
                    if (gz > gyro_max[2]) gyro_max[2] = gz;
                    if (gz < gyro_min[2]) gyro_min[2] = gz;

                    /* 提前检�? 陀螺抖动过�?*/
                    float diff0 = gyro_max[0] - gyro_min[0];
                    float diff1 = gyro_max[1] - gyro_min[1];
                    float diff2 = gyro_max[2] - gyro_min[2];
                    if (diff0 > GYRO_DIFF_THRESH || diff1 > GYRO_DIFF_THRESH || diff2 > GYRO_DIFF_THRESH) {
                        aborted = 1;
                        break;
                    }
                }
            }
            knx_delay_ms(1);
        }

        /* 记录 diff 诊断 */
        for (uint8_t j = 0; j < 3; j++)
            imu_data.gyro_diff[j] = gyro_max[j] - gyro_min[j];

        if (aborted) {
            knx_delay_ms(50);
            continue;  /* 重试 */
        }

        /* 验证通过: 检查最终结�?*/
        uint8_t pass = 1;

        if (do_accel) {
            float g_avg = g_norm_acc / CALI_SAMPLES;
            float g_diff = g_norm_max - g_norm_min;
            if (g_diff > G_NORM_DIFF_THRESH || fabsf(g_avg - 9.8f) > G_NORM_ERR_THRESH) {
                pass = 0;
            }
        }

        if (do_gyro) {
            for (uint8_t j = 0; j < 3; j++) {
                float off = gyro_off[j] / CALI_SAMPLES;
                if (fabsf(off) > 0.01f) {
                    pass = 0;
                    break;
                }
            }
        }

        if (pass) {
            /* 校准成功 */
            if (do_accel) {
                imu_data.g_norm = g_norm_acc / CALI_SAMPLES;
                imu_data.accel_scale = 9.81f / imu_data.g_norm;
            }
            if (do_gyro) {
                for (uint8_t j = 0; j < 3; j++)
                    imu_data.gyro_offset[j] = gyro_off[j] / CALI_SAMPLES;
            }
            imu_data.cali_success = 1;
            imu_data.is_calibrated = 1;
            break;
        }

        knx_delay_ms(50);
    }

    /* 全部重试失败: 回退默认�?*/
    if (!imu_data.cali_success) {
        if (do_accel) {
            imu_data.g_norm = BMI088_G_NORM;
            imu_data.accel_scale = 9.81f / imu_data.g_norm;
        }
        if (do_gyro) {
            imu_data.gyro_offset[0] = BMI088_GX_OFFSET;
            imu_data.gyro_offset[1] = BMI088_GY_OFFSET;
            imu_data.gyro_offset[2] = BMI088_GZ_OFFSET;
        }
        imu_data.is_calibrated = 1;
    }

    /* 读取一次温�?*/
    bmi088_accel_read_multi(BMI088_TEMP_M, buf, 2);
    raw = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (raw > 1023) raw -= 2048;
    imu_data.temperature = raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

/* ==================== 加热器绑�?& PI温控 ==================== */

void BMI088_AttachHeater(const knx_pwm_channel_t *heater_pwm)
{
    if (heater_pwm == NULL) return;
    s_heater_pwm = *heater_pwm;
    s_heater_attached = 1;
    knx_pwm_start(&s_heater_pwm);
    knx_pwm_set_duty(&s_heater_pwm, 0.0f);
    s_heater_integral = 0.0f;
    s_heater_last_ms = knx_millis();
    bmi088_heater_duty = 0.0f;
    if (s_heater_pid != NULL) {
        pid_clear(s_heater_pid);
    }
}

static void bmi088_heater_update(void)
{
    /* 未绑定或未启�? 关闭加热, 清积�?*/
    if (!s_heater_attached) return;

    if (bmi088_heater_enable < 0.5f) {
        bmi088_heater_duty = 0.0f;
        s_heater_integral = 0.0f;
        s_heater_last_ms = knx_millis();
        if (s_heater_pid != NULL) {
            pid_clear(s_heater_pid);
        }
        knx_pwm_set_duty(&s_heater_pwm, 0.0f);
        return;
    }

    /* 温度读取失败时不�?(保留上次输出) */
    if (!imu_data.accel_ok) return;

    float error = bmi088_temp_target_c - imu_data.temperature;

    /* 温度高于目标: duty=0, 不积�?*/
    if (error <= 0.0f) {
        bmi088_heater_duty = 0.0f;
        knx_pwm_set_duty(&s_heater_pwm, 0.0f);
        s_heater_integral = 0.0f;
        if (s_heater_pid != NULL) {
            pid_clear(s_heater_pid);
        }
        return;
    }

    float duty_max = bmi088_heater_duty_max;
    if (duty_max < 0.0f) duty_max = 0.0f;
    if (duty_max > 1.0f) duty_max = 1.0f;

    if (s_heater_pid == NULL) {
        pid_config_t config = INIT_PID_CONFIG(bmi088_temp_kp,
                                              bmi088_temp_ki,
                                              0.0f,
                                              duty_max,
                                              duty_max,
                                              PID_Integral_Limit);
        s_heater_pid = pid_register(&config);
    }
    if (s_heater_pid == NULL) return;

    s_heater_pid->Kp = bmi088_temp_kp;
    s_heater_pid->Ki = bmi088_temp_ki;
    s_heater_pid->Kd = 0.0f;
    s_heater_pid->MaxOut = duty_max;
    s_heater_pid->IntegralLimit = duty_max;
    s_heater_pid->Improve = PID_Integral_Limit;
    s_heater_pid->DeadBand = 0.0f;

    float duty = pid_calculate(s_heater_pid, imu_data.temperature, bmi088_temp_target_c);
    if (duty < 0.0f) duty = 0.0f;
    if (duty > duty_max) duty = duty_max;

    bmi088_heater_duty = duty;
    s_heater_integral = s_heater_pid->Iout;
    s_heater_last_ms = knx_millis();
    knx_pwm_set_duty(&s_heater_pwm, duty);
}

/* ==================== 公共API ==================== */

void BMI088_Init(knx_spi_t *accel_spi, knx_spi_t *gyro_spi)
{
    bmi088_init_count++;
    spi_accel = accel_spi;
    spi_gyro  = gyro_spi;

    /* 清零 imu_data */
    for (uint8_t i = 0; i < 3; i++) {
        imu_data.accel[i] = 0;
        imu_data.gyro[i] = 0;
        imu_data.gyro_offset[i] = 0;
    }
    imu_data.accel_scale = 1.0f;
    imu_data.g_norm = 9.81f;
    imu_data.pitch = 0; imu_data.roll = 0; imu_data.yaw = 0;
    imu_data.yaw_total = 0; imu_data.roll_total = 0; imu_data.pitch_total = 0;
    imu_data.temperature = 0;
    imu_data.is_calibrated = 0;
    imu_data.frame_count = 0;
    imu_data.accel_ok = 0;
    imu_data.gyro_ok = 0;
    imu_data.accel_init_err = 0;
    imu_data.gyro_init_err = 0;
    imu_data.accel_zero_cnt = 0;
    imu_data.acc_chip_id_last = 0;
    imu_data.acc_status_last = 0;
    imu_data.acc_err_last = 0;
    imu_data.spi_err_cnt = 0;
    imu_data.gyro_diff[0] = 0; imu_data.gyro_diff[1] = 0; imu_data.gyro_diff[2] = 0;
    imu_data.cali_success = 0;
    imu_data.cali_retry = 0;

    /* 初始化前强制 CS 拉高, 确保总线空闲 */
    knx_spi_cs_high(spi_accel);
    knx_spi_cs_high(spi_gyro);
    knx_delay_ms(10);

    /* 初始化加速度�? 最多重�?10 �?*/
    uint8_t retry;
    for (retry = 0; retry < 10; retry++) {
        uint8_t err = bmi088_accel_init();
        if (err == 0) {
            imu_data.accel_ok = 1;
            imu_data.accel_init_err = 0;
            break;
        }
        imu_data.accel_init_err = err;
        knx_delay_ms(50);
    }

    /* 初始化陀螺仪, 最多重�?10 �?*/
    for (retry = 0; retry < 10; retry++) {
        uint8_t err = bmi088_gyro_init();
        if (err == 0) {
            imu_data.gyro_ok = 1;
            imu_data.gyro_init_err = 0;
            break;
        }
        imu_data.gyro_init_err = err;
        knx_delay_ms(50);
    }

    /* 条件校准: 至少有一个传感器 OK 才校�?*/
    if (imu_data.accel_ok && imu_data.gyro_ok) {
        /* 完整校准: accel + gyro */
        bmi088_calibrate(1, 1);
    } else if (imu_data.gyro_ok) {
        /* �?gyro 校准, accel 标记无效 */
        bmi088_calibrate(0, 1);
        imu_data.accel_ok = 0;
    } else if (imu_data.accel_ok) {
        /* �?accel 校准 (罕见) */
        bmi088_calibrate(1, 0);
    } else {
        /* 两个都失�? 不校�?*/
        imu_data.is_calibrated = 0;
    }

    /* 启动后主动读一�? 验证 Read 路径可用, frame_count 至少�?1 */
    BMI088_Read();
}

void BMI088_Read(void)
{
    bmi088_read_count++;
    s_spi_frame_error = 0;
    uint8_t buf[8];
    int16_t raw;

    /* ---- 加速度�?---- */
    if (imu_data.accel_ok) {
        bmi088_accel_read_multi(BMI088_ACCEL_XOUT_L, buf, 6);
        int16_t ax = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t ay = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t az = (int16_t)((buf[5] << 8) | buf[4]);

        /* 全零检�?*/
        if (ax == 0 && ay == 0 && az == 0) {
            imu_data.accel_zero_cnt++;
            if (imu_data.accel_zero_cnt >= ACCEL_ZERO_THRESHOLD) {
                /* 连续全零, 诊断: 读取 CHIP_ID / STATUS / ERR */
                bmi088_accel_read_reg(BMI088_ACC_CHIP_ID, &imu_data.acc_chip_id_last);
                bmi088_accel_read_reg(BMI088_ACC_STATUS, &imu_data.acc_status_last);
                bmi088_accel_read_reg(BMI088_ACC_ERR_REG, &imu_data.acc_err_last);

                if (imu_data.acc_chip_id_last != BMI088_ACC_CHIP_ID_VALUE) {
                    /* CHIP_ID 丢失, 触发重新初始�?*/
                    imu_data.accel_ok = 0;
                    uint8_t err = bmi088_accel_init();
                    if (err == 0) {
                        imu_data.accel_ok = 1;
                        imu_data.accel_init_err = 0;
                        imu_data.accel_zero_cnt = 0;
                    } else {
                        imu_data.accel_init_err = err;
                    }
                }
                /* CHIP_ID 正确但全�? 可能是传感器暂时无输�? 保留 last_valid_accel 不覆�?*/
            }
            /* 单帧全零: 不覆�?accel[], 保留上一次有效�?*/
        } else {
            /* 有效�?*/
            imu_data.accel_zero_cnt = 0;
            imu_data.accel_raw[0] = ax;
            imu_data.accel_raw[1] = ay;
            imu_data.accel_raw[2] = az;
            imu_data.accel[0] = ax * BMI088_ACCEL_SEN * imu_data.accel_scale;
            imu_data.accel[1] = ay * BMI088_ACCEL_SEN * imu_data.accel_scale;
            imu_data.accel[2] = az * BMI088_ACCEL_SEN * imu_data.accel_scale;
            imu_data.acc_norm = sqrtf(imu_data.accel[0] * imu_data.accel[0] +
                                      imu_data.accel[1] * imu_data.accel[1] +
                                      imu_data.accel[2] * imu_data.accel[2]);
        }
    }

    /* ---- 陀螺仪 ---- */
    if (imu_data.gyro_ok) {
        bmi088_gyro_read_multi(BMI088_GYRO_CHIP_ID, buf, 8);
        if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
            raw = (int16_t)((buf[3] << 8) | buf[2]);
            imu_data.gyro[0] = raw * BMI088_GYRO_SEN - imu_data.gyro_offset[0];
            raw = (int16_t)((buf[5] << 8) | buf[4]);
            imu_data.gyro[1] = raw * BMI088_GYRO_SEN - imu_data.gyro_offset[1];
            raw = (int16_t)((buf[7] << 8) | buf[6]);
            imu_data.gyro[2] = raw * BMI088_GYRO_SEN - imu_data.gyro_offset[2];
        }
    }

    /* 本帧 SPI 出错: 数据不可信, 跳过温度/加热/姿态更新, 仅累计帧计数 */
    if (s_spi_frame_error) {
        imu_data.frame_count++;
        return;
    }

    /* ---- 温度 (挂在 accel CS �? ---- */
    if (imu_data.accel_ok) {
        bmi088_accel_read_multi(BMI088_TEMP_M, buf, 2);
        raw = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
        if (raw > 1023) raw -= 2048;
        imu_data.temperature = raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

        /* 温度更新后立即执行加热温�?*/
        bmi088_heater_update();
    }

    /* ---- 姿态解�? 三轴均用陀螺仪积分 ---- */
    uint32_t now_ms = knx_millis();
    uint32_t elapsed_ms = now_ms - imu_data.timestamp;
    float dt = elapsed_ms * 0.001f;
    if (imu_data.timestamp == 0 || dt <= 0.0f || dt > 0.05f)
        dt = 0.001f;
    imu_data.timestamp = now_ms;

    float DEG = 57.295779513f;

    float roll_inc  = imu_data.gyro[0] * DEG * dt;
    float pitch_inc = imu_data.gyro[1] * DEG * dt;
    float yaw_inc   = imu_data.gyro[2] * DEG * dt;

    imu_data.roll  += roll_inc;
    imu_data.pitch += pitch_inc;
    imu_data.yaw   += yaw_inc;

    /* 累加�?(无环�? 用于角度闭环) */
    imu_data.roll_total  += roll_inc;
    imu_data.pitch_total += pitch_inc;
    imu_data.yaw_total   += yaw_inc;

    /* yaw ±180环绕�?*/
    if (imu_data.yaw > 180.0f)  imu_data.yaw -= 360.0f;
    if (imu_data.yaw < -180.0f) imu_data.yaw += 360.0f;

    /*
     * 注意: 当前不是完整姿态解�?(�?Mahony/Madgwick), q 不再每帧写死�?     * 若需要四元数, 应从 roll/pitch/yaw 转换:
     *   q = euler_to_quat(roll, pitch, yaw)
     * 暂不生成, 避免误导上层�?     */

    imu_data.frame_count++;
}

void BMI088_DebugOcto(Octolinker_Instance_t *octo)
{
    if (octo == NULL) return;
    bmi088_debug_count++;

    Octolinker_SendF32(octo, 20, imu_data.roll);
    Octolinker_SendF32(octo, 21, imu_data.pitch);
    Octolinker_SendF32(octo, 22, imu_data.yaw);
    Octolinker_SendF32(octo, 23, imu_data.temperature);

    /* 加热温控调试通道 */
    Octolinker_SendF32(octo, 24, bmi088_temp_target_c);
    Octolinker_SendF32(octo, 25, bmi088_heater_duty);
}
