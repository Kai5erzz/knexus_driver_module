/**
 * @file    bmi088.h
 * @author  kaiser
 * @version V5.0.0
 * @date    2026-06-02
 * @brief   BMI088 SPI驱动 (平台抽象层)
 *
 * 硬件连接:
 *   SPI1: SCK=PA5, MOSI=PD7, MISO=PG9
 *   CS0 (Accel): PH8
 *   CS1 (Gyro):  PI3
 */

#ifndef BMI088_H
#define BMI088_H

#include "knx_spi.h"
#include "knx_pwm.h"
#include "octolinker.h"

/* ==================== BMI088寄存器定义 ==================== */

/* Accelerometer */
#define BMI088_ACC_CHIP_ID          0x00
#define BMI088_ACC_CHIP_ID_VALUE    0x1E
#define BMI088_ACC_ERR_REG          0x02
#define BMI088_ACC_STATUS           0x03
#define BMI088_ACCEL_XOUT_L         0x12
#define BMI088_TEMP_M               0x22
#define BMI088_TEMP_L               0x23
#define BMI088_ACC_CONF             0x40
#define BMI088_ACC_CONF_MUST_SET    0x80
#define BMI088_ACC_NORMAL           0x20
#define BMI088_ACC_800_HZ           0x0B
#define BMI088_ACC_RANGE            0x41
#define BMI088_ACC_RANGE_6G         0x01
#define BMI088_INT1_IO_CTRL         0x53
#define BMI088_ACC_INT1_IO_ENABLE   0x08
#define BMI088_ACC_INT1_GPIO_PP     0x00
#define BMI088_ACC_INT1_GPIO_LOW    0x00
#define BMI088_INT_MAP_DATA         0x58
#define BMI088_ACC_INT1_DRDY        0x04
#define BMI088_ACC_PWR_CONF         0x7C
#define BMI088_ACC_PWR_ACTIVE_MODE  0x00
#define BMI088_ACC_PWR_CTRL         0x7D
#define BMI088_ACC_ENABLE_ACC_ON    0x04
#define BMI088_ACC_SOFTRESET        0x7E
#define BMI088_ACC_SOFTRESET_VALUE  0xB6

/* Gyroscope */
#define BMI088_GYRO_CHIP_ID         0x00
#define BMI088_GYRO_CHIP_ID_VALUE   0x0F
#define BMI088_GYRO_X_L             0x02
#define BMI088_GYRO_RANGE           0x0F
#define BMI088_GYRO_2000            0x00
#define BMI088_GYRO_BANDWIDTH       0x10
#define BMI088_GYRO_2000_230_HZ     0x01
#define BMI088_GYRO_BANDWIDTH_SET   0x80
#define BMI088_GYRO_LPM1            0x11
#define BMI088_GYRO_NORMAL_MODE     0x00
#define BMI088_GYRO_SOFTRESET       0x14
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6
#define BMI088_GYRO_CTRL            0x15
#define BMI088_DRDY_ON              0x80
#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16
#define BMI088_GYRO_INT3_GPIO_PP    0x00
#define BMI088_GYRO_INT3_GPIO_LOW   0x00
#define BMI088_GYRO_INT3_INT4_IO_MAP 0x18
#define BMI088_GYRO_DRDY_IO_INT3    0x01

/* ==================== 灵敏度常量 ==================== */

#define BMI088_ACCEL_6G_SEN   0.00179443359375f    /* 6g * 9.8 / 2^15, m/s^2/LSB */
#define BMI088_GYRO_2000_SEN  0.00106526443603169529841533860381f  /* 2000dps*pi/180/2^15, rad/s/LSB */
#define BMI088_TEMP_FACTOR    0.125f
#define BMI088_TEMP_OFFSET    23.0f

/* ==================== 默认校准回退值 (需手动测量) ==================== */

#define BMI088_GX_OFFSET  -0.00292451167f
#define BMI088_GY_OFFSET  -0.00267276145f
#define BMI088_GZ_OFFSET   0.00147538004f
#define BMI088_G_NORM      9.72065258f

/* ==================== IMU数据结构体 ==================== */

typedef struct {
    float accel[3];       /* 加速度 [X,Y,Z] (m/s^2), 已校准 */
    float gyro[3];        /* 角速度 [X,Y,Z] (rad/s), 已去偏 */
    float pitch;           /* 俯仰角 (deg), [-90, 90] */
    float yaw;             /* 航向角 (deg), [-180, 180] */
    float roll;            /* 横滚角 (deg), [-180, 180] */
    float yaw_total;       /* 航向累加角 (deg), 无环绕 */
    float roll_total;      /* 横滚累加角 (deg) */
    float pitch_total;     /* 俯仰累加角 (deg) */
    float q[4];            /* 四元数 [W,X,Y,Z] */
    float temperature;     /* 温度 (°C) */
    float gyro_offset[3]; /* 陀螺仪零偏 */
    float accel_scale;     /* 加速度计标度因数 */
    float g_norm;          /* 重力加速度标定值 */
    uint32_t timestamp;
    uint8_t  data_flags;
    uint8_t  is_calibrated;
    uint8_t  frame_count;

    /* 调试字段 */
    int16_t  accel_raw[3];   /* 加速度原始ADC值 */
    float    acc_norm;       /* 加速度矢量模 (m/s^2) */

    /* 诊断字段 (V4新增) */
    uint8_t  accel_ok;           /* 加速度计初始化成功 */
    uint8_t  gyro_ok;            /* 陀螺仪初始化成功 */
    uint8_t  accel_init_err;     /* 加速度计初始化错误码 */
    uint8_t  gyro_init_err;      /* 陀螺仪初始化错误码 */
    uint32_t accel_zero_cnt;     /* 加速度连续全0帧计数 */
    uint8_t  acc_chip_id_last;   /* 最近一次读取的ACC_CHIP_ID */
    uint8_t  acc_status_last;    /* 最近一次读取的ACC_STATUS */
    uint8_t  acc_err_last;       /* 最近一次读取的ACC_ERR_REG */
    uint32_t spi_err_cnt;        /* SPI传输错误计数 */

    /* 鲁棒校准诊断 */
    float    gyro_diff[3];       /* 校准时陀螺 max-min (rad/s) */
    uint8_t  cali_success;       /* 校准是否通过鲁棒检查 */
    uint8_t  cali_retry;         /* 校准重试次数 */
} DM_IMU_Data_t;

extern DM_IMU_Data_t imu_data;

/* ==================== 公共API ==================== */

/**
 * @brief  初始化BMI088 SPI驱动 (含校准, 阻塞)
 * @param  accel_spi  加速度计 SPI 总线描述符 (含 handle + CS GPIO)
 * @param  gyro_spi   陀螺仪 SPI 总线描述符 (含 handle + CS GPIO)
 */
void BMI088_Init(knx_spi_t *accel_spi, knx_spi_t *gyro_spi);

/**
 * @brief  读取BMI088全部数据 (加速度+陀螺仪+温度)
 * @note   从SPI直接读取, 更新全局imu_data
 */
void BMI088_Read(void);

/**
 * @brief  输出BMI088诊断信息到OctoLink
 * @param  octo: OctoLink实例指针
 */
void BMI088_DebugOcto(Octolinker_Instance_t *octo);

/* ==================== BMI088 加热温控 ==================== */

/* 温控调参全局变量 (Octo/GDB 可直接改) */
extern float bmi088_heater_enable;     /* 1=启用加热, 0=关闭 */
extern float bmi088_temp_target_c;     /* 目标温度 (°C), 默认 40 */
extern float bmi088_temp_kp;           /* 比例增益 */
extern float bmi088_temp_ki;           /* 积分增益 */
extern float bmi088_heater_duty_max;   /* 最大占空比 [0,1] */
extern float bmi088_heater_duty;       /* 当前输出占空比 (只读) */

/**
 * @brief  绑定加热器 PWM 通道, 启动 PWM, duty=0
 * @param  heater_pwm  PWM 通道描述符 (board 层静态定义)
 */
void BMI088_AttachHeater(const knx_pwm_channel_t *heater_pwm);

#endif /* BMI088_H */
