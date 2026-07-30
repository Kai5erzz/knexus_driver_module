#ifndef KNEXUS_CONFIG_H
#define KNEXUS_CONFIG_H

#include "knx_target_config.h"

/*
 * KNexus 2026 用户配置中心
 *
 * 使用原则：
 * 1. 普通使用者只需要修改本文件，不要修改 STM32/MSPM0 板级引脚文件。
 * 2. 工作模式通过“注释/取消注释”选择，必须且只能启用一个模式。
 * 3. 带 DEFAULT 后缀的宏只提供上电默认值；程序会把它们复制到普通变量，
 *    因此仍可通过 OctoLink/GDB 在线调参。
 * 4. 速度单位为 m/s，角度单位为 rad，角速度单位为 rad/s，时间单位为 ms。
 */

/* ======================== 1. 工作模式选择 ======================== */

// #define KNEXUS_MODE_LINE_FOLLOW          /* 巡线任务模式：KEY0 校准，KEY1 启停 */
// #define KNEXUS_MODE_INTERSECTION_SAMPLE  /* 路口采样模式：采集 64x8 灰度矩阵 */
// #define KNEXUS_MODE_PID_TUNE             /* 电机/底盘 PID 基础调参模式 */
// #define KNEXUS_MODE_USER                 /* 用户自定义模式，见 knexus_mode_user.c */
// #define KNEXUS_MODE_BOARD_TEST           /* 全板硬件验收：两种 MCU 共用 */
#define KNEXUS_MODE_SCREW_TEST              /* 丝杆测试：KEY0按住正转，KEY1按住反转 */

#if (defined(KNEXUS_MODE_LINE_FOLLOW) + \
     defined(KNEXUS_MODE_INTERSECTION_SAMPLE) + \
     defined(KNEXUS_MODE_PID_TUNE) + \
     defined(KNEXUS_MODE_USER) + \
     defined(KNEXUS_MODE_BOARD_TEST) + \
     defined(KNEXUS_MODE_SCREW_TEST)) != 1
#error "KNexus: 必须且只能启用一个 KNEXUS_MODE_xxx 工作模式"
#endif

/* PID 调参子模式：只有启用 KNEXUS_MODE_PID_TUNE 时生效，且只能选一个。 */
#define KNEXUS_PID_TUNE_SPEED_STEP          /* 直线速度阶跃，左右轮同速 */
// #define KNEXUS_PID_TUNE_TURN_STEP        /* 角速度阶跃，左右轮反向 */

#if defined(KNEXUS_MODE_PID_TUNE) && \
    ((defined(KNEXUS_PID_TUNE_SPEED_STEP) + \
      defined(KNEXUS_PID_TUNE_TURN_STEP)) != 1)
#error "KNexus: PID 调参模式必须且只能启用一个 KNEXUS_PID_TUNE_xxx 子模式"
#endif

/* ======================== 2. 调试与任务周期 ======================== */

#define KNEXUS_DEBUG_OCTOLINK_ENABLE        1U  /* 1=发送 OctoLink 数据；0=完全关闭 */
#define KNEXUS_DEBUG_LINE_DATA_ENABLE       1U  /* 巡线归一化值、误差和控制量 */
#define KNEXUS_DEBUG_PID_DATA_ENABLE        1U  /* PID 目标、反馈、误差和输出 */
#define KNEXUS_DEBUG_IMU_DATA_ENABLE        1U  /* BMI088、温控和零偏学习信息 */

/* 当前赛题不需要路口判断：0=不创建推理任务、不运行TinyCNN；1=启用。 */
#define KNEXUS_INTERSECTION_ENABLE           0U

/*
 * 六个基础任务 + 按模式启用的小球/路口任务：
 * fast    1 kHz：电机内环、安全、蜂鸣器（最高优先级，禁止阻塞）
 * control 100/200 Hz：BMI088、里程计和底盘外环
 * track   100 Hz：8路灰度采样和归一化（不运行模型）
 * app     100 Hz：按键、模式状态机和上层策略
 * ball    100 Hz：H题在control任务中的小球位置闭环子周期
 * perception 100 Hz：可选；路口窗口、TinyCNN/规则识别和事件分发
 * comm    200 Hz：串口/CAN协议轮询与心跳
 * debug    50 Hz：OctoLink，仅允许最低优先级运行
 *
 * PHASE 用于错开同一时刻的大量唤醒；必须小于对应 PERIOD。
 */
#define KNEXUS_FAST_PERIOD_MS               1U
#if defined(KNX_PLATFORM_MSPM0)
#define KNEXUS_CONTROL_PERIOD_MS           10U  /* M0+ 软件浮点 EKF 需要较长周期 */
#else
#define KNEXUS_CONTROL_PERIOD_MS            5U
#endif
#define KNEXUS_TRACK_PERIOD_MS             10U
#define KNEXUS_APP_PERIOD_MS               10U
#define KNEXUS_PERCEPTION_PERIOD_MS        10U
#define KNEXUS_COMM_PERIOD_MS               5U
#define KNEXUS_DEBUG_PERIOD_MS             20U

#define KNEXUS_FAST_PHASE_MS                0U
#define KNEXUS_CONTROL_PHASE_MS             0U
#define KNEXUS_TRACK_PHASE_MS               1U
#define KNEXUS_APP_PHASE_MS                 2U
#define KNEXUS_COMM_PHASE_MS                3U
#define KNEXUS_PERCEPTION_PHASE_MS          4U
#define KNEXUS_DEBUG_PHASE_MS               7U
#define KNEXUS_CAN_HEARTBEAT_MS           100U
#define KNEXUS_LINK_HEARTBEAT_MS          250U
#define KNEXUS_OCTO_BASE_ID               700U

/* ======================== 3. 机械与安全参数 ======================== */

#define KNEXUS_WHEEL_RADIUS_M_DEFAULT       0.0325f
#define KNEXUS_WHEEL_BASE_M_DEFAULT         0.1600f
#define KNEXUS_TRACK_WIDTH_M_DEFAULT        0.1600f
#define KNEXUS_BODY_MASS_KG_DEFAULT         0.2152f
#define KNEXUS_BODY_INERTIA_KGM2_DEFAULT    0.00064386311f
#define KNEXUS_COM_HEIGHT_M_DEFAULT         0.05288f
#define KNEXUS_MOTOR_TORQUE_KT_DEFAULT      0.00984f
#define KNEXUS_MOTOR_GEAR_RATIO_DEFAULT    28.0f
#define KNEXUS_MOTOR_EFFICIENCY_DEFAULT     0.65f

/* 电机和编码器极性只允许填 +1.0f 或 -1.0f。不要在 drive 层重复取反。 */
#define KNEXUS_MOTOR_LEFT_CMD_SIGN_DEFAULT       1.0f
#define KNEXUS_MOTOR_RIGHT_CMD_SIGN_DEFAULT     -1.0f
#define KNEXUS_MOTOR_LEFT_FEEDBACK_SIGN_DEFAULT -1.0f
#define KNEXUS_MOTOR_RIGHT_FEEDBACK_SIGN_DEFAULT -1.0f

#define KNEXUS_SAFETY_MAX_TILT_DEG_DEFAULT       65.0f
#define KNEXUS_SAFETY_MAX_CURRENT_A_DEFAULT       4.2f
#define KNEXUS_SAFETY_MAX_SPEED_MPS_DEFAULT       8.0f
#define KNEXUS_SAFETY_IMU_TIMEOUT_MS_DEFAULT     50U
/* 上电后 BMI088/EKF 需要短暂建立首帧；宽限期内仅告警，不锁存 IMU_FAIL。 */
#define KNEXUS_SAFETY_STARTUP_GRACE_MS_DEFAULT  1000U
#define KNEXUS_SAFETY_WARMUP_TOLERANCE_C_DEFAULT  0.8f
#define KNEXUS_SAFETY_WARMUP_STABLE_MS_DEFAULT  500U

/* ======================== 4. 电机速度环 ======================== */

/*
 * 调节顺序：先前馈，再 Kp，最后 Ki；通常保持 Kd=0。
 * Kp 增大可提高响应，过大会抖动；Ki 消除稳态误差，过大会来回振荡。
 * 数值单位为驱动器内部 PWM 计数/速度误差，并非标准 SI 增益。
 */
#define KNEXUS_MOTOR_LEFT_KP_DEFAULT        1500.0f
#define KNEXUS_MOTOR_LEFT_KI_DEFAULT         120.0f
#define KNEXUS_MOTOR_LEFT_KD_DEFAULT           0.0f
#define KNEXUS_MOTOR_RIGHT_KP_DEFAULT       1500.0f
#define KNEXUS_MOTOR_RIGHT_KI_DEFAULT        120.0f
#define KNEXUS_MOTOR_RIGHT_KD_DEFAULT          0.0f
#define KNEXUS_MOTOR_INTEGRAL_MAX_DEFAULT   1500.0f
/*
 * 2026-07-29 实车稳态拟合约为 2750～2990 cnt/(m/s)。保留450计数静摩擦补偿后，
 * 第一轮取2100，让速度环主要由前馈承担，PID只修正负载和左右差异。
 */
#define KNEXUS_MOTOR_FEEDFORWARD_CNT_PER_MPS_DEFAULT 2100.0f
#define KNEXUS_MOTOR_DUTY_DEADZONE_CNT_DEFAULT       450.0f
#define KNEXUS_MOTOR_ACCEL_LIMIT_MPS2_DEFAULT          3.0f

/* ======================== 5. 通用底盘限制与 PID ======================== */

#define KNEXUS_DRIVE_MAX_LINEAR_MPS_DEFAULT          1.0f
#define KNEXUS_DRIVE_MAX_ANGULAR_RADPS_DEFAULT       4.0f
#define KNEXUS_DRIVE_MAX_LINEAR_ACCEL_MPS2_DEFAULT   1.5f
#define KNEXUS_DRIVE_MAX_ANGULAR_ACCEL_RADPS2_DEFAULT 6.0f
#define KNEXUS_DRIVE_COMMAND_TIMEOUT_MS_DEFAULT      200U

/* 距离环：输出目标线速度。Kp 过大会在终点前后往返。 */
#define KNEXUS_POSITION_KP_DEFAULT          1.0f
#define KNEXUS_POSITION_KI_DEFAULT          0.03f
#define KNEXUS_POSITION_KD_DEFAULT          0.10f
#define KNEXUS_POSITION_MAX_OUT_DEFAULT     0.80f
#define KNEXUS_POSITION_I_LIMIT_DEFAULT     0.30f

/* 航向角环：输入 rad，输出目标角速度 rad/s。 */
#define KNEXUS_ANGLE_KP_DEFAULT             0.005f
#define KNEXUS_ANGLE_KI_DEFAULT             0.0005f
#define KNEXUS_ANGLE_KD_DEFAULT             0.004f
#define KNEXUS_ANGLE_MAX_OUT_DEFAULT        2.0f
#define KNEXUS_ANGLE_I_LIMIT_DEFAULT        0.30f

/* 线速度与角速度修正环。通常先把电机速度环调好，再调整这里。 */
#define KNEXUS_LINEAR_SPEED_KP_DEFAULT      0.80f
#define KNEXUS_LINEAR_SPEED_KI_DEFAULT      0.00f
#define KNEXUS_LINEAR_SPEED_KD_DEFAULT      0.00f
#define KNEXUS_LINEAR_SPEED_MAX_OUT_DEFAULT 1.00f
#define KNEXUS_LINEAR_SPEED_I_LIMIT_DEFAULT 0.30f
#define KNEXUS_ANGULAR_SPEED_KP_DEFAULT      0.80f
#define KNEXUS_ANGULAR_SPEED_KI_DEFAULT      0.00f
#define KNEXUS_ANGULAR_SPEED_KD_DEFAULT      0.00f
#define KNEXUS_ANGULAR_SPEED_MAX_OUT_DEFAULT 4.00f
#define KNEXUS_ANGULAR_SPEED_I_LIMIT_DEFAULT 0.50f

/* ======================== 6. 巡线任务模式 ======================== */

#define KNEXUS_LINE_CAL_DELAY_MS            2000U /* 黑/白面放置等待时间 */
#define KNEXUS_LINE_CAL_SAMPLES              100U /* 每个表面采样次数 */
#define KNEXUS_LINE_CAL_MIN_SPAN               40U /* 任一路量程小于此值则校准失败 */
#define KNEXUS_LINE_TRACK_FRESH_MS            100U /* 灰度数据最大允许年龄 */
#define KNEXUS_LINE_LOST_STOP_ENABLE             1U /* 1=确认脱线后自动停车 */
#define KNEXUS_LINE_LOST_STOP_CONFIRM_MS       150U /* 连续脱线确认时间，滤除瞬时空白 */
#define KNEXUS_LINE_MAX_DUTY_DEFAULT           0.80f
#define KNEXUS_LINE_BASE_SPEED_MPS_DEFAULT     0.30f

/* Kp=吸线力度；Ki=消除长期偏线；Kd=抑制快速摆动。 */
#define KNEXUS_LINE_KP_DEFAULT                 0.75f
#define KNEXUS_LINE_KI_DEFAULT                 0.15f
#define KNEXUS_LINE_KD_DEFAULT                 0.0015f
#define KNEXUS_LINE_MAX_ANGULAR_RADPS_DEFAULT  4.00f
#define KNEXUS_LINE_MIN_ANGULAR_RADPS_DEFAULT  0.35f /* 克服低速转向死区 */
#define KNEXUS_LINE_INTEGRAL_LIMIT_DEFAULT     0.80f
#define KNEXUS_LINE_MIN_STRENGTH_DEFAULT       0.30f
#define KNEXUS_LINE_RECOVERY_RADPS_DEFAULT     2.00f
#define KNEXUS_LINE_ERROR_FILTER_ALPHA         0.25f
#define KNEXUS_LINE_D_FILTER_ALPHA             0.20f
#define KNEXUS_LINE_ANGULAR_SMOOTH_ALPHA       0.65f
#define KNEXUS_LINE_CURVE_SLOWDOWN_GAIN        0.12f
#define KNEXUS_LINE_MIN_SPEED_SCALE            0.55f
#define KNEXUS_LINE_ERROR_ABS_LIMIT             3.50f
#define KNEXUS_LINE_WEAK_FLOOR                  0.15f
#define KNEXUS_LINE_VALID_CONFIDENCE            0.50f
#define KNEXUS_LINE_INTEGRAL_REVERSE_DECAY      0.70f
#define KNEXUS_LINE_INTEGRAL_NORMAL_DECAY       0.99f
#define KNEXUS_LINE_INTEGRAL_WEAK_DECAY         0.80f
#define KNEXUS_LINE_STICTION_SMOOTHING          0.18f
#define KNEXUS_LINE_SEARCH_SPEED_BASE           0.18f
#define KNEXUS_LINE_SEARCH_SPEED_CONFIDENCE     0.82f

/*
 * 纵向竞速速度规划：转向控制量不经过这里，因此不会削弱当前转弯力度。
 * 曲率指标 = |滤波误差| + 预瞄时间 * |误差变化率|；入弯快响应、出弯慢释放，
 * 避免传感器在相邻通道间切换时让车速来回抽动。
 */
#define KNEXUS_LINE_CURVE_LOOKAHEAD_S            0.025f
#define KNEXUS_LINE_CURVE_LOOKAHEAD_MAX_ERROR    0.75f
#define KNEXUS_LINE_CURVE_ATTACK_ALPHA           0.25f
#define KNEXUS_LINE_CURVE_RELEASE_ALPHA          0.06f

/*
 * S形纵向加减速器。加速较小以保护滚球稳定；减速稍快以保证入弯及时，
 * JERK 限制用于消除速度命令的一阶突变。RESPONSE 越大，速度变化越舒缓。
 */
#define KNEXUS_LINE_ACCEL_LIMIT_MPS2              0.80f
#define KNEXUS_LINE_DECEL_LIMIT_MPS2              1.20f
#define KNEXUS_LINE_JERK_LIMIT_MPS3               6.00f
#define KNEXUS_LINE_SPEED_RESPONSE_S              0.22f
#define KNEXUS_LINE_MEASURED_ACCEL_FILTER_ALPHA   0.15f

/*
 * Straight-line anti-hunting zone.  Only small errors are softened; once the
 * error leaves this zone, the original cornering gains and limits apply.
 */
/*
 * 8 路传感器为偶数排列，黑线落在两颗中央探头之一时，质心误差天然约为
 * -0.5 / +0.5；原来的 0.10 死区会把这种“实际已居中”误判成需要持续转向，
 * 形成左右往复摆动。实车数据表明把中心死区扩到 0.55 后，直线修正量可
 * 降低约 85%，而 |error|>=0.70 的弯道仍使用原来的完整 Kp/Ki/Kd 与限幅。
 */
#define KNEXUS_LINE_STRAIGHT_ZONE_ERROR          0.70f
#define KNEXUS_LINE_STRAIGHT_DEADBAND_ERROR      0.55f
#define KNEXUS_LINE_STRAIGHT_MIN_GAIN            0.35f
#define KNEXUS_LINE_STRAIGHT_INTEGRAL_DECAY      0.50f

/* 巡线模式下允许的底盘动态范围。 */
#define KNEXUS_LINE_DRIVE_MAX_LINEAR_MPS        0.50f
#define KNEXUS_LINE_DRIVE_MAX_ANGULAR_RADPS     4.00f
#define KNEXUS_LINE_DRIVE_MAX_LINEAR_ACCEL      2.50f
#define KNEXUS_LINE_DRIVE_MAX_ANGULAR_ACCEL    18.00f

/* ======================== 6.1 H题环形赛道任务 ======================== */

/* 仅在 KNEXUS_MODE_LINE_FOLLOW 下生效；0可退回普通无限巡线。 */
#define KNEXUS_H_TASK_ENABLE                         1U

/* 赛道理论周长约 6.14m。速度需结合载球后的稳定性逐步提高。 */
#define KNEXUS_H_LINE_SPEED_MPS_DEFAULT           0.45f
#define KNEXUS_H_FINISH_APPROACH_SPEED_MPS        0.16f
#define KNEXUS_H_MAX_RUN_MS                      35000U

/* 题目要求：启动即计时显示，回到A点后冻结总时间；屏幕实物必须不大于2英寸。 */
#define KNEXUS_H_OLED_ENABLE                         1U
#define KNEXUS_H_OLED_I2C_ADDRESS                  0x3CU
#define KNEXUS_H_OLED_I2C_TIMEOUT_MS                  5U
#define KNEXUS_H_OLED_UPDATE_MS                      50U
#define KNEXUS_H_SCORE_TIME_LIMIT_MS              20000U

/* A点横向启停线识别：正常纵向黑线通常只覆盖中间1～3路。 */
#define KNEXUS_H_START_LINE_ACTIVE_MIN               3U
#define KNEXUS_H_START_LINE_CONTIGUOUS_MIN           3U
#define KNEXUS_H_START_LINE_CENTER_ACTIVE_MIN        2U
#define KNEXUS_H_START_LINE_CENTER_MASK            0x3CU /* 8路中的中间4路：CH2~CH5 */
#define KNEXUS_H_START_LINE_CHANNEL_THRESHOLD      0.55f
#define KNEXUS_H_START_CLEAR_CONFIRM_MS              80U
/* 中间三路或总计四路覆盖属于强 A 点，布防后立即确认；该时间只用于偏移的弱三路组合。 */
#define KNEXUS_H_FINISH_CONFIRM_MS                   30U
#define KNEXUS_H_FINISH_IMMEDIATE_CENTER_MIN          3U
#define KNEXUS_H_FINISH_IMMEDIATE_ACTIVE_MIN          4U

/* 防止刚起步时重复识别脚下的A线，也过滤途中偶发的宽黑区域。 */
#define KNEXUS_H_FINISH_ARM_MIN_TIME_MS            5000U
#define KNEXUS_H_FINISH_ARM_MIN_DISTANCE_M          5.00f

/*
 * 传感器识别A线后继续前进的距离。默认立即停车；实车根据“红外排到车身
 * 唯一测试点”的纵向距离和制动距离调节，正值表示越过检测线后继续前进。
 */
#define KNEXUS_H_STOP_AFTER_MARK_M_DEFAULT          0.000f

/* 行驶项目默认把球保持在中心。要求6可在线修改到任意指定位置。 */
#define KNEXUS_H_BALL_TARGET_CM_DEFAULT             0.0f
#define KNEXUS_H_REQUIRE_BALL_READY                   0U
#define KNEXUS_H_BALL_CONTROL_PERIOD_MS              10U
#define KNEXUS_H_BALL_STATIC_POSITIVE_CM             5.0f
#define KNEXUS_H_BALL_STATIC_NEGATIVE_CM            -5.0f
#define KNEXUS_H_BALL_STATIC_TIMEOUT_MS              5000U

/* ======================== 7. 路口采样模式 ======================== */

#define KNEXUS_SAMPLE_ROWS                       64U
#define KNEXUS_SAMPLE_DISTANCE_M                  0.20f
#define KNEXUS_SAMPLE_INTERVAL_M \
    (KNEXUS_SAMPLE_DISTANCE_M / (float)KNEXUS_SAMPLE_ROWS)
#define KNEXUS_SAMPLE_POSITION_TOLERANCE_M        0.02f
#define KNEXUS_SAMPLE_SPEED_SETTLED_MPS           0.15f
#define KNEXUS_SAMPLE_SETTLE_DELAY_MS            200U
#define KNEXUS_SAMPLE_TIMEOUT_MS               10000U
#define KNEXUS_SAMPLE_MAX_DUTY_DEFAULT            0.50f
#define KNEXUS_SAMPLE_POSITION_KP                 3.00f
#define KNEXUS_SAMPLE_POSITION_KI                 0.00f
#define KNEXUS_SAMPLE_POSITION_KD                 0.00f
#define KNEXUS_SAMPLE_POSITION_MAX_OUT            0.50f
#define KNEXUS_SAMPLE_HEADING_KP                   0.30f
#define KNEXUS_SAMPLE_HEADING_KI                   0.05f
#define KNEXUS_SAMPLE_HEADING_KD                   0.10f
#define KNEXUS_SAMPLE_HEADING_MAX_OUT              1.00f
#define KNEXUS_SAMPLE_OCTO_MATRIX_ID                100U
#define KNEXUS_SAMPLE_OCTO_CLASS_ID                 120U
#define KNEXUS_SAMPLE_OCTO_CONFIDENCE_ID            121U
#define KNEXUS_SAMPLE_NN_COMPLEX_CLASS                 0U
#define KNEXUS_SAMPLE_NN_COMPLEX_CONFIDENCE            0.95f

/* ======================== 8. PID 基础调参模式 ======================== */

/* KEY0 开始/重复，KEY1 立即停车；正负目标每隔一段时间自动切换。 */
#define KNEXUS_TUNE_MAX_DUTY_DEFAULT               0.30f
#define KNEXUS_TUNE_LINEAR_SPEED_MPS                0.30f
#define KNEXUS_TUNE_ANGULAR_SPEED_RADPS             1.50f
#define KNEXUS_TUNE_STEP_HOLD_MS                   2000U
#define KNEXUS_TUNE_STEP_PAUSE_MS                   500U

/* ======================== 9. 全板硬件验收模式 ======================== */

/*
 * 电机上电默认不使能；长按 KEY0 一秒后才开始循环测试，KEY1 立即停车。
 * 验收模式允许翻转底板测量，因此绕过姿态/通信安全停车，但仍保留30%硬占空比。
 */
#define KNEXUS_BOARD_TEST_MOTOR_MAX_DUTY             0.30f
#define KNEXUS_BOARD_TEST_MOTOR_SEGMENT_MS          2000U
#define KNEXUS_BOARD_TEST_CAN_PERIOD_MS              100U
#define KNEXUS_BOARD_TEST_OCTO_ENABLE                  1U
#define KNEXUS_BOARD_TEST_OCTO_PERIOD_MS             100U
#define KNEXUS_BOARD_TEST_OCTO_BASE_ID               600U
#define KNEXUS_BOARD_TEST_CAN1_ID                  0x701U
#define KNEXUS_BOARD_TEST_CAN2_ID                  0x702U

/* DJI M3508 + C620：STM32使用FDCAN2，MSPM0使用其唯一的MCAN。 */
#if defined(KNX_PLATFORM_MSPM0)
#define KNEXUS_DJI_CAN_BUS_INDEX                       0U
#else
#define KNEXUS_DJI_CAN_BUS_INDEX                       1U
#endif
#define KNEXUS_DJI_MOTOR_RX_ID                     0x201U /* C620拨码ID=1 */
#define KNEXUS_DJI_MOTOR_TX_ID                     0x200U
#define KNEXUS_DJI_TEST_SPEED_RPM                  3000.0f
#define KNEXUS_DJI_TEST_REVERSE_PERIOD_MS          3000U
#define KNEXUS_DJI_TARGET_SLEW_RPM_PER_S          12000.0f
#define KNEXUS_DJI_MAX_TARGET_RPM                  6000.0f
/* C620未接或反馈中断时仅低频探测，避免无ACK总线被1 kHz控制帧占满。 */
#define KNEXUS_DJI_FEEDBACK_TIMEOUT_MS                100U
#define KNEXUS_DJI_OFFLINE_TX_PERIOD_MS                20U
#define KNEXUS_DJI_SPEED_KP                          15.0f
#define KNEXUS_DJI_SPEED_KI                           0.001f
#define KNEXUS_DJI_SPEED_KD                           0.0f
#define KNEXUS_DJI_PID_MAX_CURRENT_CMD             12000.0f
#define KNEXUS_DJI_PID_INTEGRAL_LIMIT                500.0f

/* ======================== 9.1 丝杆与外置IMU测试 ======================== */

/* STM32: M3508/C620=FDCAN2，DM-IMU-L1=FDCAN1；MSPM0只有一路CAN，不执行该组合测试。 */
#define KNEXUS_SCREW_TEST_SPEED_RPM                  3000.0f
#define KNEXUS_SCREW_TEST_DM_CAN_BUS_INDEX              0U
#define KNEXUS_SCREW_TEST_DM_INTERVAL_MS                 1U
#define KNEXUS_SCREW_TEST_OCTO_BASE_ID                 900U

/* ======================== 10. BMI088 温控和零偏学习 ======================== */

#define KNEXUS_BMI088_HEATER_ENABLE_DEFAULT         1.0f
#define KNEXUS_BMI088_TEMP_TARGET_C_DEFAULT         40.0f
#define KNEXUS_BMI088_TEMP_KP_DEFAULT                0.020f
#define KNEXUS_BMI088_TEMP_KI_DEFAULT                0.0005f
#define KNEXUS_BMI088_HEATER_FEEDFORWARD_DEFAULT     0.12f
#define KNEXUS_BMI088_HEATER_MAX_DUTY_DEFAULT        0.30f

#define KNEXUS_IMU_EKF_ENABLE_DEFAULT                1.0f
#define KNEXUS_IMU_ZERO_DRIFT_ENABLE_DEFAULT         1.0f
#define KNEXUS_IMU_ZERO_DRIFT_GYRO_ENTER_DEFAULT     0.030f
#define KNEXUS_IMU_ZERO_DRIFT_ACCEL_ENTER_DEFAULT    0.12f
#define KNEXUS_IMU_ZERO_DRIFT_BIAS_TAU_S_DEFAULT    20.0f
#define KNEXUS_IMU_ZERO_DRIFT_BIAS_Z_DEFAULT        -0.00089f
#define KNEXUS_IMU_ZERO_DRIFT_CONFIRM_MS_DEFAULT    300U

#endif /* KNEXUS_CONFIG_H */
