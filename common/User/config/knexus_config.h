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

// #define KNEXUS_MODE_LINE_FOLLOW             /* 免校准巡线：KEY0启动，KEY1急停 */
// #define KNEXUS_MODE_INTERSECTION_SAMPLE  /* 路口采样模式：采集 64x8 灰度矩阵 */
// #define KNEXUS_MODE_PID_TUNE             /* 电机/底盘 PID 基础调参模式 */
// #define KNEXUS_MODE_USER                 /* 用户自定义模式，见 knexus_mode_user.c */
// #define KNEXUS_MODE_BOARD_TEST           /* 全板硬件验收：两种 MCU 共用 */
// #define KNEXUS_MODE_SCREW_TEST           /* 丝杆测试：KEY0按住正转，KEY1按住反转 */
// #define KNEXUS_MODE_STATIC_ROD_ANGLE       /* 底盘静止、杆角闭环：当前目标 roll=0° */
// #define KNEXUS_MODE_LINE_FOLLOW_BALL_CENTER /* 循迹归中：纵向运动补偿并预留视觉闭环 */
// #define KNEXUS_MODE_JC4310_LINK_CENTER       /* JC4310运动补偿：KEY0启动，KEY1停止 */
// #define KNEXUS_MODE_SCREW_OFFSET_CENTER      /* 丝杆视觉归中：KEY0启动偏移量PID，KEY1急停 */
#define KNEXUS_MODE_SIX_MENU                  /* 六模式菜单：模式3/4/5自动回零后按KEY0执行赛题任务 */

#if (defined(KNEXUS_MODE_LINE_FOLLOW) + \
     defined(KNEXUS_MODE_INTERSECTION_SAMPLE) + \
     defined(KNEXUS_MODE_PID_TUNE) + \
     defined(KNEXUS_MODE_USER) + \
     defined(KNEXUS_MODE_BOARD_TEST) + \
     defined(KNEXUS_MODE_SCREW_TEST) + \
     defined(KNEXUS_MODE_STATIC_ROD_ANGLE) + \
     defined(KNEXUS_MODE_LINE_FOLLOW_BALL_CENTER) + \
     defined(KNEXUS_MODE_JC4310_LINK_CENTER) + \
     defined(KNEXUS_MODE_SCREW_OFFSET_CENTER) + \
     defined(KNEXUS_MODE_SIX_MENU)) != 1
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
#define KNEXUS_DEBUG_LINE_DATA_ENABLE       0U  /* Mode3调参期间关闭巡线批量输出 */
#define KNEXUS_DEBUG_PID_DATA_ENABLE        0U  /* Mode3调参期间关闭底盘PID批量输出 */
#define KNEXUS_DEBUG_IMU_DATA_ENABLE        0U  /* Mode3调参期间关闭BMI088批量输出 */
/* 旧遥测任务会重复发送电机、底盘、BMI088、按键和灰度数据；当前关闭。 */
#define KNEXUS_DEBUG_LEGACY_TELEMETRY_ENABLE 0U

/* 当前赛题不需要路口判断：0=不创建推理任务、不运行TinyCNN；1=启用。 */
#define KNEXUS_INTERSECTION_ENABLE           0U

/*
 * 六个基础任务 + 按模式启用的小球/路口任务：
 * fast    1 kHz：电机内环、安全、蜂鸣器（最高优先级，禁止阻塞）
 * control 100/200 Hz：BMI088、里程计和底盘外环
 * track    50 Hz：新版I2C数字循迹模块采样（不运行模型）
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
#define KNEXUS_TRACK_PERIOD_MS             20U /* 新版循迹模块内部更新频率50Hz */
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

#define KNEXUS_LINE_SENSOR_REQUIRES_CALIBRATION  0U /* 新版I2C数字模块内部完成阈值处理 */
#define KNEXUS_LINE_CAL_DELAY_MS            2000U /* 黑/白面放置等待时间 */
#define KNEXUS_LINE_CAL_SAMPLES              100U /* 每个表面采样次数 */
#define KNEXUS_LINE_CAL_MIN_SPAN               40U /* 任一路量程小于此值则校准失败 */
#define KNEXUS_LINE_TRACK_FRESH_MS            100U /* 灰度数据最大允许年龄 */
#define KNEXUS_LINE_LOST_STOP_ENABLE             1U /* 1=确认脱线后自动停车 */
#define KNEXUS_LINE_LOST_STOP_CONFIRM_MS       150U /* 连续脱线确认时间，滤除瞬时空白 */
#define KNEXUS_LINE_MAX_DUTY_DEFAULT           0.80f
#define KNEXUS_LINE_BASE_SPEED_MPS_DEFAULT     0.30f

/* Kp=吸线力度；Ki=消除长期偏线；Kd=抑制快速摆动。 */
#define KNEXUS_LINE_KP_DEFAULT                 0.42f
#define KNEXUS_LINE_KI_DEFAULT                 0.03f
#define KNEXUS_LINE_KD_DEFAULT                 0.022f
#define KNEXUS_LINE_MAX_ANGULAR_RADPS_DEFAULT  4.00f
#define KNEXUS_LINE_MIN_ANGULAR_RADPS_DEFAULT  0.35f /* 克服低速转向死区 */
#define KNEXUS_LINE_INTEGRAL_LIMIT_DEFAULT     0.80f
#define KNEXUS_LINE_MIN_STRENGTH_DEFAULT       0.30f
#define KNEXUS_LINE_RECOVERY_RADPS_DEFAULT     2.00f
#define KNEXUS_LINE_ERROR_FILTER_ALPHA         0.25f
#define KNEXUS_LINE_D_FILTER_ALPHA             0.20f
/*
 * 转向滤波只承担命令整形，不再用很慢的释放系数压摆；实测慢释放会增加
 * 闭环相位滞后，把约 2.7 Hz 的摆动变成幅度更大的约 2.2 Hz 摆动。
 */
#define KNEXUS_LINE_ANGULAR_ATTACK_ALPHA       0.65f
#define KNEXUS_LINE_ANGULAR_RELEASE_ALPHA      0.65f
#define KNEXUS_LINE_ANGULAR_REVERSE_ALPHA      0.65f
#define KNEXUS_LINE_ANGULAR_SLEW_RADPS2       14.00f /* 新版二值传感器稳定，可提高转向建立速度 */
/*
 * H题赛道固定，三圈实测弯道里程位置重复性优于约2.5cm。弯道前馈因此由
 * 里程表生成，不再由正在振荡的灰度误差/curve metric生成。
 */
#define KNEXUS_LINE_CURVE_FEEDFORWARD_RADPS    0.95f
#define KNEXUS_H_CURVE_SPEED_SCALE             0.82f
#define KNEXUS_H_CURVE_STEERING_SIGN          -1.00f
#define KNEXUS_H_CURVE_RAMP_M                  0.18f
#define KNEXUS_H_CURVE1_START_M                0.03f
#define KNEXUS_H_CURVE1_END_M                  1.77f
#define KNEXUS_H_CURVE2_START_M                2.93f
#define KNEXUS_H_CURVE2_END_M                  4.68f
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
#define KNEXUS_LINE_ACCEL_LIMIT_MPS2              0.65f
#define KNEXUS_LINE_DECEL_LIMIT_MPS2              0.75f
#define KNEXUS_LINE_JERK_LIMIT_MPS3               3.50f
#define KNEXUS_LINE_SPEED_RESPONSE_S              0.26f
#define KNEXUS_LINE_MEASURED_ACCEL_FILTER_ALPHA   0.15f

/*
 * Straight-line anti-hunting zone.  Only small errors are softened; once the
 * error leaves this zone, the original cornering gains and limits apply.
 */
/*
 * 8 路传感器为偶数排列，黑线落在两颗中央探头之一时，质心误差天然约为
 * -0.5 / +0.5；原来的 0.10 死区会把这种“实际已居中”误判成需要持续转向，
 * 形成左右往复摆动。实车数据表明把中心死区扩到 0.55 后，直线修正量可
 * 降低约 85%；新的平滑过渡在 |error|>=1.10 时恢复完整 Kp/Ki/Kd 与限幅。
 */
/* 2026-07-31：直线振荡时误差在约 -0.7/+0.7 间跳变。扩宽中心增益过渡区，
 * 只压低离散探头换位造成的反复修正；|error|>=1.10 恢复完整 PID。
 * 弯道前馈、最大角速度与角速度斜率保持不变。 */
#define KNEXUS_LINE_STRAIGHT_ZONE_ERROR          1.10f
#define KNEXUS_LINE_STRAIGHT_DEADBAND_ERROR      0.55f
#define KNEXUS_LINE_STRAIGHT_MIN_GAIN            0.15f
#define KNEXUS_LINE_STRAIGHT_INTEGRAL_DECAY      0.50f

/* 巡线模式下允许的底盘动态范围。 */
#define KNEXUS_LINE_DRIVE_MAX_LINEAR_MPS        0.50f
#define KNEXUS_LINE_DRIVE_MAX_ANGULAR_RADPS     4.00f
#define KNEXUS_LINE_DRIVE_MAX_LINEAR_ACCEL      2.50f
#define KNEXUS_LINE_DRIVE_MAX_ANGULAR_ACCEL    18.00f

/* ======================== 6.1 H题环形赛道任务 ======================== */

/* 在普通巡线与循迹归中模式下生效；0可退回普通无限巡线。 */
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
#define KNEXUS_H_START_LINE_ACTIVE_MIN               4U
#define KNEXUS_H_START_LINE_CONTIGUOUS_MIN           4U
#define KNEXUS_H_START_LINE_CENTER_ACTIVE_MIN        2U
#define KNEXUS_H_START_LINE_CENTER_MASK            0x3CU /* 8路中的中间4路：CH2~CH5 */
#define KNEXUS_H_START_LINE_CHANNEL_THRESHOLD      0.55f
#define KNEXUS_H_START_CLEAR_CONFIRM_MS              80U
/* 新版模块普通循迹会短暂出现三路0x1C；A点必须同一帧至少四路亮起，
 * 且至少两路位于中间CH2~CH5。四路及以上属于强A点，立即确认。 */
#define KNEXUS_H_FINISH_CONFIRM_MS                   30U
#define KNEXUS_H_FINISH_IMMEDIATE_CENTER_MIN          4U
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
#if defined(KNEXUS_MODE_LINE_FOLLOW_BALL_CENTER)
#define KNEXUS_H_REQUIRE_BALL_READY                   1U
#else
#define KNEXUS_H_REQUIRE_BALL_READY                   0U
#endif
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
/* stop()后的零电流帧必须重发一段时间。单次FDCAN发送若恰好BUSY，C620会
 * 保持上一条电流命令，造成软件已停而电机继续加速。 */
#define KNEXUS_DJI_STOP_FLUSH_MS                       250U
#define KNEXUS_DJI_STOP_FLUSH_PERIOD_MS                  2U
#define KNEXUS_DJI_SPEED_KP                          15.0f
#define KNEXUS_DJI_SPEED_KI                           0.001f
#define KNEXUS_DJI_SPEED_KD                           0.0f
#define KNEXUS_DJI_PID_MAX_CURRENT_CMD             12000.0f
#define KNEXUS_DJI_PID_INTEGRAL_LIMIT                500.0f

/* ======================== 9.1 丝杆与外置IMU测试 ======================== */

/* STM32: M3508/C620=FDCAN2，DM-IMU-L1=FDCAN1；MSPM0只有一路CAN，不执行该组合测试。 */
#define KNEXUS_SCREW_TEST_SPEED_RPM                  1000.0f
#define KNEXUS_SCREW_TEST_DM_CAN_BUS_INDEX              0U
#define KNEXUS_SCREW_TEST_DM_INTERVAL_MS                 1U
#define KNEXUS_SCREW_TEST_OCTO_BASE_ID                 900U
/*
 * 丝杆/倾角实测极性：
 *   +rpm（KEY0） -> 丝杆下沉 -> roll增大；
 *   -rpm（KEY1） -> 丝杆上升 -> roll减小。
 * 软限位只屏蔽继续撞向限位的方向，始终允许反向离开限位。
 */
#define KNEXUS_SCREW_MOTOR_DOWN_DIRECTION                 1
#define KNEXUS_SCREW_MOTOR_UP_DIRECTION                  -1
#define KNEXUS_SCREW_TILT_LOWER_LIMIT_DEG               7.00f
#define KNEXUS_SCREW_TILT_UPPER_LIMIT_DEG             -12.00f
#define KNEXUS_SCREW_TILT_MAX_AGE_MS                     50U
/* DM-IMU-L1倒置安装：原始roll约±180°对应杆水平0°。 */
#define KNEXUS_DM_IMU_ROLL_OFFSET_DEG                  180.0f
#define KNEXUS_DM_IMU_ROLL_SIGN                         1.0f
/* 2026-07-31专用“零位.csv”复核：手动机械复位末2s的修正后roll中位数
 * 为-4.622947°，叠加采样固件偏置+4.479024°后，原始机械零位为-0.143923°。
 * 驱动输出会减去该值，因此机械水平位置输出为0°。 */
#define KNEXUS_DM_IMU_ROLL_ZERO_OFFSET_DEG             -0.143923f
/*
 * DM-IMU-L1内部姿态融合在倒置附近会产生约15s周期的roll/pitch假振荡。
 * 改用MCU侧一维互补融合：陀螺积分保留快速响应，重力角负责长期纠偏。
 */
#define KNEXUS_DM_IMU_TILT_FUSION_ENABLE                  1U
#define KNEXUS_DM_IMU_TILT_CORRECTION_TAU_S             0.50f
#define KNEXUS_DM_IMU_TILT_ACCEL_PERIOD_MS                 5U
#define KNEXUS_DM_IMU_TILT_ACCEL_NORM_MPS2               9.80665f
#define KNEXUS_DM_IMU_TILT_ACCEL_TOLERANCE_MPS2          2.00f
#define KNEXUS_DM_IMU_ROLL_GYRO_SIGN                     1.00f
#define KNEXUS_DM_IMU_PITCH_GYRO_SIGN                   -1.00f

/* ======================== 9.2 底盘静止杆角闭环 ======================== */

/*
 * 当前阶段不接上下位机，目标角固定为 0°。这些 DEFAULT 值会复制到同名
 * 普通变量，可在调试器中在线修改后观察 OctoLink 曲线。
 *
 * 已标定机械极性：+rpm -> 丝杆下沉 -> roll 增大。因此控制误差采用
 * target-roll，KNEXUS_ROD_MOTOR_SIGN 保持 +1；不要再额外取反。
 */
#define KNEXUS_ROD_TARGET_DEG_DEFAULT                    0.00f
#define KNEXUS_ROD_ANGLE_KP_DEFAULT                     70.00f /* rpm/deg */
#define KNEXUS_ROD_ANGLE_KI_DEFAULT                      2.00f /* rpm/(deg*s) */
#define KNEXUS_ROD_ANGLE_KD_DEFAULT                     16.00f /* rpm/(deg/s) */
#define KNEXUS_ROD_INTEGRAL_MAX_RPM_DEFAULT             30.00f
#define KNEXUS_ROD_INTEGRAL_ZONE_DEG_DEFAULT             3.00f
#define KNEXUS_ROD_INTEGRAL_RATE_ZONE_DPS_DEFAULT        4.00f
#define KNEXUS_ROD_MAX_RPM_DEFAULT                     500.00f
/*
 * 该斜率主要限制杆角外环命令，不是底盘的舒适性加速度。实测 800 rpm/s
 * 会让 +300 到 -300 rpm 的反向制动滞后约 0.75 s，明显放大杆角过冲。
 */
#define KNEXUS_ROD_RPM_SLEW_RPMPS_DEFAULT             8000.00f
#define KNEXUS_ROD_DEADBAND_DEG_DEFAULT                  0.30f
#define KNEXUS_ROD_RATE_DEADBAND_DPS_DEFAULT             1.00f
#define KNEXUS_ROD_MOTOR_SIGN                            1.00f
#define KNEXUS_ROD_OCTO_BASE_ID                         900U
/*
 * 0=静止杆角闭环不做角度软限位，让D项在越界回中时仍可反向制动；
 * 1=沿用 -12°/+7° 单向软限位。丝杆手动测试始终保留原有限位。
 */
#define KNEXUS_ROD_SOFT_LIMIT_ENABLE                       0U

/*
 * 杆角模式专用 M3508 速度内环。C620 电流指令满量程为 16384；6000 约为
 * 37% 满量程。原通用 Kp=15、Ki=0.001 在丝杆负载下长期零速，无法执行
 * 杆角环给出的目标转速，因此只在本模式下换用以下参数。
 */
#define KNEXUS_ROD_DJI_SPEED_KP                          35.00f
#define KNEXUS_ROD_DJI_SPEED_KI                          10.00f
#define KNEXUS_ROD_DJI_SPEED_KD                           0.00f
#define KNEXUS_ROD_DJI_SPEED_I_LIMIT                   2000.00f
#define KNEXUS_ROD_DJI_MAX_CURRENT_CMD                 6000.00f
#define KNEXUS_ROD_DJI_CURRENT_FF_CMD                  1200.00f
#define KNEXUS_ROD_DJI_CURRENT_FF_MIN_RPM                 5.00f

/* ======================== 9.3 循迹归中与运动补偿 ======================== */

/*
 * 管道沿车体前后中轴线安装。DM-IMU-L1只用于测量杆角，不再使用它的加速度计
 * 推算底盘加速度，否则丝杆自身运动会被重新反馈到目标角并形成自激振荡。
 * 2026-07-31手推“先前进、再后退”采样确认：底板BMI088的车头方向为-Y。
 * 手推测试时编码器不更新，因此直接使用BMI088；实际发车且编码器有效时再融合。
 */
#define KNEXUS_BALL_ACCEL_AXIS_DEFAULT                    1
#define KNEXUS_BALL_ACCEL_AXIS_SIGN_DEFAULT           -1.0f
#define KNEXUS_BALL_ROLL_GRAVITY_SIGN_DEFAULT           1.0f
#define KNEXUS_BALL_COMPENSATION_ENABLE_DEFAULT         1.0f
/* 1=当前人工推动小车验证补偿：KEY0解锁、KEY1停止，底盘电机始终失能。 */
#define KNEXUS_BALL_MANUAL_COMP_TEST_ENABLE                0U

/* 底盘静止且杆角速度接近0时对三个加速度轴同时取均值；100点对应约1秒。 */
#define KNEXUS_BALL_BIAS_SAMPLES                         100U
#define KNEXUS_BALL_ACCEL_LPF_HZ                         3.0f
#define KNEXUS_BALL_CHASSIS_IMU_AUTO_MAP_ENABLE            0U
#define KNEXUS_BALL_CHASSIS_IMU_MAP_MIN_ACCEL_MPS2       0.20f
#define KNEXUS_BALL_CHASSIS_IMU_MAP_SAMPLES                80U
#define KNEXUS_BALL_CHASSIS_IMU_MAP_MIN_CORRELATION       0.60f
/* 编码器有效时BMI088占80%；手推测试中编码器无效，自动使用100%BMI088。 */
#define KNEXUS_BALL_CHASSIS_IMU_BLEND_WEIGHT              0.80f
/* 巡线时以已经限加速度/限加加速度的速度规划为主，传感器只校正模型误差。 */
#define KNEXUS_BALL_COMMAND_ACCEL_WEIGHT                   0.85f
#define KNEXUS_BALL_COMMAND_MOTION_ACCEL_MPS2              0.03f
/*
 * 静止锁使用迟滞：BMI088纵向加速度连续30ms超过0.25m/s²立即解锁；
 * 回落到0.10m/s²以下且编码器也静止200ms后重新锁零。
 */
#define KNEXUS_BALL_STATIONARY_SPEED_MPS                   0.02f
#define KNEXUS_BALL_STATIONARY_ENCODER_ACCEL_MPS2          0.12f
#define KNEXUS_BALL_MOTION_UNLOCK_ACCEL_MPS2               0.25f
#define KNEXUS_BALL_MOTION_UNLOCK_HOLD_MS                    30U
#define KNEXUS_BALL_STATIONARY_IMU_ACCEL_MPS2              0.10f
#define KNEXUS_BALL_STATIONARY_HOLD_MS                     200U
/* 补偿略作超前放大，用来抵消滤波、丝杆和杆角闭环的实际滞后。 */
#define KNEXUS_BALL_ACCEL_COMPENSATION_GAIN                1.15f

/* 10 mm GCr15实心钢球，理想纯滚动系数=5/7；质量约4.09 g。 */
#define KNEXUS_BALL_DIAMETER_M                         0.010f
#define KNEXUS_BALL_DENSITY_KG_M3                    7810.0f
#define KNEXUS_BALL_ROLLING_FACTOR              (5.0f / 7.0f)
#define KNEXUS_BALL_GRAVITY_MPS2                       9.80665f
/* 等效粘性阻力 a_drag=-k*v，首轮按要求置0，后续用视觉轨迹辨识。 */
#define KNEXUS_BALL_DRAG_S_INV_DEFAULT                   0.0f

/* 比机械极限 -12°/+7° 各预留约2°制动余量。 */
#define KNEXUS_BALL_ROLL_TARGET_MIN_DEG                 -10.0f
#define KNEXUS_BALL_ROLL_TARGET_MAX_DEG                   5.0f
#define KNEXUS_BALL_ROLL_TARGET_SLEW_DPS                 35.0f
#define KNEXUS_BALL_ROD_SOFT_LIMIT_ENABLE                   1U
/* 动态目标比静止追零更需要带宽；仅循迹归中模式覆盖共享杆角默认值。 */
#define KNEXUS_BALL_ROD_ANGLE_KP                         240.0f
#define KNEXUS_BALL_ROD_ANGLE_KI                           2.0f
/* 拆除摩擦滑块后适当减小速度阻尼，让杆角更快跟随动态目标。 */
#define KNEXUS_BALL_ROD_ANGLE_KD                          14.0f
#define KNEXUS_BALL_ROD_MAX_RPM                         2500.0f
#define KNEXUS_BALL_ROD_RPM_SLEW_RPMPS                 40000.0f
/* 静止锁已建立后恢复小量目标角速度前馈；满斜率时最多增加480rpm。 */
#define KNEXUS_BALL_ROD_TARGET_RATE_FF_RPM_PER_DPS          8.0f
#define KNEXUS_BALL_DJI_TARGET_SLEW_RPMPS              40000.0f

/* 无视觉时的位置/速度仅用于观察，不参与控制；限制积分漂移以免污染曲线量程。 */
#define KNEXUS_BALL_PREDICT_VELOCITY_LIMIT_MPS            2.0f
#define KNEXUS_BALL_PREDICT_POSITION_LIMIT_M              1.0f
#define KNEXUS_BALL_OCTO_BASE_ID                          970U

/* ======================== 9.4 JC4310无刷连杆运动补偿 ======================== */

/* 沿用原JC4310协议与ID：FDCAN2，TX=0x603，RX=0x583；杆角DM-IMU-L1
 * 位于FDCAN1。上电自动进入力矩模式并持续重发0 N*m，BMI088完成静态偏置后
 * KEY0启动手推运动补偿，KEY1立即停止并恢复0 N*m。 */
#define KNEXUS_JC4310_LINK_CAN_BUS_INDEX                     1U
#define KNEXUS_JC4310_LINK_MOTOR_ID                           3U
#define KNEXUS_JC4310_LINK_TIMEOUT_MS                         8U
#define KNEXUS_JC4310_LINK_MODE_GAP_MS                      200U
/* 动态标定：正力矩 -> 电机位置增加 -> roll增大，因此杆角PD输出到电机
 * 力矩的符号为+1。 */
#define KNEXUS_JC4310_ROLL_TORQUE_SIGN                     1.00f
#define KNEXUS_JC4310_ANGLE_KP_DEFAULT                     0.055f
#define KNEXUS_JC4310_DOWN_KP_GAIN                         1.35f /* 正roll=连杆下沉 */
#define KNEXUS_JC4310_ANGLE_KI_DEFAULT                     0.004f
#define KNEXUS_JC4310_ANGLE_KD_DEFAULT                     0.0070f
#define KNEXUS_JC4310_FAR_KP_START_DEG                     3.00f
#define KNEXUS_JC4310_FAR_KP_FULL_DEG                      6.00f
#define KNEXUS_JC4310_FAR_KP_GAIN                          1.20f
#define KNEXUS_JC4310_DISTURBANCE_ERROR_DEG                 3.00f
#define KNEXUS_JC4310_DISTURBANCE_TARGET_RATE_DPS          10.00f
#define KNEXUS_JC4310_DISTURBANCE_KP_GAIN                   1.45f
#define KNEXUS_JC4310_NEAR_KD_ZONE_DEG                     3.00f
#define KNEXUS_JC4310_NEAR_KD_GAIN                         1.60f
#define KNEXUS_JC4310_D_GUARD_ERROR_DEG                    1.00f
#define KNEXUS_JC4310_D_OPPOSE_P_MAX_RATIO                 0.75f
#define KNEXUS_JC4310_REVERSAL_LOOKAHEAD_S                  0.060f
#define KNEXUS_JC4310_REVERSAL_MIN_TARGET_RATE_DPS         20.00f
#define KNEXUS_JC4310_REVERSAL_MIN_ROLL_RATE_DPS           12.00f
#define KNEXUS_JC4310_REVERSAL_CONFIRM_MS                    30U
#define KNEXUS_JC4310_REVERSAL_HOLD_MS                       80U
#define KNEXUS_JC4310_REVERSAL_D_OPPOSE_P_MAX_RATIO         1.40f
/*
 * 两级换向：预测到力矩即将反向后，先用很短的反向脉冲克服机构惯性/静摩擦，
 * 随后进入柔和回拉窗口限制力矩和斜率。脉冲只负责“换向”，不承担持续回拉，
 * 因而既缩短换向延迟，又避免高速反向一直顶到另一侧。
 */
#define KNEXUS_JC4310_REVERSE_TRIGGER_COMMAND_NM            0.080f
#define KNEXUS_JC4310_REVERSE_TRIGGER_REQUEST_NM            0.060f
#define KNEXUS_JC4310_REVERSE_TRIGGER_CONFIRM_MS               10U
#define KNEXUS_JC4310_REVERSE_KICK_MS                          45U
#define KNEXUS_JC4310_REVERSE_KICK_TORQUE_NM                 0.42f
#define KNEXUS_JC4310_REVERSE_LIMIT_KICK_TORQUE_NM           0.45f
#define KNEXUS_JC4310_REVERSE_KICK_SLEW_NMPS                150.0f
#define KNEXUS_JC4310_REVERSE_SETTLE_MS                       180U
#define KNEXUS_JC4310_REVERSE_SETTLE_MAX_TORQUE_NM           0.20f
#define KNEXUS_JC4310_REVERSE_SETTLE_SLEW_NMPS               8.0f
#define KNEXUS_JC4310_ANGLE_I_LIMIT_NM_DEFAULT             0.080f
#define KNEXUS_JC4310_MAX_TORQUE_NM_DEFAULT                0.50f
#define KNEXUS_JC4310_TORQUE_SLEW_NMPS_DEFAULT            45.00f
#define KNEXUS_JC4310_ANGLE_DEADBAND_DEG                   0.30f
#define KNEXUS_JC4310_RATE_DEADBAND_DPS                    0.80f
#define KNEXUS_JC4310_STARTUP_BLEND_MS                       700U
#define KNEXUS_JC4310_STARTUP_TARGET_SLEW_DPS              30.00f
#define KNEXUS_JC4310_STARTUP_MAX_TORQUE_NM                 0.14f
#define KNEXUS_JC4310_STARTUP_TORQUE_SLEW_NMPS              5.00f
/* Smooth static-friction compensation: no step at zero error, and it fades
 * out as soon as the rod is moving. */
#define KNEXUS_JC4310_STATIC_TORQUE_NM                      0.035f
#define KNEXUS_JC4310_STATIC_ENTER_ERROR_DEG                 0.45f
#define KNEXUS_JC4310_STATIC_FULL_ERROR_DEG                  1.00f
#define KNEXUS_JC4310_STATIC_MAX_ROLL_RATE_DPS               3.00f
#define KNEXUS_JC4310_STATIC_MAX_TARGET_RATE_DPS             8.00f
#define KNEXUS_JC4310_INTEGRAL_MAX_ERROR_DEG                3.00f
#define KNEXUS_JC4310_INTEGRAL_MAX_TARGET_RATE_DPS         10.00f
#define KNEXUS_JC4310_INTEGRAL_MAX_ROLL_RATE_DPS           15.00f
#define KNEXUS_JC4310_ACCEL_LPF_HZ                           5.00f
#define KNEXUS_JC4310_TARGET_LPF_HZ                          7.00f
#define KNEXUS_JC4310_TARGET_ROLL_SLEW_DPS                 140.00f

/* 连杆重装后的保护外边界；它只用于最后一道绝对方向拦截，不作为正常工作区。 */
#define KNEXUS_JC4310_MECHANICAL_MIN_POSITION_DEG         -42.00f
#define KNEXUS_JC4310_MECHANICAL_MAX_POSITION_DEG          45.00f
/* 2026-07-31 重装连杆后静止端位置达到约+43.6°。软件限位适度放宽，但仍在
 * 机械边界内预留3~6°制动空间，并继续使用速度预测和限速恢复，禁止大摆锤。 */
#define KNEXUS_JC4310_MOTOR_MIN_POSITION_DEG             -36.00f
#define KNEXUS_JC4310_MOTOR_MAX_POSITION_DEG              41.00f
/* 兼容旧名称；新代码统一使用MIN/MAX，避免“upper=-38”的歧义。 */
#define KNEXUS_JC4310_MOTOR_UPPER_LIMIT_DEG KNEXUS_JC4310_MOTOR_MIN_POSITION_DEG
#define KNEXUS_JC4310_MOTOR_LOWER_LIMIT_DEG KNEXUS_JC4310_MOTOR_MAX_POSITION_DEG
#define KNEXUS_JC4310_POSITION_MAX_AGE_MS                   150U
#define KNEXUS_JC4310_LIMIT_BRAKE_ZONE_DEG                 16.00f
#define KNEXUS_JC4310_LIMIT_MAX_TORQUE_NM                   0.50f
#define KNEXUS_JC4310_LIMIT_HARD_GUARD_MARGIN_DEG           3.00f
#define KNEXUS_JC4310_LIMIT_LOOKAHEAD_S                      0.080f
#define KNEXUS_JC4310_LIMIT_RETURN_MARGIN_DEG               5.00f
#define KNEXUS_JC4310_LIMIT_RECOVERY_POS_TO_VEL_GAIN        2.50f
#define KNEXUS_JC4310_LIMIT_RECOVERY_MAX_VEL_DPS           30.00f
#define KNEXUS_JC4310_LIMIT_RECOVERY_VEL_KP_NM_PER_DPS      0.0050f
#define KNEXUS_JC4310_LIMIT_VELOCITY_KD_NM_PER_DPS          0.0022f
#define KNEXUS_JC4310_LIMIT_RETURN_MIN_TORQUE_NM            0.055f
#define KNEXUS_JC4310_LIMIT_RETURN_MAX_TORQUE_NM            0.12f
#define KNEXUS_JC4310_LIMIT_RECOVERY_MAX_BRAKE_TORQUE_NM    0.50f
#define KNEXUS_JC4310_LIMIT_RECOVERY_POSITION_TOL_DEG       1.50f
#define KNEXUS_JC4310_LIMIT_RECOVERY_EXIT_VEL_DPS           8.00f
#define KNEXUS_JC4310_LIMIT_POST_SETTLE_MS                      0U
#define KNEXUS_JC4310_LIMIT_POST_SETTLE_MAX_TORQUE_NM       0.00f
#define KNEXUS_JC4310_LIMIT_POST_SETTLE_SLEW_NMPS           5.00f
#define KNEXUS_JC4310_POSITION_VELOCITY_LPF_ALPHA            0.35f
/* 这里是DM-IMU杆角目标，与JC编码器角度限位不是同一种角度。 */
#define KNEXUS_JC4310_TARGET_ROLL_MIN_DEG                 -6.00f
#define KNEXUS_JC4310_TARGET_ROLL_MAX_DEG                  6.00f
#define KNEXUS_JC4310_LINK_OCTO_BASE_ID                    1040U
#define KNEXUS_JC4310_COMP_OCTO_BASE_ID                    1200U

/* ======================== 9.5 六模式菜单 / 模式3视觉归中 ======================== */

/* 菜单：KEY0切换，KEY1确认；进入任意模式后长按KEY0一秒返回主菜单。 */
#define KNEXUS_MENU_MODE_COUNT                                6U
#define KNEXUS_MENU_EXIT_HOLD_MS                           1000U
#define KNEXUS_MENU_OLED_UPDATE_MS                          100U
#define KNEXUS_MENU_OCTO_BASE_ID                           1280U
/* Mode3只发送调参必需量；避免每20ms打包上百个Octo变量拖慢控制。 */
#define KNEXUS_MENU_MODE3_OCTO_COMPACT_ENABLE                 1U

/* 六模式菜单中的H题任务。模式3按用户指定执行-5cm -> +5cm并最终保持；
 * 模式4运行A到B；模式5以0cm为目标整圈；模式6采样任意球位后整圈保持。
 * 模式4/5/6均叠加底盘运动补偿，但球控不得改变巡线速度或启停状态。 */
#define KNEXUS_MENU_MODE3_NEGATIVE_TARGET_CM               -5.00f
#define KNEXUS_MENU_MODE3_POSITIVE_TARGET_CM                5.00f
#define KNEXUS_MENU_MODE3_REACH_TOLERANCE_CM                0.80f
#define KNEXUS_MENU_MODE3_HOLD_DEADBAND_CM                  0.45f
/* 正向折返先维持适量右移前馈，到达制动区后撤掉。 */
#define KNEXUS_MENU_MODE3_POSITIVE_APPROACH_FF_MPS2        -0.08f
/*
 * -5cm折返后先完成“单向跨零”，到达该位置前不允许速度环给出反向坡度。
 * pj1.csv中小球仍在负侧时速度已达到约0.095m/s，而旧目标速度仅0.070m/s，
 * 速度环因此提前制动并令实际杆角冲到+7.17deg，直接把球送回负侧。
 * 到+2.5cm后才解除单向约束并撤掉前馈，留下约2.5cm用于平滑制动至+5cm。
 */
#define KNEXUS_MENU_MODE3_POSITIVE_CAPTURE_START_CM          2.50f
#define KNEXUS_MENU_MODE3_POSITIVE_CAPTURE_FF_MPS2          -0.10f
/* 单向跨零阶段至少维持这一右移加速度；负值对应负roll、钢球向正偏移移动。 */
#define KNEXUS_MENU_MODE3_POSITIVE_TRANSIT_MIN_ACCEL_MPS2    0.04f
/*
 * 进入最终保持后，若球回落到目标左侧超过0.20cm，补一个较小的静态前馈；
 * 回到4.8cm以上立即撤掉，避免固定偏置把球持续推过+5cm。
 */
#define KNEXUS_MENU_MODE3_POSITIVE_HOLD_FF_ENABLE_ERROR_CM   0.20f
#define KNEXUS_MENU_MODE3_POSITIVE_HOLD_FF_MPS2             -0.06f
/*
 * 正向接近/保持阶段的快速静摩擦捕获：偏差存在但球速很低时，不再等待慢积分，
 * 连续约3帧(120fps下约24ms)即强制目标杆角达到最小值，使钢球确定开始滚动。
 * 球速建立后立即释放给速度环制动；允许目标附近有小幅往复，但目标是始终留在
 * +/-0.8cm评分带内。
 */
#define KNEXUS_MENU_MODE3_STICTION_ENTER_ERROR_CM            0.30f
#define KNEXUS_MENU_MODE3_STICTION_EXIT_ERROR_CM             0.18f
#define KNEXUS_MENU_MODE3_STICTION_MAX_RATE_MPS             0.012f
#define KNEXUS_MENU_MODE3_STICTION_RELEASE_RATE_MPS         0.025f
#define KNEXUS_MENU_MODE3_STICTION_DETECT_MS                   24U
#define KNEXUS_MENU_MODE3_STICTION_MIN_ROLL_DEG              3.80f
#define KNEXUS_MENU_MODE3_STICTION_TARGET_SLEW_DPS         140.00f
/* 2026-07-31采样：到达-5cm时球速约-10cm/s，连杆换向后继续冲到-6.46cm。
 * 用视觉速度预测130ms后的球位；进入末端2cm且预测将越过-5时立即开始反向
 * 制动。球仍靠已有惯性到达-5，但不会等越线后才让连杆换向。 */
#define KNEXUS_MENU_MODE3_NEGATIVE_BRAKE_LOOKAHEAD_S        0.20f
#define KNEXUS_MENU_MODE3_NEGATIVE_BRAKE_ARM_DISTANCE_CM    2.50f
#define KNEXUS_MENU_MODE3_TIME_LIMIT_MS                    5000U
/* 模式3专用的高阻尼位置-速度串级参数，限制大行程折返时的摆幅。 */
#define KNEXUS_MENU_MODE3_POSITION_TO_SPEED_S_INV           1.25f
#define KNEXUS_MENU_MODE3_NEAR_POSITION_TO_SPEED_S_INV      2.20f
#define KNEXUS_MENU_MODE3_VELOCITY_KP_S_INV                 5.50f
#define KNEXUS_MENU_MODE3_MAX_SPEED_MPS                     0.055f
#define KNEXUS_MENU_MODE3_MAX_ACCEL_MPS2                    0.30f
/* 0 -> -5cm单独软启动，避免起步瞬间把杆角和力矩直接推到上限。 */
#define KNEXUS_MENU_MODE3_NEGATIVE_MAX_SPEED_MPS            0.040f
#define KNEXUS_MENU_MODE3_NEGATIVE_MAX_ACCEL_MPS2           0.16f
#define KNEXUS_MENU_MODE3_NEGATIVE_TARGET_SLEW_DPS         14.00f
/* 去+5cm阶段单独提高一档，负端制动和最终保持仍使用上面的柔和参数。 */
#define KNEXUS_MENU_MODE3_POSITIVE_POSITION_TO_SPEED_S_INV  1.40f
#define KNEXUS_MENU_MODE3_POSITIVE_NEAR_POSITION_TO_SPEED_S_INV 2.50f
#define KNEXUS_MENU_MODE3_POSITIVE_VELOCITY_KP_S_INV        5.50f
#define KNEXUS_MENU_MODE3_POSITIVE_MAX_SPEED_MPS            0.070f
#define KNEXUS_MENU_MODE3_POSITIVE_MAX_ACCEL_MPS2           0.44f
#define KNEXUS_MENU_MODE3_POSITIVE_MAX_ROLL_DEG              3.60f
/* B点理论里程1.50m；多走2cm确保车体测试基准点已经真正“通过B”。 */
#define KNEXUS_MENU_MODE4_AB_DISTANCE_M                     1.52f
#define KNEXUS_MENU_MODE4_TIME_LIMIT_MS                    8000U
/*
 * 模式4/5/6保持原巡线目标速度0.45m/s，只降低纵向加速度以减小钢球扰动。
 * 对模式4的1.52m直线，理想梯形速度模型要求a约>=0.10m/s^2才能在8s内完成；
 * 取0.12并给制动留余量。该限制只约束纵向速度建立，不修改循迹转向输出。
 */
#define KNEXUS_MENU_MODE4_LINE_SPEED_MPS                    0.45f
#define KNEXUS_MENU_MOVING_ACCEL_LIMIT_MPS2                 0.12f
#define KNEXUS_MENU_MOVING_DECEL_LIMIT_MPS2                 0.18f
#define KNEXUS_MENU_MOVING_JERK_LIMIT_MPS3                  0.80f
/* 兼容旧名称，模式4/5/6实际统一使用上面的MOVING参数。 */
#define KNEXUS_MENU_MODE4_ACCEL_LIMIT_MPS2 KNEXUS_MENU_MOVING_ACCEL_LIMIT_MPS2
#define KNEXUS_MENU_MODE4_DECEL_LIMIT_MPS2 KNEXUS_MENU_MOVING_DECEL_LIMIT_MPS2
#define KNEXUS_MENU_MODE4_JERK_LIMIT_MPS3  KNEXUS_MENU_MOVING_JERK_LIMIT_MPS3
#define KNEXUS_MENU_MODE4_FINISH_MIN_SPEED_MPS              0.06f
#define KNEXUS_MENU_MODE4_FINISH_MARGIN_M                   0.025f
#define KNEXUS_MENU_MODE5_TIME_LIMIT_MS                   30000U
#define KNEXUS_MENU_MODE6_TIME_LIMIT_MS                   30000U
/* 第六问的指定位置必须留在25cm摆杆可观测范围内。 */
#define KNEXUS_MENU_MODE6_TARGET_ABS_MAX_CM                 12.50f
#define KNEXUS_MENU_BALL_ERROR_LIMIT_CM                     0.80f

/*
 * 模式3的视觉偏移控制。
 * 上位机正偏移表示小球在右侧；已标定正roll使小球向左滚，因此输入极性为+1。
 * 外环输出小球期望加速度，再按实心钢球无滑动模型
 *     a = (5/7) * g * sin(roll)
 * 换算为目标杆角。这里不使用底盘BMI088，不加入车辆运动补偿。
 */
/* 上位机偏移量单位已经固定为cm，合法范围[-12.5,+12.5]；控制器内部统一换算为m。 */
#define KNEXUS_VISION_CENTER_INPUT_SCALE                  0.010f
#define KNEXUS_VISION_CENTER_INPUT_SIGN                    1.00f
/* 上位机相机120fps，MCU端rx_count实测约70~100帧/s；OctoLink中的单变量
 * 约8Hz只是调试上报频率。控制仍改为更可控的串级结构：
 *   位置误差 -> 限幅目标球速 -> 球速误差P环 -> 期望纠偏加速度。
 * 这样远处也只允许约10cm/s，接近目标会自然减速，不再每帧把杆角打满。 */
#define KNEXUS_VISION_CENTER_DEADBAND                     0.0005f /* 0.05cm */
#define KNEXUS_VISION_CENTER_POSITION_TO_SPEED_S_INV        1.50f
#define KNEXUS_VISION_CENTER_MAX_SPEED_MPS                  0.10f
#define KNEXUS_VISION_CENTER_VELOCITY_KP_S_INV              4.00f
/*
 * 远端仍使用原参数避免大行程过冲；进入目标前最后3.5cm后，提高位置到
 * 目标球速的增益和速度环刚度，克服实测在目标前1~2cm处长期爬行的问题。
 * 到达模式3的+/-0.8cm验收带后，控制器立即进入带内保持，不再触发脱困脉冲。
 */
#define KNEXUS_VISION_CENTER_NEAR_ZONE_M                    0.035f
#define KNEXUS_VISION_CENTER_NEAR_POSITION_TO_SPEED_S_INV   2.40f
#define KNEXUS_VISION_CENTER_NEAR_VELOCITY_KP_S_INV         5.00f
#define KNEXUS_VISION_CENTER_KP_MPS2                        6.00f /* 等效Kp，仅作说明 */
#define KNEXUS_VISION_CENTER_KI_MPS3                        0.15f
#define KNEXUS_VISION_CENTER_KD_MPS2                        4.00f /* 等效Kd，仅作说明 */
#define KNEXUS_VISION_CENTER_D_LPF_HZ                       1.50f
#define KNEXUS_VISION_CENTER_INTEGRAL_ZONE_M               0.060f
#define KNEXUS_VISION_CENTER_INTEGRAL_RATE_MAX_MPS         0.20f
#define KNEXUS_VISION_CENTER_I_ACCEL_LIMIT_MPS2            0.04f
#define KNEXUS_VISION_CENTER_INTEGRAL_DECAY                 0.85f
/* 不再连续叠加脱困力。误差持续存在且球几乎不动350ms后，只给200ms短脉冲；
 * 一旦球速超过1cm/s立即撤掉，再冷却400ms，兼顾静摩擦和目标附近稳定性。 */
#define KNEXUS_VISION_CENTER_BREAKAWAY_ENTER_ERROR_M      0.0030f
#define KNEXUS_VISION_CENTER_BREAKAWAY_MAX_RATE_MPS        0.010f
#define KNEXUS_VISION_CENTER_BREAKAWAY_DETECT_MS             350U
#define KNEXUS_VISION_CENTER_BREAKAWAY_PULSE_MS              200U
#define KNEXUS_VISION_CENTER_BREAKAWAY_COOLDOWN_MS           400U
#define KNEXUS_VISION_CENTER_BREAKAWAY_ACCEL_MPS2          0.28f
#define KNEXUS_VISION_CENTER_MAX_ACCEL_MPS2                 0.42f
#define KNEXUS_VISION_CENTER_ROLLING_GAIN        (5.0f / 7.0f)
#define KNEXUS_VISION_CENTER_GRAVITY_MPS2                   9.80665f
#define KNEXUS_VISION_CENTER_GRAVITY_FEEDFORWARD_GAIN       1.00f
#define KNEXUS_VISION_CENTER_MAX_ROLL_DEG                    3.50f
#define KNEXUS_VISION_CENTER_TARGET_SLEW_DPS                70.00f
#define KNEXUS_VISION_CENTER_TIMEOUT_MS                       150U
#define KNEXUS_VISION_CENTER_INVALID_LOW_CM                -12.60f
#define KNEXUS_VISION_CENTER_INVALID_HIGH_CM                12.60f

/* ======================== 9.6 上位机偏移量丝杆归中 ======================== */

/*
 * USART2：PD5(TX)/PD6(RX)，921600-8-N-1。
 * 上位机发送8字节：AA 55 + float32小端偏移量 + CRC16/Modbus小端。
 * 上位机偏移量单位为cm，范围-12.5~+12.5；旧丝杆模式保留仅用于兼容，
 * 六模式菜单的模式3直接换算为m后驱动JC4310杆角闭环。
 */
#define KNEXUS_OFFSET_INPUT_SCALE_DEFAULT                  0.010f /* cm -> m */
#define KNEXUS_OFFSET_INPUT_SIGN_DEFAULT                    1.00f
#define KNEXUS_OFFSET_INPUT_ABS_MAX                         12.60f
#define KNEXUS_OFFSET_TIMEOUT_MS                            150U

/* 外环：偏移量 -> 目标杆角。当前0.8/0/1.3为多轮采样后的基线，所有值均有
 * 普通变量镜像，可通过OctoLink/GDB在线调整。D项带低通，避免视觉量化噪声
 * 直接形成丝杆速度尖峰。 */
#define KNEXUS_OFFSET_KP_DEFAULT                            0.80f
#define KNEXUS_OFFSET_KI_DEFAULT                            0.00f
#define KNEXUS_OFFSET_KD_DEFAULT                            1.30f
#define KNEXUS_OFFSET_D_LPF_HZ_DEFAULT                      3.00f
#define KNEXUS_OFFSET_D_LIMIT_DEG_DEFAULT                   5.00f
#define KNEXUS_OFFSET_STATIC_TILT_DEG_DEFAULT               0.60f
/* 只有偏移误差超过此值才加入静摩擦补偿；靠近中心时只保留线性P/D，
 * 避免固定补偿使钢球反复越过零点。0.35对应当前上位机原始偏移约70。 */
#define KNEXUS_OFFSET_STATIC_TILT_ENTER_ERROR_DEFAULT       0.60f
#define KNEXUS_OFFSET_STATIC_TILT_FULL_ERROR_DEFAULT        1.20f
/* 钢球已经明显运动时撤掉静摩擦补偿，只留下P/D进行提前制动。单位为缩放后
 * 偏移误差每秒；0.60约等于当前上位机原始偏移120/s。 */
#define KNEXUS_OFFSET_STATIC_TILT_MAX_VELOCITY_DEFAULT      0.60f
/* 近零区额外提高比例刚度，远端保持原Kp以免加剧大行程振荡。 */
#define KNEXUS_OFFSET_NEAR_KP_BOOST_DEFAULT                  2.00f
#define KNEXUS_OFFSET_NEAR_KP_ZONE_DEFAULT                   0.50f
/* 防卡滞采用“观察 -> 破静摩擦 -> 冷却”状态机。偏移连续300ms没有变化
 * 0.04（原始偏移约8）才累加恢复角；检测到真实位移立即清零，并冷却500ms。
 * 恢复角上限不再固定为3°：误差从0.10增至0.50时，上限由0线性增至3°。
 * 越接近零点推力越小，避免小偏差被防卡滞环踢成大行程；远端仍可使用完整
 * 3°迅速越过静摩擦。 */
#define KNEXUS_OFFSET_HOLD_ENTER_ERROR_DEFAULT                0.10f
#define KNEXUS_OFFSET_HOLD_RELEASE_DELTA_DEFAULT              0.04f
#define KNEXUS_OFFSET_HOLD_DETECT_MS                           300U
#define KNEXUS_OFFSET_HOLD_COOLDOWN_MS                         500U
#define KNEXUS_OFFSET_HOLD_RAMP_DPS_DEFAULT                  3.00f
#define KNEXUS_OFFSET_HOLD_LIMIT_DEG_DEFAULT                 3.00f
#define KNEXUS_OFFSET_HOLD_FULL_ERROR_DEFAULT                0.50f
#define KNEXUS_OFFSET_INTEGRAL_LIMIT_DEG_DEFAULT            2.00f
#define KNEXUS_OFFSET_INTEGRAL_ZONE_DEFAULT                 5.00f
#define KNEXUS_OFFSET_DEADBAND_DEFAULT                      0.025f

/* 目标杆角先限制在机械极限内侧，再由knx_rod_runtime执行-12°/+7°单向
 * 硬保护：触限后只禁止继续撞限位，始终允许反向退出。 */
#define KNEXUS_OFFSET_TARGET_ROLL_MIN_DEG                   -5.00f
#define KNEXUS_OFFSET_TARGET_ROLL_MAX_DEG                    5.00f
#define KNEXUS_OFFSET_TARGET_ROLL_SLEW_DPS                  35.00f
#define KNEXUS_OFFSET_ROD_SOFT_LIMIT_ENABLE                    1U
#define KNEXUS_OFFSET_ROD_FAULT_HOLD_MS                     100U

/* 上位机识别失败时实测会固定输出约-700.5，缩放后即-3.5025。它不是钢球
 * 真实位置，禁止送入PID。小于该阈值的负边缘值视为无效；失效期间杆角以
 * 较慢速度回到0°，识别恢复后的第一帧重新初始化D项，防止产生速度尖峰。 */
#define KNEXUS_OFFSET_VISION_INVALID_LOW_RAW              -650.00f
#define KNEXUS_OFFSET_VISION_INVALID_HIGH_RAW            10000.00f
#define KNEXUS_OFFSET_INVALID_LEVEL_SLEW_DPS                15.00f
/* 短时丢识别通常发生在钢球靠近画面边缘。先沿最后一次有效控制方向保持一个
 * 受限恢复倾角，帮助钢球重新进入视野；超过时间后仍回0°，避免盲控。 */
#define KNEXUS_OFFSET_INVALID_RECOVERY_HOLD_MS              800U
#define KNEXUS_OFFSET_INVALID_RECOVERY_MAX_DEG              2.50f

/* 复用已标定的DM-IMU-L1杆角内环和3508速度内环，但第一版适当降低最大速度。 */
#define KNEXUS_OFFSET_ROD_ANGLE_KP                         230.00f
#define KNEXUS_OFFSET_ROD_ANGLE_KI                           1.00f
#define KNEXUS_OFFSET_ROD_ANGLE_KD                          52.00f
#define KNEXUS_OFFSET_ROD_MAX_RPM                         1800.00f
#define KNEXUS_OFFSET_ROD_RPM_SLEW_RPMPS                 20000.00f
#define KNEXUS_OFFSET_DJI_TARGET_SLEW_RPMPS              20000.00f
#define KNEXUS_OFFSET_ROD_DEADBAND_DEG                      0.15f
#define KNEXUS_OFFSET_ROD_RATE_DEADBAND_DPS                 0.60f
/* 归中模式不再复用高摩擦丝杆标定时的激进速度PI和1200电流前馈。当前滑块
 * 摩擦已降低，较小前馈可以避免低速换向时电流在正负限幅间来回撞击。 */
#define KNEXUS_OFFSET_DJI_SPEED_KP                          30.00f
#define KNEXUS_OFFSET_DJI_SPEED_KI                           6.00f
#define KNEXUS_OFFSET_DJI_SPEED_KD                           0.00f
#define KNEXUS_OFFSET_DJI_SPEED_I_LIMIT                   1200.00f
#define KNEXUS_OFFSET_DJI_MAX_CURRENT_CMD                 5500.00f
#define KNEXUS_OFFSET_DJI_CURRENT_FF_CMD                   500.00f
#define KNEXUS_OFFSET_DJI_CURRENT_FF_MIN_RPM                40.00f
#define KNEXUS_OFFSET_OCTO_BASE_ID                         1080U

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
