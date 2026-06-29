# Knexus-ST 项目架构文档

> **项目**: Knexus 自平衡机器人 STM32 固件
> **MCU**: STM32H743IIT6 (Cortex-M7, 480 MHz, FPU)
> **RTOS**: FreeRTOS (CMSIS-RTOS V2 封装)
> **构建**: CMake + arm-none-eabi-gcc
> **最后更新**: 2026-06-05

---

## 目录

1. [项目总览](#1-项目总览)
2. [目录结构](#2-目录结构)
3. [分层架构](#3-分层架构)
4. [启动流程](#4-启动流程)
5. [FreeRTOS 任务调度](#5-freertos-任务调度)
6. [Platform 抽象层](#6-platform-抽象层)
7. [Board 层](#7-board-层)
8. [Driver 层 (drv)](#8-driver-层-drv)
9. [Module 层 (modules)](#9-module-层-modules)
10. [App 层](#10-app-层)
11. [Task 层](#11-task-层)
12. [Config 层](#12-config-层)
13. [Algorithm 层](#13-algorithm-层)
14. [数据流图](#14-数据流图)
15. [控制回路详解](#15-控制回路详解)
16. [线程安全与关键区](#16-线程安全与关键区)
17. [硬件引脚映射](#17-硬件引脚映射)
18. [已知问题与 TODO](#18-已知问题与-todo)

---

## 1. 项目总览

Knexus-ST 是一个两轮自平衡机器人的嵌入式固件项目。核心功能包括:

- **姿态估计**: BMI088 IMU + QuaternionEKF 四元数扩展卡尔曼滤波
- **平衡控制**: LQR / Cascade PID / Cascade PWM 三种控制器可选
- **电机驱动**: DRV8701E H 桥驱动 + ADC 电流采样 + PID 电流环
- **编码器测速**: 左轮 TIM1 四倍频 / 右轮 LPTIM1 两倍频
- **循迹传感**: 8 通道模拟 MUX + ADC2
- **安全监控**: 过流 / 过倾 / 过速 / IMU 超时 故障检测
- **调试输出**: OctoLink Binary V1 协议 (UART1)

### 关键参数

| 参数             | 值                       |
|------------------|--------------------------|
| SYSCLK           | 480 MHz                  |
| AHB              | 240 MHz                  |
| APB1/APB2        | 120 MHz                  |
| FreeRTOS tick    | 1 kHz (configTICK_RATE_HZ) |
| 控制任务周期     | 1 ms (Ctrl task)         |
| App 任务周期     | 1 ms (App task)          |
| 平衡控制分频     | 2 ms (每 2 次 ctrl tick) |
| 遥测输出周期     | 20 ms (50 Hz)            |

---

## 2. 目录结构

```
knexus_st/
├── CMakeLists.txt                 # 主构建脚本 (从模板自动生成)
├── STM32H743IITX_FLASH.ld        # 链接脚本 (FLASH 区域)
│
├── Core/                          # === CubeMX 自动生成代码 ===
│   ├── Inc/                       # 外设头文件
│   │   ├── main.h                 #   主程序头文件
│   │   ├── FreeRTOSConfig.h       #   FreeRTOS 配置
│   │   ├── adc.h / dma.h / gpio.h #   外设句柄声明
│   │   ├── spi.h / tim.h / usart.h
│   │   └── lptim.h               #   LPTIM1 (右轮编码器)
│   └── Src/                       # 外设初始化实现
│       ├── main.c                 #   ★ 入口: MPU→HAL→Clock→外设→RTOS
│       ├── freertos.c             #   ★ MX_FREERTOS_Init→knx_tasks_init()
│       ├── adc.c / dma.c / gpio.c #   CubeMX 生成
│       └── ...                    #   stm32h7xx_it.c / hal_msp.c 等
│
├── Drivers/                       # === ST 官方驱动 ===
│   ├── CMSIS/                     #   ARM CMSIS 核心 + DSP 头文件
│   └── STM32H7xx_HAL_Driver/     #   STM32H7 HAL 库
│
├── Middlewares/                   # === 中间件 ===
│   ├── Third_Party/FreeRTOS/     #   FreeRTOS 内核 (heap_4, ARM_CM4F port)
│   └── ST/ARM/DSP/              #   CMSIS-DSP 静态库
│
├── User/                         # === 用户代码 (分层架构) ===
│   ├── platform/                 # ★ 第 1 层: 硬件抽象层
│   │   ├── include/              #   抽象接口 (knx_gpio/pwm/spi/adc/...)
│   │   └── stm32h743/            #   STM32H743 平台实现
│   ├── board/                    # ★ 第 2 层: 板级配置
│   │   └── knexus_stm32h743/     #   引脚实例化 + AttachPorts
│   ├── drv/                      # ★ 第 3 层: 外设驱动
│   │   ├── bmi088/               #     BMI088 IMU SPI 驱动
│   │   ├── drv8701e/             #     DRV8701E 电机驱动
│   │   ├── encoder/              #     编码器驱动
│   │   ├── track_sensor/         #     循迹传感器
│   │   ├── octolinker/           #     OctoLink 调试协议
│   │   └── vofa/                 #     VOFA+ (备用)
│   ├── modules/                  # ★ 第 4 层: 功能模块
│   │   ├── motor/                #     电机模块
│   │   ├── imu/                  #     IMU 模块
│   │   ├── ctrl/                 #     控制模块 (balance + lqr)
│   │   ├── safety/               #     安全监控
│   │   ├── sys/                  #     系统状态机
│   │   └── telemetry/            #     遥测模块
│   ├── app/                      # ★ 第 5 层: 应用层
│   │   ├── knx_app.c             #     模块编排 + 时间片调度
│   │   └── test_modes/           #     硬件测试模式
│   ├── tasks/                    # ★ 第 6 层: FreeRTOS 任务
│   │   ├── ctrl/knx_ctrl_task.c  #     1ms 控制任务
│   │   └── app/knx_app_task.c    #     1ms 应用任务
│   ├── config/                   # 配置层
│   ├── algorithm/                # 第三方算法 (EKF/PID/Kalman/...)
│   └── util/                     # 通用工具
│
├── build/                        # 构建输出
├── docs/                         # 文档
└── knexus_st.ioc                 # CubeMX 工程
```

---

## 3. 分层架构

项目采用 **6 层架构**，依赖方向严格自上而下:

```
┌─────────────────────────────────────────────────────────┐
│                    Task Layer (tasks/)                    │
│          FreeRTOS 任务封装, 周期性调度入口               │
├─────────────────────────────────────────────────────────┤
│                    App Layer (app/)                       │
│          模块编排 + 时间片调度 + 测试模式                 │
├─────────────────────────────────────────────────────────┤
│                 Module Layer (modules/)                   │
│   motor │ imu │ ctrl(balance+lqr) │ safety │ sys │ tel  │
├─────────────────────────────────────────────────────────┤
│                 Driver Layer (drv/)                       │
│   bmi088 │ drv8701e │ encoder │ track_sensor │ octolink  │
├─────────────────────────────────────────────────────────┤
│                  Board Layer (board/)                     │
│            引脚实例化 + AttachPorts + 驱动编排             │
├─────────────────────────────────────────────────────────┤
│               Platform Layer (platform/)                  │
│      knx_gpio │ knx_pwm │ knx_spi │ knx_adc │ knx_time  │
├─────────────────────────────────────────────────────────┤
│              HAL / CMSIS / FreeRTOS (不修改)              │
└─────────────────────────────────────────────────────────┘
```

### 依赖规则

| 层       | 依赖                                    | 不得依赖        |
|----------|-----------------------------------------|----------------|
| Task     | App, CMSIS-RTOS                         | Driver, Board  |
| App      | Module, Board                           | HAL            |
| Module   | Driver, Config, Platform(公共头文件)     | Board, HAL     |
| Driver   | Platform(公共头文件), Config             | Board, HAL*    |
| Board    | Driver, Platform, HAL                   | Module, App    |
| Platform | HAL                                     | 以上所有层      |

> *注: 部分旧驱动 (octolinker, vofa) 仍直接引用 `usart.h`, 未来应迁移到
> platform UART 抽象层。

---

## 4. 启动流程

### 4.1 复位向量 → main()

```
Reset_Handler
  └→ SystemInit()                    // system_stm32h7xx.c
  └→ __libc_init_array()             // 全局初始化
  └→ main()                          // Core/Src/main.c
```

### 4.2 main() 执行序列

```
main()
  │
  ├── 1. MPU_Config()                // MPU Region 0: 4GB NO_ACCESS
  ├── 2. HAL_Init()                  // HAL 内核初始化
  ├── 3. SystemClock_Config()        // ★ HSE→PLL1→480MHz
  ├── 4. PeriphCommonClock_Config()  // ADC 时钟: PLL2→50MHz
  │
  ├── 5. MX_xxx_Init()              // CubeMX 外设初始化 (共 9 个)
  │      ├── MX_GPIO_Init()         // GPIO
  │      ├── MX_DMA_Init()          // DMA
  │      ├── MX_USART1_UART_Init()  // UART1 (调试)
  │      ├── MX_SPI1_Init()         // SPI1 (BMI088)
  │      ├── MX_TIM15_Init()        // TIM15 CH1 (加热器 PWM)
  │      ├── MX_TIM2_Init()         // TIM2 CH1/CH3 (电机 PWM)
  │      ├── MX_LPTIM1_Init()       // LPTIM1 (右编码器)
  │      ├── MX_TIM1_Init()         // TIM1 (左编码器)
  │      ├── MX_ADC1_Init()         // ADC1 (电流采样)
  │      └── MX_ADC2_Init()         // ADC2 (循迹传感器)
  │
  ├── 6. osKernelInitialize()        // FreeRTOS 内核初始化
  ├── 7. MX_FREERTOS_Init()          // ★ 创建任务
  │      └── knx_tasks_init()
  │           ├── knx_app_task_init()    // 创建 knx_app 任务
  │           └── knx_ctrl_task_init()   // 创建 knx_ctrl 任务
  ├── 8. osKernelStart()             // ★ 启动调度器 (永不返回)
  └── 9. while(1) {}                 // 不可达代码 (安全兜底)
```

### 4.3 任务内初始化序列

**App 任务** (优先级 Normal) 先启动, 执行 `knx_app_init()`:

```
knx_app_init()
  ├── knx_board_init()           // 板级: AttachPorts + Init
  │    ├── Octolinker_Init()     //   OctoLink UART
  │    ├── BMI088_AttachHeater() //   绑定加热器 PWM
  │    ├── BMI088_Init()         //   BMI088 SPI + 校准
  │    ├── DRV8701E_AttachPorts()//   绑定电机端口
  │    ├── DRV8701E_Init()       //   电机驱动
  │    ├── Encoder_AttachPorts() //   绑定编码器端口
  │    ├── Encoder_Init()        //   编码器启动
  │    ├── TrackSensor_AttachPorts()
  │    └── TrackSensor_Init()
  ├── knx_sys_init()             // 系统状态机 → IMU_WARMUP
  ├── knx_motor_init()           // 电机: 电流采样 + 零点校准
  ├── knx_imu_init()             // IMU: 首次读取 + EKF 初始化
  ├── knx_balance_init()         // 平衡: 重置积分器
  ├── knx_telemetry_init()       // 遥测
  ├── knx_safety_init()          // 安全监控
  ├── knx_test_mode_init()       // 测试模式 (如启用)
  ├── knx_board_post_init()      // 后初始化钩子
  └── s_app_initialized = true   // ★ 通知 Ctrl 任务
```

**Ctrl 任务** (优先级 AboveNormal) 等待初始化完成后进入主循环:

```
knx_ctrl_task_entry()
  ├── while (!knx_app_is_initialized()) osDelay(1)
  │
  └── for(;;) osDelayUntil(1ms)
       ├── [每 2ms] knx_imu_update()      // BMI088 + EKF
       ├── [每 2ms] knx_balance_update()   // 平衡控制
       └── [每 1ms] knx_motor_update()     // 电流环 + 输出
```

### 4.4 启动时序图

```
时间 ──────────────────────────────────────────────►

[Reset]  [main: 外设初始化]  [osKernelStart]
  │              │                  │
  ▼              ▼                  ▼
  ┌──────────────┬─────────────┬────┬──────────────────────┐
  │  硬件初始化   │ CubeMX 外设  │ RTOS│    FreeRTOS 任务     │
  │  ~2ms        │  ~5ms        │启动│                      │
  └──────────────┴─────────────┴────┴──────────────────────┘
                                              │
                                  ┌───────────┼───────────┐
                                  ▼                       ▼
                            App Task                Ctrl Task
                            (Normal)             (AboveNormal)
                                  │                       │
                            knx_app_init()         等待 initialized
                            ~50ms (含 BMI088 校准)         │
                                  │                       │
                            s_app_initialized ─────────►  │
                                  │                  主循环开始
                            主循环 (1ms)            主循环 (1ms)
```

---

## 5. FreeRTOS 任务调度

### 5.1 任务列表

| 任务名       | 优先级            | 栈大小 | 周期   | 功能                        |
|-------------|-------------------|--------|--------|-----------------------------|
| `knx_app`   | osPriorityNormal  | 4096 B | 1 ms   | sys/safety/telemetry/LED    |
| `knx_ctrl`  | osPriorityAboveNormal | 4096 B | 1 ms | IMU/balance/motor 控制      |

### 5.2 优先级设计

```
osPriorityAboveNormal (25) ─── knx_ctrl: 实时控制回路
                                ├── 必须高优先级保证控制周期精确
                                └── 使用 osDelayUntil() 精确定时

osPriorityNormal (24)     ─── knx_app: 非实时监控
                                ├── 系统状态机 (5ms)
                                ├── 安全检测 (1ms)
                                ├── 遥测输出 (20ms)
                                └── LED 闪烁 (100/500ms)
```

### 5.3 任务间同步机制

```
knx_ctrl_task                          knx_app_task
      │                                      │
      │  等待 s_app_initialized ──────────►  设置 s_app_initialized
      │  (轮询 osDelay(1))                  (knx_app_init 完成后)
      │                                      │
      ├── knx_imu_update() ──┐               │
      │                      │ taskENTER_    │
      │                      │ CRITICAL      │
      │                      ▼               │
      │              s_imu_data (全局)        │
      │                      │               │
      ├── knx_balance_update()               │
      │    ├── knx_imu_snapshot() ◄── CritSec──► knx_imu_snapshot()
      │    └── knx_motor_snapshot() ◄─ CritSec──► knx_motor_snapshot()
      │                                      │
      ├── knx_motor_update()                 ├── knx_safety_update()
      │    └── sync_public_state()           ├── knx_telemetry_update()
      │                                      └── knx_sys_update()
```

**共享数据保护**:
- `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()` 保护所有跨任务共享变量
- 关键区极短 (仅结构体拷贝), 不影响中断延迟

---

## 6. Platform 抽象层

### 6.1 设计目标

Platform 层将 STM32 HAL 细节封装为统一的 `knx_xxx` 接口。
更换 MCU 时只需替换 `platform/stm32h743/` 下的实现文件。

### 6.2 抽象接口一览

| 接口头文件     | 数据类型             | 核心 API                              |
|---------------|---------------------|--------------------------------------|
| `knx_gpio.h`  | `knx_gpio_t`        | write / read / toggle                |
| `knx_pwm.h`   | `knx_pwm_channel_t` | start / stop / set_duty              |
| `knx_spi.h`   | `knx_spi_t`         | transmit / receive / cs_low / cs_high|
| `knx_adc.h`   | `knx_adc_channel_t` | read_raw / start_dma_pair / read_dma_pair |
| `knx_encoder.h`| `knx_encoder_port_t`| start / read_raw                     |
| `knx_time.h`  | (无结构体)           | millis / micros / delay_ms           |
| `knx_uart.h`  | `knx_uart_t`        | init / transmit / receive (TODO)     |
| `knx_port.h`  | (无结构体)           | enter_critical / exit_critical / reset|

### 6.3 数据类型设计模式

所有 platform 数据类型使用 `void*` 指针存储平台句柄:

```c
typedef struct {
    void       *handle;   // SPI_HandleTypeDef* (STM32)
    knx_gpio_t  cs_gpio;  // 片选 GPIO (port + pin)
} knx_spi_t;

typedef struct {
    void     *timer;     // TIM_HandleTypeDef*
    uint32_t  channel;   // TIM_CHANNEL_1 等
    uint32_t  arr;       // 自动重装值
} knx_pwm_channel_t;
```

Board 层在编译时将具体类型强制转换为 `void*`:

```c
static knx_spi_t bmi088_accel_spi = {
    .handle  = &hspi1,
    .cs_gpio = { .port = GPIOH, .pin = GPIO_PIN_8 },
};
```

---

## 7. Board 层

### 7.1 职责

Board 层是 **硬件绑定层**, 负责:
1. 实例化所有平台抽象数据结构 (静态分配, 无动态内存)
2. 按正确顺序调用各驱动的 `AttachPorts()` + `Init()`
3. 提供 getter 函数供上层访问板级资源

### 7.2 硬件资源实例化

```
knx_board.c 静态实例:
  ├── board_debug_led        // GPIO: PH7
  ├── bmi088_accel_spi       // SPI: hspi1, CS=PH8
  ├── bmi088_gyro_spi        // SPI: hspi1, CS=PI3
  ├── board_octolinker       // OctoLink: huart1
  ├── drv_left_port          // Motor: TIM2_CH1, PHASE=PB0, ADC1
  ├── drv_right_port         // Motor: TIM2_CH3, PHASE=PB11, ADC1
  ├── track_port             // Track: MUX PB4/PB5/PB7, ADC2
  ├── enc_left_port          // Encoder: TIM1, 4x
  ├── enc_right_port         // Encoder: LPTIM1, 2x
  └── bmi088_heater_pwm      // Heater: TIM15_CH1, PE5
```

### 7.3 初始化顺序

```c
knx_status_t knx_board_init(void)
{
    Octolinker_Init(&board_octolinker, &huart1);  // 1. 调试输出
    BMI088_AttachHeater(&bmi088_heater_pwm);       // 2. IMU
    BMI088_Init(&bmi088_accel_spi, &bmi088_gyro_spi);
    DRV8701E_AttachPorts(&drv_left_port, &drv_right_port); // 3. 电机
    DRV8701E_Init();
    Encoder_AttachPorts(&enc_left_port, &enc_right_port);  // 4. 编码器
    Encoder_Init();
    TrackSensor_AttachPorts(&track_port);          // 5. 循迹传感器
    TrackSensor_Init();
    return KNX_OK;
}
```

---

## 8. Driver 层 (drv)

### 8.1 BMI088 (IMU)

**文件**: `drv/bmi088/bmi088.h`, `bmi088.c`

**职责**: BMI088 六轴 IMU 的 SPI 读写、校准、温度控制

**关键数据**: `DM_IMU_Data_t imu_data` (全局)

**数据流**:
```
SPI1 ──► BMI088_Read() ──► imu_data (全局)
  │         ├── accel[3]  (m/s²)
  │         ├── gyro[3]   (rad/s, 已去偏)
  │         ├── temperature (°C)
  │         └── roll/pitch/yaw (度, Mahony 备用)
  │
  └── 温控: bmi088_heater_duty → TIM15_CH1 PWM
             PI 控制, 目标温度 40°C
```

**校准**: 上电后连续读取 N 帧陀螺仪, 计算零偏, 鲁棒检查后生效。

### 8.2 DRV8701E (电机驱动)

**文件**: `drv/drv8701e/drv8701e.h`, `drv8701e.c`

**硬件模式**: PHASE/EN
- EN/PWM: TIM2_CH1 (左) / TIM2_CH3 (右) → 速度
- PHASE: GPIO PB0 (左) / PB11 (右) → 方向

**关键结构体**:
```c
DRV8701E_Motor_t motor_left, motor_right;     // 电机句柄
DRV8701E_CurrentSample_t drv8701e_current;     // 电流采样
DRV8701E_VelocityCtrl_t vc_left, vc_right;     // 速度环 PID
DRV8701E_CurrentCtrl_t cc_left, cc_right;      // 电流环 PI
```

**电流采样**:
```
ADC1 (DMA 循环) → Rank1: PC0 (右) + Rank2: PA4 (左)
  → DRV8701E_Current_Poll() → 电压→电流换算 → 一阶 LPF (10Hz)
```

**控制模式**:
```
knx_sys_get_mode()
  ├── IDLE           → DRV8701E_StopAll()
  ├── SPEED          → VelocityControl()
  ├── CURRENT_TEST   → CurrentControl()
  ├── TORQUE         → torque→current → CurrentControl()
  └── BALANCE
       ├── CASCADE_PWM → SetDutyNorm() (直通)
       └── 其他        → torque→current → CurrentControl()
```

### 8.3 Encoder (编码器)

| 编码器 | 定时器  | 引脚      | 倍频 | 每圈计数 |
|--------|---------|-----------|------|---------|
| 左轮   | TIM1    | PA8/PA9   | 4x   | 1456    |
| 右轮   | LPTIM1  | PG12/PG11 | 2x   | 728     |

**速度解算**: delta_count / counts_per_rev / dt → rpm → mps → LPF (10Hz)

### 8.4 TrackSensor (循迹传感器)

8 通道模拟 MUX (PB4/PB5/PB7 地址线) + ADC2 (PC5), 逐通道扫描。

### 8.5 Octolinker (调试协议)

OctoLink Binary V1: `[SOF 0xA5 0x5A][version][varId:u16][type][shape][payload][crc8]`

API: `Octolinker_SendF32()` / `SendI32()` / `SendU32()` / `PrintLine()` 等

---

## 9. Module 层 (modules)

### 9.1 knx_motor — 电机模块

**状态结构体**:
```c
typedef struct {
    float target_speed;      // 目标速度 (m/s)
    float target_current;    // 目标电流 (A)
    float target_torque;     // 目标扭矩 (N·m)
    float target_duty;       // 目标占空比 (-1.0~1.0)
    float current_speed;     // 实际速度 (m/s)
    float current_filtered;  // 实际电流 (A)
    float torque_estimated;  // 估算扭矩 (N·m)
    float duty;              // 实际占空比
} knx_motor_state_t;
```

**扭矩-电流换算**:
```
torque_gain = 0.00984 * 28.0 * 0.65 = 0.1791 N·m/A
torque_to_current: I = T / 0.1791
current_to_torque: T = I * 0.1791
```

**update() 流程** (每 1ms):
```
knx_motor_update()
  ├── 读取 target (Critical Section)
  ├── DRV8701E_Current_Poll()     // ADC 电流采样
  ├── Encoder_CalcSpeed() x2      // 左右轮速度解算
  ├── 根据 mode 分支控制
  └── sync_public_state()         // 更新公共状态 (Critical Section)
```

### 9.2 knx_imu — IMU 模块

**数据流**:
```
BMI088_Read() → imu_data.accel/gyro
  → IMU_QuaternionEKF_Update()
    → QEKF_INS.Roll/Pitch/Yaw/q
      → copy_ekf_output_to_public()
        → s_imu_data (模块公共数据)
```

**EKF 参数** (运行时可调):
```c
float knx_imu_ekf_q1      = 10.0f;      // 过程噪声 (陀螺仪)
float knx_imu_ekf_q2      = 0.001f;     // 过程噪声 (零偏)
float knx_imu_ekf_r       = 1000000.0f; // 测量噪声 (加速度计)
float knx_imu_ekf_lambda  = 1.0f;       // 渐消因子
float knx_imu_ekf_acc_lpf = 0.0085f;    // 加速度计 LPF
```

### 9.3 knx_balance — 平衡控制

**控制器选择** (编译时):
```c
#define KNX_BALANCE_CONTROLLER  KNX_BALANCE_CONTROLLER_CASCADE_PWM
```

**控制器 A: LQR**
```
u = -(K[0]*pos + K[1]*vel + K[2]*theta + K[3]*dtheta)
K = [0.01, 2.5, 6.5, 0.02], 输出 torque (N·m)
```

**控制器 B: Cascade PID** (串级 PID → torque)

**控制器 C: Cascade PWM** (当前默认, 串级 → duty):
```
速度环: theta_target = -Kp_vel * velocity + ∫Ki_vel
角度环: duty = Kp_ang*error + Ki_ang*∫error + Kd_ang*d(error)/dt
```

**通用功能**: 死区补偿, 增益调度, 斜率限制, 位置积分 anti-windup

### 9.4 knx_lqr — LQR 计算

纯计算模块: `u = -(K·x)`, 带扭矩限幅。

### 9.5 knx_safety — 安全监控

**故障码**:
```c
KNX_FAULT_OVERCURRENT  (1<<0)  // 电流 > 4.2A
KNX_FAULT_IMU_FAIL     (1<<1)  // IMU 未就绪/超时
KNX_FAULT_TILT         (1<<4)  // 倾角 > 65°
KNX_FAULT_OVERSPEED    (1<<5)  // 速度 > 8.0 m/s
```

每 1ms 检测, 故障时报告给 knx_sys → 进入 FAULT 状态 → 电机停止。

### 9.6 knx_sys — 系统状态机

```
BOOT → IMU_WARMUP → READY → RUNNING
                ↑                │
                └── FAULT ◄──────┘ (故障时)
```

**运行模式**: IDLE / CURRENT_TEST / TORQUE / SPEED / BALANCE

### 9.7 knx_telemetry — 遥测

通过 OctoLink 每 20ms 输出调试数据:
- 通道 1-4: 电机电流
- 通道 20-25: IMU 姿态 + 温度
- 通道 30-49: 平衡调试 (角度/角速度/速度/扭矩/LQR 项)

---

## 10. App Layer

### 10.1 knx_app - Application Orchestration

Main loop (non-blocking time-sliced scheduling):

    knx_app_loop()
      +-- [5ms]  knx_sys_update()        // System state machine
      +-- [1ms]  knx_safety_update()     // Safety monitoring
      +-- [100/500ms] LED toggle         // Heartbeat (fast on fault)
      +-- [20ms] knx_telemetry_update()  // Telemetry (normal mode only)
      +-- knx_test_mode_loop()           // Test mode (if enabled)

### 10.2 Test Modes

Compile-time selection via knx_debug_config.h. Options: DRV8701E / BMI088 / ENCODER.
When active: Ctrl task balance/motor control is skipped, test mode takes over.

---

## 11. Task Layer

### 11.1 Task Creation

    knx_tasks_init()
      +-- knx_app_task_init()    // Create knx_app (Normal priority)
      +-- knx_ctrl_task_init()   // Create knx_ctrl (AboveNormal priority)

### 11.2 Ctrl Task Key Code

    static void knx_ctrl_task_entry(void *argument)
    {
        while (!knx_app_is_initialized()) osDelay(1U);
        uint32_t next_wake = osKernelGetTickCount();
        uint32_t balance_div = 0U;
        for (;;) {
            balance_div++;
            if (balance_div >= 2) {         // Every 2ms
                balance_div = 0;
                knx_imu_update();           // IMU + EKF
                knx_balance_update(0.002f); // Balance control
            }
            knx_motor_update();             // Every 1ms: current loop
            next_wake += 1;
            osDelayUntil(next_wake);        // Precise timing, no drift
        }
    }

Key: osDelayUntil() ensures no cumulative drift.
- Motor current loop: 1 kHz (every tick)
- Balance control: 500 Hz (every 2nd tick)
- IMU + EKF: 500 Hz (every 2nd tick)

---

## 12. Config Layer

### 12.1 Configuration Hierarchy

    knx_board_config.h     <- Hardware constants (pins/timers/clocks)
          |
          v
    knx_target_config.h    <- Target platform resolution (STM32/MSPM0)
          |
          v
    knx_project_config.h   <- Project switches (module enable/controller select)
          |
          v
    knx_debug_config.h     <- Debug switches (test mode/assert)
          |
          v
    knx_params.h/c         <- Runtime parameters (dynamically adjustable)

### 12.2 Runtime Parameters

    knx_params_t g_knx_params = {
        .mech = {
            .wheel_radius_m = 0.032f,         // 32mm
            .wheel_base_m = 0.1600f,          // 160mm
            .body_mass_kg = 0.2152f,          // 215g
            .com_height_m = 0.05288f,         // 53mm
            .motor_gear_ratio = 28.0f,        // 28:1
            .motor_torque_efficiency = 0.65f,  // 65%
        },
        .safety = {
            .max_tilt_deg = 65.0f,
            .max_current_a = 4.2f,
            .max_speed_mps = 8.0f,
            .imu_timeout_ms = 50U,
        },
        .balance = {
            .lqr_k = {0.01, 2.5, 6.5, 0.02},
            .pwm_speed_kp = 20.0f,
            .pwm_angle_kp = 0.1f,
            .pwm_angle_kd = 0.0014f,
            .balance_en = true,
            .balance_req = true,
        },
    };

Modifiable at runtime via GDB or OctoLink for online tuning.

---

## 13. Algorithm Layer

### 13.1 QuaternionEKF

**File**: algorithm/QuaternionEKF/
**Role**: Fuse gyro + accel, estimate attitude quaternion and gyro bias
**Input**: gyro[3] (rad/s), accel[3] (m/s2), dt (s)
**Output**: Roll/Pitch/Yaw (deg), q[4], GyroBias[3]
**Algorithm**: Error-state EKF, state = [q0,q1,q2,q3, bx,by,bz] (7-dim)

### 13.2 Other Libraries

| Library         | Usage                         | Status      |
|-----------------|-------------------------------|-------------|
| QuaternionEKF   | Attitude estimation (core)    | Active      |
| PID             | Motor velocity/current loops  | Active      |
| CMSIS-DSP       | Matrix math for EKF (ARM opt) | Active      |
| MahonyAHRS      | Backup attitude in BMI088 drv | Non-primary |
| Kalman Filter   | Generic Kalman filter         | Unused      |
| Filter / Ramp   | Digital filters / ramp funcs  | Standby     |
| CRC             | CRC8/CRC16                    | OctoLink    |

---

## 14. Data Flow Diagrams

### 14.1 Main Data Flow (Balance Mode)

                          +-------------+
                          |   BMI088    |
                          |  (SPI1)     |
                          +------+------+
                                 | accel[3], gyro[3]
                                 v
    +----------+        +--------------+
    | Encoder  |------> |  knx_imu     |
    | TIM1     | speed  |  EKF fusion  |
    | LPTIM1   |        +------+-------+
    +----+-----+               | knx_imu_data_t
         |                     v
         |             +--------------+
         |             | knx_balance  |
         |             | LQR/PID/PWM  |
         |             +------+-------+
         |                    | torque/duty
         |      +-------------v------------+
         |      |      knx_motor           |
         |      |   (unified control)      |
         |      +-------------+------------+
         |                    |
         |      +-------------v------------+
         |      |       DRV8701E           |
         |      |   PWM + GPIO + ADC       |
         |      +-------------+------------+
         |                    |
         |             +------v------+
         |             |    Motor    |
         |             +-------------+
         |                    |
         +--------------------+ (encoder feedback)

### 14.2 Telemetry Data Flow

    motor_state + imu_data + balance_debug
        -> knx_telemetry_update() -> OctoLink -> UART1

    Channels:
      1-4:   Motor current
      20-25: IMU attitude + temperature
      30-49: Balance debug (angle/rate/velocity/torque/LQR terms)

---

## 15. Control Loops Explained

### 15.1 Current Loop (1 kHz)

    target_current (A)
        -> DRV8701E_CurrentControl()
        -> PI controller (Kp, Ki, integral clamp)
        -> duty (normalized -1 to 1)
        -> DRV8701E_SetDutyNorm() -> TIM2 PWM

### 15.2 Balance Loop (500 Hz, Cascade PWM)

    velocity (m/s)
        |
        v
    [Speed Loop: Kp=20, Ki=0]  --> theta_target (deg)
                                        |
    theta_target - theta_actual         |
        |                               |
        v                               v
    [Angle Loop: Kp=0.1, Kd=0.0014] --> duty (-1.0 to 1.0)
                                            |
                                            v
                                  knx_motor_set_duty_target()

### 15.3 Control Timing

    Time(ms)  0    1    2    3    4    5
    Ctrl:     |M   |M+B |M   |M+B |M   |   M=motor(1ms), B=balance(2ms)
    App:      |S   |S   |S   |S   |S   |   S=sys+safety(1ms)
    Tel:      |           |           |T  |   T=telemetry(20ms)

---

## 16. Thread Safety and Critical Sections

### 16.1 Shared Data Inventory

| Shared Variable       | Writer    | Reader    | Protection           |
|-----------------------|-----------|-----------|----------------------|
| s_imu_data            | Ctrl(imu) | App       | taskENTER_CRITICAL   |
| motor_*_state         | Ctrl(mot) | App       | taskENTER_CRITICAL   |
| s_state / s_mode      | App(sys)  | Ctrl      | taskENTER_CRITICAL   |
| s_fault_latch         | App       | App       | taskENTER_CRITICAL   |
| s_balance_debug       | Ctrl      | App       | No protection        |
| g_knx_params          | GDB       | Ctrl+App  | No protection        |

### 16.2 Potential Race Conditions

1. **g_knx_params**: float writes are non-atomic. TODO: double-buffer.
2. **s_balance_debug**: non-atomic struct copy. Low priority fix.

---

## 17. Hardware Pin Mapping

### 17.1 MCU: STM32H743IIT6

| Function           | Pin       | Peripheral    | Description      |
|--------------------|-----------|---------------|------------------|
| Left motor PWM     | PA15      | TIM2_CH1      | DRV8701E EN      |
| Left motor dir     | PB0       | GPIO          | PHASE            |
| Left motor current | PA4       | ADC1_CH18 R2  | Current sense    |
| Right motor PWM    | PA2       | TIM2_CH3      | DRV8701E EN      |
| Right motor dir    | PB11      | GPIO          | PHASE            |
| Right motor current| PC0       | ADC1_CH10 R1  | Current sense    |
| Left encoder A/B   | PA8/PA9   | TIM1          | Quadrature 4x    |
| Right encoder A/B  | PG12/PG11 | LPTIM1        | External 2x      |
| BMI088 SCK         | PA5       | SPI1_SCK      |                  |
| BMI088 MOSI        | PD7       | SPI1_MOSI     |                  |
| BMI088 MISO        | PG9       | SPI1_MISO     |                  |
| BMI088 Accel CS    | PH8       | GPIO          |                  |
| BMI088 Gyro CS     | PI3       | GPIO          |                  |
| BMI088 Heater      | PE5       | TIM15_CH1     |                  |
| Debug UART         | (CubeMX)  | USART1        | OctoLink         |
| Track MUX AD0-2    | PB4/5/7   | GPIO          |                  |
| Track ADC          | PC5       | ADC2_CH8      |                  |
| Debug LED          | PH7       | GPIO          | Heartbeat        |

### 17.2 Clock Tree

    HSE -> PLL1(/5,*192,/2) -> SYSCLK=480MHz
    SYSCLK -> AHB=240MHz -> APB1/2=120MHz
    PLL2(/15,*100,/2) -> ADC=50MHz
    Flash latency = 4 WS

---

## 18. Known Issues and TODO

### 18.1 Code Issues

| #  | Issue                                    | Severity |
|----|------------------------------------------|----------|
| 1  | Octolinker/VOFA+ include usart.h directly | Low      |
| 2  | g_knx_params has no concurrency guard    | Medium   |
| 3  | s_balance_debug has no protection         | Low      |
| 4  | knx_uart.h is a stub implementation      | Low      |
| 5  | LPTIM 16-bit counter overflow risk        | Medium   |

### 18.2 Feature TODO

| #  | Feature                              | Priority |
|----|--------------------------------------|----------|
| 1  | Parameter persistence (Flash)        | Medium   |
| 2  | Battery voltage monitoring           | Medium   |
| 3  | FreeRTOS watchdog task               | Medium   |
| 4  | Line tracking control algorithm      | Medium   |
| 5  | Unified UART platform abstraction    | Low      |
| 6  | OTA firmware update                  | Low      |
| 7  | Stack usage monitoring               | Low      |

### 18.3 Tuning Guide (Cascade PWM Controller)

**Tuning sequence**:
1. Verify IMU attitude signs (roll/pitch direction)
2. Verify encoder speed signs (positive = forward)
3. Tune angle loop: pwm_angle_kp (stand up), pwm_angle_kd (suppress oscillation)
4. Tune speed loop: pwm_speed_kp (disturbance rejection)
5. Fine-tune: pwm_angle_ki (steady-state, use carefully), pwm_deadzone (friction)

**Polarity parameters** (knx_params.h):

    KNX_BALANCE_ANGLE_SIGN       = -1.0
    KNX_BALANCE_RATE_SIGN        = -1.0
    KNX_BALANCE_VELOCITY_SIGN    =  1.0
    KNX_BALANCE_LEFT_SPEED_SIGN  =  1.0
    KNX_BALANCE_RIGHT_SPEED_SIGN = -1.0

---

## Appendix A: Build Instructions

    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Debug ..
    cmake --build . --parallel
    # Output: build/knexus_st.elf / .hex / .bin

Toolchain: arm-none-eabi-gcc 13.x+, CMake 4.2+

---

## Appendix B: File Dependency Graph

    knx_types.h <- nearly all User/ files
      +-- knx_gpio.h <- knx_spi/pwm/adc.h, board, drv
      +-- knx_spi.h  <- drv/bmi088.h
      +-- knx_pwm.h  <- drv/drv8701e.h, bmi088.h
      +-- knx_adc.h  <- drv/drv8701e.h, track_sensor.h
      +-- knx_encoder.h <- drv/encoder.h, board
      +-- knx_time.h <- modules/imu, sys

    knx_params.h <- modules/motor, ctrl, safety, sys, telemetry
    knx_project_config.h <- modules/motor, ctrl
    knx_debug_config.h <- tasks/ctrl, app, modules/safety

---

*End of document. Update this document when source code changes.*
