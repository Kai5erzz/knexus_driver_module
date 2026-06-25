# Knexus Driver Module

嵌入式机器人云台与底盘控制固件，支持多平台构建。

## 概述

本项目为机器人系统提供完整的硬件驱动、控制算法和运行时模块，支持两个 MCU 平台：

| 目标平台 | 芯片 | 架构 | 主频 | 用途 |
|---------|------|------|------|------|
| **STM32** | STM32H743IIT6 | Cortex-M7 | 480 MHz | 主控：双轴云台 + 视觉跟踪 + 底盘驱动 |
| **MSPM0** | MSPM0G3507 | Cortex-M0+ | 80 MHz | 辅控：电机驱动测试 |

---

## 功能模块详细说明

### 1. 双轴云台控制

**文件**: `common/User/modules/gimbal/knx_gimbal_ctrl.*`

云台控制器驱动 Yaw (JC4310, CAN ID=3) 和 Pitch (JC2804, CAN ID=1) 两轴 FOC 电机，实现视觉跟踪和位置保持。

#### 状态机

采用 4 状态线性状态机：

```
BOOT ──(boot_delay_ms)──► ENTER_SERVO ──(mode_gap_ms)──► SPEED_MODE ──(mode_gap_ms)──► CONTROL
                                                                                        │
                                                                                   (稳态闭环控制)
```

| 状态 | 动作 | 默认延时 |
|------|------|---------|
| `BOOT` | 上电等待 | 500 ms |
| `ENTER_SERVO` | 发送 `JC_EnterServo()` 到两轴电机 | 300 ms |
| `SPEED_MODE` | 发送 `JC_SwitchMode(JC_MODE_SPEED)` | 300 ms |
| `CONTROL` | 稳态闭环控制，10 ms 周期（100 Hz） | — |

#### PID 控制架构

系统使用 **四组独立 PID 控制器**，分两对互斥工作：

**视觉跟踪 PID**（像素误差驱动，视觉数据有效时使用）：

| 轴 | Kp | Ki | Kd | 输出限幅 |
|----|----|----|-----|---------|
| Yaw | 0.25 | 0.02 | 0.01 | 20 RPM |
| Pitch | 0.10 | 0.005 | 0.01 | 20 RPM |

**位置保持 PID**（角度误差驱动，视觉数据超时时自动切换）：

| 轴 | Kp | Ki | Kd | 输出限幅 |
|----|----|----|-----|---------|
| Yaw | 0.20 | 0.0 | 0.0 | 15 RPM |
| Pitch | 0.20 | 0.0 | 0.0 | 15 RPM |

两组 PID 互斥：跟踪激活时重置保持 PID，视觉超时时重置跟踪 PID。所有增益均可通过参数系统运行时调整（范围 [-100, 100]）。

#### 控制流程

1. **视觉跟踪模式**: 误差信号 = 像素空间偏差（`-vision.err_x` 用于 yaw，`+vision.err_y` 用于 pitch）→ PID → 符号校正 → 机械角度限位 → `JC_SetSpeed()`
2. **位置保持模式**: 误差信号 = IMU 角度误差（目标角度 - 当前角度）→ PID → 输出限幅 → `JC_SetSpeed()`。Pitch 目标角度限制在 ±60°。

#### 机械限位

当 IMU 角度超过限位阈值且指令方向继续超出时，指令清零并触发限位标志。限位激活时对应轴的 PID 积分器重置，防止积分饱和。

---

### 2. 视觉跟踪模块

**文件**: `common/User/modules/vision/knx_vision.*`

接收上位机（如树莓派、Jetson 等）通过串口发送的目标检测结果。

#### 协议格式

- 消息 ID: `0x10`
- 负载长度: 13 字节（1B msg_id + 4B err_x + 4B err_y + 4B conf）
- 字节序: 小端

#### 数据结构

```c
typedef struct {
    float err_x;            // 目标中心水平像素偏差
    float err_y;            // 目标中心垂直像素偏差
    float conf;             // 检测置信度 [0.0, 1.0]
    uint32_t timestamp_ms;  // 接收时间戳
    uint32_t rx_count;      // 累计有效消息计数
    uint8_t valid;          // 首次收到有效消息后置 1
} knx_vision_target_t;
```

#### 数据流

```
上位机 ──(UART)──► knx_host_comm (解帧/CRC校验) ──► knx_vision_payload_callback
                                                         │
                                                    写入 s_target
                                                         │
                                                    knx_vision_snapshot() ──► 云台控制器
```

#### 有效性判定

视觉目标被视为"新鲜"需同时满足：
1. `valid == 1`（至少收到过一条有效消息）
2. `conf >= 0.15`（置信度超过最小阈值）
3. `age_ms <= 200 ms`（数据未超时）

---

### 3. 底盘驱动模块

**文件**: `common/User/modules/drive/knx_drive.*`

差速驱动底盘控制，支持 5 种运行模式。

#### 运行模式

| 模式 | 说明 |
|------|------|
| `IDLE` | 停机，输出清零 |
| `VELOCITY` | 直接线速度 + 角速度指令 |
| `POSITION` | 位置 PID 产生线速度参考，角速度强制为 0 |
| `ANGLE` | 航向 PID 产生角速度参考，线速度强制为 0 |
| `POSITION_ANGLE` | 位置 PID + 航向 PID 同时工作 |

#### 四级级联 PID 控制架构

```
目标位置 ──► [位置 PID] ──► 线速度参考 ──┐
                                          ├──► [斜坡限制] ──► [线速度 PID] ──► 左/右轮目标
目标航向 ──► [航向 PID] ──► 角速度参考 ──┘     [斜坡限制] ──► [角速度 PID] ──►
```

1. **外环位置 PID**: 位置误差 → 线速度参考 (m/s)
2. **外环航向 PID**: 航向误差（归一化到 [-π, π]）→ 角速度参考 (rad/s)
3. **斜坡限制**: 线速度和角速度参考均通过 `slope_following()` 进行加速度限幅
4. **内环线速度 PID**: 测量速度 vs 规划速度
5. **内环角速度 PID**: 测量角速度 vs 规划角速度

#### 差速运动学

正运动学（里程计）：
```
线速度 = 0.5 × (左轮速 + 右轮速)
角速度 = (右轮速 - 左轮速) / 轮距
航向增量 = (右轮增量 - 左轮增量) / 轮距
```

逆运动学（指令分配）：
```
左轮目标 = (线速度指令 - 角速度指令 × 半轮距) × 左轮符号
右轮目标 = (线速度指令 + 角速度指令 × 半轮距) × 右轮符号
```

所有符号校正（反馈符号、指令符号）均可通过参数配置，适配不同电机安装方向。

#### 指令超时保护

在 `VELOCITY` 模式下，若 `command_timeout_ms` 内无新指令到达，底盘自动停车。

---

### 4. 电机驱动模块

**文件**: `common/User/modules/motor/knx_motor.*`

统一电机抽象层，封装 DRV8701E H 桥驱动器 + 正交编码器，支持左右双电机。

#### 控制模式

| 模式 | 说明 |
|------|------|
| `SPEED` | 速度闭环，编码器反馈，1 ms 电流环 |
| `TORQUE` | 力矩目标 → 电流转换 → 电流闭环 |
| `CURRENT_TEST` | 直接电流控制（调试用） |
| `DUTY` | 直接 PWM 占空比（-1.0 ~ 1.0），开环 |

力矩-电流转换：`电流 = 力矩 / (力矩常数 × 减速比 × 效率)`

#### 更新频率

`knx_motor_update()` 以 **1 ms** 周期调用（电流环），每次执行：
1. 读取目标值（临界区保护）
2. ADC 电流采样
3. 编码器速度计算
4. 按模式分发控制
5. 同步状态

---

### 5. 灰度循迹与神经网络路口判断

**文件**: `common/User/modules/track_nn/track_tinycnn.*`

使用轻量级 1D-CNN 对灰度传感器采集的赛道图像进行路口类型分类，实现在嵌入式端的实时推理。

#### 工作流程

```
灰度传感器 (8通道) ──► 等距采样 64 行 ──► 64×8 灰度矩阵 ──► TinyCNN 推理 ──► 路口分类结果
                                                                      │
底盘前进 20cm (位置PID + IMU航向保持) ◄────────────────────────────────┘
```

1. **标定阶段**: 按 KEY0 采样黑/白基准面，计算各通道的归一化参数
2. **采集阶段**: 按 KEY1 触发，底盘以位置 PID 前进 20cm，同时 IMU 航向保持，等距采集 64 行 × 8 列灰度数据
3. **推理阶段**: 对 64×8 矩阵运行 TinyCNN 推理，输出路口分类
4. **输出阶段**: 按 KEY0 通过 OctoLink 发送矩阵和推理结果

#### CNN 网络结构

输入: 64 行 × 8 列 `uint8` 灰度值（0=白底, 255=黑线），归一化到 [0, 1]，转置为 `[8通道][64长度]` 的 1D 信号。

| 层 | 类型 | 输入通道 | 输出通道 | 卷积核 | 填充 | 步长 | 激活 | 输出形状 |
|----|------|---------|---------|--------|------|------|------|---------|
| Conv1 | Conv1D | 8 | 8 | 5 | 2 | 1 | ReLU | [8][64] |
| Conv2 | Conv1D | 8 | 8 | 5 | 2 | 1 | ReLU | [8][64] |
| GAP | 全局平均池化 | 8 | 8 | — | — | — | — | [8] |
| FC | 全连接 | 8 | 2 | — | — | — | Softmax | [2] |

#### 输出

```c
typedef struct {
    uint8_t pred_class;     // 预测类别
    float confidence;       // 置信度
    float logits[2];        // 原始输出
    float probs[2];         // Softmax 概率
} track_tinycnn_result_t;
```

#### 内存策略

使用 ping-pong 双缓冲（`buf_a[8][64]` / `buf_b[8][64]`）避免栈分配，嵌入式友好。权重由 Python 训练工具链（`track_sample/`）导出为 C 数组（`track_tinycnn_weights.c`）。

#### 训练工具链

`track_sample/` 目录包含完整的 PyTorch 训练流水线：
- 数据采集: 通过 OctoLink 从嵌入式端获取灰度矩阵
- 标注: 路口类型分类（正常/左转/右转/直行/十字/丢失）
- 训练: 1D-CNN 模型训练
- 导出: 自动生成 C 权重文件和推理代码

---

### 6. IMU 姿态估计

项目支持两套 IMU 方案和多种姿态估计算法。

#### BMI088 (SPI)

**文件**: `common/User/drv/bmi088/bmi088.*`

- 接口: SPI1（SCK=PA5, MOSI=PD7, MISO=PG9）
- 加速度计: 800 Hz ODR, 6g 量程
- 陀螺仪: 2000 dps, 230 Hz 带宽
- 校准: 3000 次采样/次，最多 5 次重试；陀螺仪偏移 < 0.01 rad/s，加速度计重力偏差 < 0.5 m/s²
- 加热器温控: PI 控制器驱动 PWM 加热通道，目标温度 40°C，最大占空比 30%
- 零值检测: 连续 5 帧全零自动触发重新初始化

#### DM-IMU-L1 (CAN)

**文件**: `common/User/drv/dm_imu_l1/dm_imu_l1.*`

- 接口: FDCAN, 默认 ID=0x11
- 数据类型: 加速度、陀螺仪、欧拉角、四元数
- 上报速率: 100/125/200/250/500/1000 Hz 可配
- 欧拉角累积: 自动检测 ±180°/±90° 跨越，维护连续角度 `yaw_total`/`roll_total`/`pitch_total`

#### 姿态估计算法

**四元数 EKF** (`common/User/algorithm/QuaternionEKF/`):
- 状态维度: 6（四元数 q0-q3 + 陀螺仪偏置 bias_x, bias_y）
- 观测维度: 3（归一化加速度计，重力方向参考）
- 特性: 卡方检验自适应增益、陀螺仪偏置衰减因子（λ=0.9996）、偏置校正限幅
- Yaw 轴偏置显式置零（加速度计不可观测）

**Mahony AHRS** (`common/User/algorithm/MahonyAHRS/`):
- 互补滤波器，PI 反馈
- 支持 6 轴（IMU）和 9 轴（含磁力计）模式
- 快速逆平方根（Quake III 算法）
- 用于 EKF 参考对比

---

### 7. 安全监控模块

**文件**: `common/User/modules/safety/knx_safety.*`

更新频率: 1 ms，优先级: `osPriorityHigh`（所有任务中最高）。

#### 安全等级

| 等级 | 含义 |
|------|------|
| `OK` | 无故障无警告 |
| `WARN` | 存在警告，无故障 |
| `FAULT` | 至少一个故障激活 |

#### 故障类型（位掩码，锁存型 — 需显式清除）

| 故障 | 说明 |
|------|------|
| `OVERCURRENT` | 电机电流超限 |
| `IMU_FAIL` | IMU 数据超时或缺失 |
| `ENCODER_FAIL` | 编码器故障 |
| `WATCHDOG` | 电机驱动看门狗 |
| `TILT` | 倾斜角超限（yaw>95° 或 pitch>50°） |
| `OVERSPEED` | 速度超限（指令 > 80 RPM） |

#### 警告类型（非锁存，每周期刷新）

| 警告 | 说明 |
|------|------|
| `STARTUP` | 启动宽限期内 |
| `VISION_STALE` | 视觉数据缺失或超时（> 500 ms） |
| `DISABLED` | 安全检查已禁用 |

#### 检查序列（每周期）

1. IMU 检查: 数据龄期 > 100 ms → 启动期内警告，否则故障
2. 视觉检查: 数据龄期 > 500 ms → 警告
3. 倾斜检查: |yaw| > 95° 或 |pitch| > 50° → 故障
4. 超速检查: |指令 RPM| > 80 → 故障
5. 看门狗: 进入伺服后电机状态异常 → 故障

---

### 8. 上位机通信协议

**文件**: `common/User/modules/host_comm/knx_host_comm.*`

#### 帧格式

```
[SOF: 0xA5 0x5A] [LEN: 2B LE] [PAYLOAD: 0~256B] [CRC16: 2B LE] [EOF: 0x0D 0x0A]
```

| 字段 | 长度 | 值 |
|------|------|----|
| SOF | 2 字节 | `0xA5 0x5A` |
| Length | 2 字节（小端） | 负载长度 0-256 |
| Payload | 0-256 字节 | 应用数据 |
| CRC | 2 字节（小端） | CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF) |
| EOF | 2 字节 | `0x0D 0x0A` |

最大帧长: 264 字节。

#### 接收状态机

9 状态字节解析器：`WAIT_SOF0 → WAIT_SOF1 → LEN0 → LEN1 → PAYLOAD → CRC0 → CRC1 → EOF0 → EOF1`

CRC 校验通过后分发至回调（1 个主回调 + 最多 4 个附加回调）。

---

### 9. 参数系统

**文件**: `common/User/modules/param/knx_param.*`, `knx_param_host.*`

#### 核心功能

- 支持类型: `F32`, `U32`, `I32`, `U8`
- 参数表: 最多 12 张表，每表可注册多个参数
- 查找方式: 按 ID（uint16）、按名称（字符串）
- 写入: 自动钳位到 [min, max]，只读参数拒绝写入
- 重置: 单参数或全部恢复默认值

#### 上位机远程访问协议

| 方向 | 消息 ID | 名称 | 负载 |
|------|---------|------|------|
| 请求 | `0x30` | GET_REQ | [id_lo, id_hi] |
| 请求 | `0x31` | SET_REQ | [id, type, value(4B)] |
| 请求 | `0x32` | RESET_REQ | [id_lo, id_hi] |
| 请求 | `0x33` | INFO_REQ | [id_lo, id_hi] |
| 响应 | `0xB0` | VALUE_RSP | [request_msg, status, id, type, flags, value(4B)] |
| 响应 | `0xB1` | INFO_RSP | [status, index, id, type, flags, default, min, max, name...] |
| 响应 | `0xB2` | ACK_RSP | [request_msg, status, id, param_count] |

---

### 10. 运行时基础设施

#### 健康监控 (`common/User/modules/health/knx_health.*`)

监控 8 个子系统: SYS, SAFETY, VISION, DM_IMU_L1, GIMBAL, JC, HOST_COMM, PARAM_HOST

健康状态优先级: `FAULT > STALE > WARN > UNKNOWN > DISABLED > OK`

状态变化自动记录到黑盒日志。

#### 黑盒日志 (`common/User/modules/blackbox/knx_blackbox.*`)

- 容量: 32 条事件的环形缓冲
- 事件类型: BOOT, HEALTH_CHANGE, CAN_UNMATCHED, SAFETY_CLEAR
- 每条记录: 时间戳 + 代码 + 来源 + 状态 + 参数

#### CAN 路由器 (`common/User/modules/can_router/knx_can_router.*`)

- 最多 8 条路由规则，每条覆盖连续 CAN ID 范围
- 首匹配优先分发
- 未匹配帧记录到黑盒

#### 遥测输出 (`common/User/modules/telemetry/knx_telemetry.*`)

通过 OctoLink 以 F32 格式输出调试变量，20 ms 更新周期。通道映射：

| 通道 | 内容 |
|------|------|
| 1-4 | 电机电流（左右目标/实际） |
| 10-19 | 底盘状态（轮速、线/角速度、模式、规划值） |
| 20-25 | IMU 姿态 + BMI088 温控 |
| 30-39 | 系统状态 + 底盘 PID 各级输出 |

---

### 11. 算法库

**目录**: `common/User/algorithm/`

| 算法 | 文件 | 说明 |
|------|------|------|
| PID | `PID/` | 位置式 PID，支持 8 种改进：积分限幅、微分先行、梯形积分、输出滤波、变积分率、微分滤波、堵转检测 |
| Kalman 滤波 | `kalman_filter/` | 通用可配置维度，CMSIS-DSP 矩阵运算，支持自动 H/K/R 调整 |
| 四元数 EKF | `QuaternionEKF/` | 6 维状态（四元数+偏置），卡方检验，自适应增益 |
| Mahony AHRS | `MahonyAHRS/` | 互补滤波器，PI 反馈，支持 6/9 轴 |
| CRC | `crc/` | CRC-16 校验 |
| 斜坡 | `ramp/` | 斜坡限制器 |
| 32 位滤波 | `filter32/` | 通用滤波器 |

---

### 12. 硬件驱动

**目录**: `common/User/drv/`

| 驱动 | 说明 |
|------|------|
| **BMI088** | 6 轴 IMU (SPI)，含自动校准（3000 采样×5 重试）和加热器 PI 温控（目标 40°C） |
| **DM-IMU-L1** | CAN 总线 IMU，支持加速度/陀螺仪/欧拉角/四元数，100-1000 Hz 可配上报率 |
| **JC4310/JC2804** | FOC 电机 CAN 驱动（协议 v3.0），支持力矩/速度/位置/PV/PVT 模式，25 种错误码 |
| **DRV8701E** | H 桥电机驱动 (PWM + 方向)，支持速度/电流/占空比控制 |
| **编码器** | 正交编码器读取，速度计算 |
| **灰度传感器** | 8 通道循迹传感器 |
| **OctoLink** | 调试变量实时输出 |
| **VOFA+** | 调试波形协议 |
| **LED/按键/蜂鸣器** | GPIO 外设 |

#### JC 电机驱动协议 (v3.0)

- CAN 寻址: TX=`0x600+id`, RX=`0x580+id`，DLC=8
- 寄存器读写: 16/32 位，大端序
- 控制模式: 力矩(0)、速度(1)、梯形位置(2)、滤波位置(3)、直接位置(4)、低速高力矩(5)
- 定点缩放: 所有物理量 ×100 传输
- 超时: 默认 20 ms，可配置

---

## FreeRTOS 任务调度

STM32 平台运行 6 个任务：

| 任务 | 优先级 | 周期 | 栈大小 | 功能 |
|------|--------|------|--------|------|
| `knx_safety` | High | 1 ms | 1024 B | 安全监控（最高优先级） |
| `knx_comm` | AboveNormal | 1 ms | 2048 B | 串口数据接收 → 视觉模块 |
| `knx_ctrl` | AboveNormal | 1 ms | 4096 B | 控制循环（云台/底盘/电机） |
| `knx_app` | Normal | 1 ms | 4096 B | 应用初始化 + 系统更新 (5ms) |
| `knx_ui` | BelowNormal | 1 ms | 2048 B | 按键(10ms)、灰度(20ms)、LED(500ms/100ms) |
| `knx_telemetry` | Low | 50 ms | 3072 B | OctoLink 遥测输出 (20 Hz) |

```
时间轴 (1ms tick):
Safety:  ████████████████████████████████  (每 1ms)
Comm:    ████████████████████████████████  (每 1ms)
Ctrl:    ████████████████████████████████  (每 1ms, IMU 每 2ms)
App:     ████░░░░████░░░░████░░░░████░░░░  (sys_update 每 5ms)
UI:      █░░░░░░░░█░░░░░░░░█░░░░░░░░█░░░  (蜂鸣器 1ms, 按键 10ms, 灰度 20ms)
Tele:    ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█  (每 50ms)
```

---

## 测试模式

通过 `knx_debug_config.h` 编译时选择，共 12 种：

| ID | 名称 | 功能 |
|----|------|------|
| 0 | `NONE` | 正常应用运行 |
| 1 | `DRV8701E` | 电流环阶梯波测试（PI 控制，4 段波形） |
| 2 | `BMI088` | BMI088 IMU 通信验证 |
| 3 | `ENCODER` | 编码器速度/位置测试 |
| 4 | `DRIVE_CALIB` | 底盘 12 步自动标定（极性/速度/位置/航向/电流） |
| 5 | `BMI088_IMU_EST` | IMU 姿态估计对比（EKF vs Mahony） |
| 6 | `GRAYSCALE_SAMPLE` | **灰度采集 + CNN 路口判断**（标定→采集→推理） |
| 7 | `JC2804` | JC2804 伺服电机协议测试 |
| 8 | `DM_IMU_L1` | DM-IMU-L1 CAN 通信测试 |
| 9 | `MINI_GIMBAL` | 小型云台测试（双 JC2804 + DM-IMU-L1） |
| 10 | `JC_DRIVER` | JC 多电机顺序测试（JC2804 + JC4310） |
| 11 | `STANDARD_GIMBAL` | 标准云台测试（JC2804 pitch + JC4310 yaw，含角度限位） |
| 12 | `COLOR_TRACK` | **彩色跟踪云台**（完整视觉→云台跟踪链路） |

---

## 架构

项目采用三层架构：

```
┌──────────────────────────────────────────────────────────────┐
│                     Application Layer                        │
│   knx_app · test_mode · FreeRTOS tasks (6 个任务)            │
├──────────────────────────────────────────────────────────────┤
│                      Module Layer                            │
│   gimbal · motor · drive · vision · safety                   │
│   track_nn · telemetry · host_comm · param · health          │
│   can_router · blackbox                                      │
├──────────────────────────────────────────────────────────────┤
│                   Platform Abstraction Layer                  │
│   knx_pwm · knx_can · knx_spi · knx_uart                    │
│   knx_encoder · knx_gpio · knx_adc · knx_port · knx_time    │
└──────────────┬──────────────────────────┬────────────────────┘
               │                          │
       ┌───────┴───────┐          ┌───────┴───────┐
       │   STM32H743   │          │    MSPM0      │
       │  实现 (HAL)    │          │  实现 (DL)     │
       └───────────────┘          └───────────────┘
```

- **平台抽象层** — `common/User/platform/include/` 定义统一接口（ADC/CAN/SPI/UART/PWM/Encoder/GPIO/Time/Port），各平台在 `User/platform/` 下提供具体实现
- **模块层** — `common/User/modules/` 包含所有业务逻辑，通过平台抽象层访问硬件，与具体 MCU 无关
- **应用层** — `common/User/app/` 负责模块初始化链和 FreeRTOS 任务调度
- **算法库** — `common/User/algorithm/` 提供 PID、Kalman、EKF、AHRS 等标准控制/信号处理算法
- **硬件驱动** — `common/User/drv/` 封装具体传感器和执行器的通信协议

---

## 目录结构

```
knexus_driver_module/
├── CMakeLists.txt                  # 顶层 CMake，选择构建目标
├── CMakePresets.json               # 构建预设：stm32-debug / mspm0-debug
├── common/                         # 跨平台共享代码
│   └── User/
│       ├── algorithm/              # 算法：PID、Kalman、EKF、AHRS、CRC、Ramp、Filter
│       ├── app/                    # 应用主逻辑 + 12 种测试模式
│       ├── config/                 # 项目/目标/调试配置头文件
│       ├── drv/                    # 硬件驱动（BMI088、DM-IMU、JC、DRV8701E 等）
│       ├── modules/                # 运行时模块（云台/底盘/视觉/安全/NN 等）
│       ├── platform/include/       # 平台抽象层接口定义
│       └── util/                   # 工具库（数学、环形缓冲、滤波器）
├── stm32/                          # STM32H743 平台代码
│   └── knexus_st/
│       ├── Core/                   # CubeMX 生成的 HAL 初始化
│       ├── Drivers/                # STM32H7xx HAL + CMSIS
│       ├── Middlewares/            # FreeRTOS (ARM_CM4F) + CMSIS-DSP
│       └── User/
│           ├── board/knexus_stm32h743/   # 板级引脚/时钟配置
│           ├── platform/stm32h743/       # 平台 HAL 实现
│           └── tasks/                    # 6 个 FreeRTOS 任务定义
├── mspm0/                          # TI MSPM0 平台代码
│   └── knexus_mspm0/
│       ├── Middlewares/            # FreeRTOS (ARM_CM0)
│       └── User/
│           ├── board/knexus_mspm0/       # 板级配置
│           ├── platform/mspm0/           # 平台 DL 实现
│           └── tasks/                    # 任务定义
├── track_sample/                   # 灰度 CNN 训练工具链 (Python/PyTorch)
└── docs/                           # 运行时文档
    ├── common_param.md             # 参数注册表 API 与云台参数 ID
    ├── common_runtime.md           # CAN 路由、环形缓冲、健康模块、OctoLink 变量
    ├── gimbal_runtime.md           # 云台运行时：硬件、任务、控制流、安全系统
    └── octolink-mcp-skill.md       # OctoLink MCP 集成参考
```

---

## 构建

### 依赖

| 工具 | 版本 | 说明 |
|------|------|------|
| arm-none-eabi-gcc | — | ARM 交叉编译器 |
| CMake | 4.2+ | STM32Cube 附带 |
| Ninja | — | 构建系统 |
| OpenOCD | 0.12.0 | 烧录/调试 |

### 编译

```bash
# STM32 目标
cmake --preset stm32-debug
cmake --build build/stm32-debug

# MSPM0 目标
cmake --preset mspm0-debug
cmake --build build/mspm0-debug
```

或手动指定：

```bash
cmake -B build -DKNEXUS_TARGET=stm32 -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

### 烧录

```bash
# STM32 (SWD 无线调试器)
openocd -f interface/rm_dap_wireless.cfg -f target/stm32h7x.cfg \
  -c "program build/stm32-debug/knexus_st.elf verify reset exit"

# MSPM0 (DAPLink 无线调试器)
openocd -f interface/daplink_wireless_mspm0.cfg -f target/mspm0.cfg \
  -c "program build/mspm0-debug/knexus_mspm0.elf verify reset exit"
```

---

## 调试

### VSCode Cortex-Debug

项目包含完整的 launch 配置，F5 即可启动调试。

### OctoLink 实时遥测

项目集成了 OctoLink MCP 调试接口，可通过 Claude Code 进行：

- 实时变量监控（遥测流，36+ 通道）
- GDB 断点与单步调试
- 故障上下文捕获
- PID 参数在线调优
- 黑盒日志查看

---

## 配置

### 模块开关

编辑 [knx_project_config.h](common/User/config/knx_project_config.h) 启用/禁用功能模块：

```c
#define KNX_MOTOR_MODULE_ENABLE     1
#define KNX_IMU_MODULE_ENABLE       1
#define KNX_GIMBAL_MODULE_ENABLE    1
#define KNX_VISION_MODULE_ENABLE    1
#define KNX_SAFETY_MODULE_ENABLE    1
#define KNX_DRIVE_MODULE_ENABLE     1
#define KNX_TELEMETRY_MODULE_ENABLE 1
```

### 测试模式

编辑 [knx_debug_config.h](common/User/config/knx_debug_config.h) 切换测试模式（12 种，见上表）。

### 板级引脚

- STM32: [knx_board_config.h](stm32/knexus_st/User/board/knexus_stm32h743/knx_board_config.h)（480 MHz SYSCLK, TIM2 PWM, TIM1/LPTIM1 编码器, FDCAN, ADC）
- MSPM0: [knx_board_config.h](mspm0/knexus_mspm0/User/board/knexus_mspm0/knx_board_config.h)（80 MHz）

---

## 许可

Private — 仅供项目内部使用。
