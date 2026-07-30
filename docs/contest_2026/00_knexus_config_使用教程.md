# KNexus 配置与工作模式使用教程

`common/User/config/knexus_config.h` 是 2026 底盘应用唯一推荐的用户配置入口。
工作模式、任务周期、调试开关、底盘限制、PID、巡线、路口采样、BMI088 温控
和零偏学习的上电默认值都集中在该文件中。

底层引脚不属于用户参数。STM32 和 MSPM0 的 GPIO、PWM、ADC、编码器与 SPI
映射仍分别放在各自的 `User/board` 和 `User/platform` 中，不要复制到配置中心。

## 1. 选择工作模式

打开 `common/User/config/knexus_config.h`，在“工作模式选择”中只取消注释一行：

```c
#define KNEXUS_MODE_LINE_FOLLOW
// #define KNEXUS_MODE_INTERSECTION_SAMPLE
// #define KNEXUS_MODE_PID_TUNE
// #define KNEXUS_MODE_USER
#define KNEXUS_MODE_BOARD_TEST
```

编译器会检查选择结果。没有启用模式或同时启用多个模式都会直接报错，不会生成
行为不确定的固件。切换模式后需要重新构建并烧录。

## 2. 四种模式的按键流程

| 模式 | KEY0 | KEY1 | 主要用途 |
|---|---|---|---|
| `LINE_FOLLOW` | 开始黑线/白底两阶段校准 | 校准后启停巡线 | 正常任务开发 |
| `INTERSECTION_SAMPLE` | 未完成时重新校准；完成后发送矩阵 | 开始采样；运行中按下则停止 | 采集路口 64×8 样本 |
| `PID_TUNE` | 开始或重新开始正负阶跃 | 立即停止并失能 | 速度与转向基础调参 |
| `USER` | 由用户定义 | 由用户定义 | 新题目和上层任务 |
| `BOARD_TEST` | 长按一秒启用/关闭电机循环测试 | 立即停止并解除电机使能 | 全板硬件验收 |

校准流程为：先把传感器放在黑线上按 KEY0，蜂鸣器响后移动到白底，等待校准完成。
LED1 常亮表示校准成功。

## 3. 巡线模式

建议按以下顺序调整：

1. 用 `KNEXUS_LINE_BASE_SPEED_MPS_DEFAULT` 设置较低基础速度。
2. 从小到大增加 `KNEXUS_LINE_KP_DEFAULT`，直到车辆能明显吸附在线上。
3. 若直线存在长期偏向，再少量增加 `KNEXUS_LINE_KI_DEFAULT`。
4. 若回中摆动明显，再少量增加 `KNEXUS_LINE_KD_DEFAULT`。
5. 最后调整最大角速度、弯道降速和基础速度。

常用参数：

- `KNEXUS_LINE_MAX_DUTY_DEFAULT`：巡线时电机最大占空比。
- `KNEXUS_LINE_MAX_ANGULAR_RADPS_DEFAULT`：最大转向速度，太小会转不过弯。
- `KNEXUS_LINE_MIN_ANGULAR_RADPS_DEFAULT`：平滑的转向死区补偿。
- `KNEXUS_LINE_MIN_STRENGTH_DEFAULT`：弱线与搜线过渡阈值。
- `KNEXUS_LINE_RECOVERY_RADPS_DEFAULT`：弱线时的搜线力度。
- `KNEXUS_LINE_ERROR_FILTER_ALPHA`：误差滤波；越大响应越快，噪声也越明显。
- `KNEXUS_LINE_D_FILTER_ALPHA`：微分滤波；越小越平滑，但响应更慢。

带 `DEFAULT` 的参数会在上电时复制到 `knx26_line_kp` 等普通变量。因此可用
OctoLink/GDB在线修改变量；确认效果后，再把学习到的值写回配置文件。

## 4. 路口采样模式与 TinyCNN

默认采样距离为 20 cm，按编码器里程等距离保存 64 行，每行包含8路二值灰度，
最终形成 64×8 bit matrix。

相关参数：

- `KNEXUS_SAMPLE_DISTANCE_M`：采样总距离。
- `KNEXUS_SAMPLE_ROWS`：采样行数，当前模型要求64行。
- `KNEXUS_SAMPLE_POSITION_TOLERANCE_M`：终点位置容差。
- `KNEXUS_SAMPLE_SPEED_SETTLED_MPS`：判定停车稳定的速度阈值。
- `KNEXUS_SAMPLE_SETTLE_DELAY_MS`：到达终点后的稳定等待时间。
- `KNEXUS_SAMPLE_POSITION_*`：距离环PID。
- `KNEXUS_SAMPLE_HEADING_*`：直行航向保持PID。

STM32采样完成后会直接运行现有 `track_tinycnn`，并输出预测类别和置信度。
MSPM0保留完全相同的采样矩阵和接口，但默认使用轻量路口判定：当前完整 MSPM0
固件静态内存已经约26 KB，而芯片 SRAM 为32 KB，不适合无验证地加入神经网络
中间缓冲。采到的矩阵仍可发给上位机训练或离线验证。

OctoLink Lite 输出：

| 变量 ID | 内容 |
|---:|---|
| 100 | 64×8 bit采样矩阵 |
| 120 | TinyCNN/轻量判定类别 |
| 121 | 置信度 |

模式诊断从普通 OctoLink ID 730 开始：状态、采样行数、里程、类别、置信度和最近
一次矩阵发送状态。

## 5. PID调参模式

先选择一个调参子模式：

```c
#define KNEXUS_PID_TUNE_SPEED_STEP
// #define KNEXUS_PID_TUNE_TURN_STEP
```

- `SPEED_STEP`：左右轮同向，测试直线速度响应。
- `TURN_STEP`：输出正负角速度，测试转向响应。

按 KEY0 后依次运行正阶跃、短暂停顿和负阶跃；KEY1 在任意时刻立即停车。
`KNEXUS_TUNE_STEP_HOLD_MS` 设置每个阶跃的保持时间，
`KNEXUS_TUNE_MAX_DUTY_DEFAULT` 设置硬输出上限。在线调试时可修改：

```c
knexus_tune_linear_speed_mps
knexus_tune_angular_speed_radps
vc_left.kp / vc_left.ki / vc_left.kd
vc_right.kp / vc_right.ki / vc_right.kd
drv8701e_velocity_kv_cnt_per_mps
drv8701e_velocity_duty_deadzone_cnt
```

调参顺序建议为：确认编码器极性 → 调前馈 → 调 Kp → 调少量 Ki → 检查正负阶跃。
若正转正常但反转异常，应先检查极性、死区和硬件，不要用更大的积分掩盖问题。

## 6. 用户自定义模式

启用 `KNEXUS_MODE_USER` 后，在
`common/User/app/contest_2026/modes/knexus_mode_user.c` 中实现：

```c
knexus_mode_user_init();
knexus_mode_user_update();
knexus_mode_user_on_intersection();
knexus_mode_user_on_peer_message();
knexus_mode_user_debug_octo();
knexus_mode_user_debug_control_octo();
```

这些默认实现是弱符号，也可以在新的 `.c` 文件中用同名函数覆盖。不要修改任务文件；
统一调度器会自动把主任务、路口事件、车间通信和调试输出转发到当前模式。

## 7. 全板硬件验收模式

`KNEXUS_MODE_BOARD_TEST` 同时支持STM32和MSPM0，测试BMI088与40℃恒温、两路
CAN、8路循迹ADC、左右电机与编码器、LED、KEY0/KEY1和蜂鸣器。

- 电机上电保持关闭，长按KEY0一秒才会解锁。
- KEY1随时立即停车。
- 电机按照左轮低/中/高、右轮低/中/高、双轮中/高循环，每档2秒。
- 占空比硬上限为30%，参数在 `KNEXUS_BOARD_TEST_*` 区域。
- 两路CAN每100 ms发送ID `0x701` 和 `0x702`。
- OctoLink每100 ms输出ID `600..652`，不受周期调试总开关影响。

因为验收时经常需要翻转底板测量，该模式单独绕过姿态安全停车；按键解锁、30%占空比
上限和KEY1急停仍然有效。正常比赛模式不会绕过安全检查。

## 8. OctoLink调试开关

总开关：

```c
#define KNEXUS_DEBUG_OCTOLINK_ENABLE 0U
```

总开关为0时不发送任何周期调试数据。设为1后，还可分别控制：

- `KNEXUS_DEBUG_LINE_DATA_ENABLE`
- `KNEXUS_DEBUG_PID_DATA_ENABLE`
- `KNEXUS_DEBUG_IMU_DATA_ENABLE`

路口采样完成后按 KEY0 发送矩阵属于明确的用户操作，不依赖周期调试开关。

## 9. 任务调度与路口开关

`KNEXUS_FAST/CONTROL/TRACK/APP/COMM/DEBUG_PERIOD_MS` 控制六个常驻任务周期，
对应的 `*_PHASE_MS` 用于错峰启动。普通上层开发不要直接修改平台任务文件。

当前路口判断关闭：

```c
#define KNEXUS_INTERSECTION_ENABLE 0U
```

设为 `1U` 后才会创建独立路口感知任务并运行TinyCNN/规则识别。完整任务表和调度
诊断变量见 `13_task_scheduling.md`。

## 10. 配置文件与运行期参数的边界

`knexus_config.h` 保存可读、可版本管理的默认值；`knx_params.c` 保存底盘运行期参数
对象；各模块中的普通全局变量保存在线可调副本。不要把所有PID直接改成纯宏，否则
OctoLink无法在线调参，也不要在多个业务文件重复写默认数值。

## 11. 构建验证

```powershell
cube-cmake --preset mspm0-debug
cube-cmake --build --preset mspm0-debug

cube-cmake --preset stm32-debug
cube-cmake --build --preset stm32-debug
```

切换模式后必须重新构建。若出现“必须且只能启用一个模式”，检查工作模式区域是否
正好只有一行没有被注释。
