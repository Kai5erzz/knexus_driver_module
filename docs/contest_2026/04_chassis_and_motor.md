# 底盘与双电机

## 当前 MSPM0 左电机飞线

左电机 `L1_EN` 已从 `PA0 / TIMA0_C0` 迁移到：

- MCU 引脚：`PB8`
- 底板网络：`SPI_CUSTOM_CS1`
- 复用功能：`TIMA0_C0`
- 输出结构：Standard I/O，可推挽输出
- PWM：20 kHz，最大占空比仍限制为 30%

飞线前必须把原来的 `PA0 -> L1_EN` 通路断开或抬脚隔离，再把 `PB8 / SPI_CUSTOM_CS1` 飞到 `L1_EN`。不要让 PA0 与 PB8 同时连接到 L1_EN，PA0 原故障网络也不要继续挂在驱动输入上。

## 保留的 KEY0 运动测试

上电后两个电机保持停止。按一次 `KEY0`，依次执行：

1. 前进 25 cm。
2. 后退 25 cm，回到起点。
3. 原地左转 90°。
4. 原地右转 90°，回到初始朝向。

动作之间暂停 500 ms；单步超过 5 秒会禁用底盘并进入故障状态。`KEY1` 可随时急停。测试期间 LED1 亮，完成后 LED2 亮，故障时长鸣。

当前极性和占空比限制：

```c
float knx26_test_left_cmd_polarity = 1.0f;
float knx26_test_right_cmd_polarity = -1.0f;
float knx26_test_left_feedback_polarity = 1.0f;
float knx26_test_right_feedback_polarity = 1.0f;
float knx26_test_max_duty = 0.30f;
```

## 当前巡线测试

当前主程序已经切换为巡线状态机，选择位置：

```c
#define KNX26_USER_TEST_MODE KNX26_USER_TEST_LINE_FOLLOW
```

将其改为 `KNX26_USER_TEST_MOTION` 即可恢复本页上方的 25 cm / 90° 测试。

## OctoLink 730 段

| ID | 类型 | 含义 |
|---:|:---:|---|
| 730 | U8 | 状态：0 空闲、1 前进、2 暂停、3 后退、4 暂停、5 左转、6 暂停、7 右转、8 完成、9 故障 |
| 731 / 732 | F32 | 左 / 右电机命令极性 |
| 733 / 734 | F32 | 左 / 右编码器反馈极性 |
| 735 / 736 / 737 | F32 | 位置目标 / 实际 / 误差，单位 m |
| 738 / 739 / 740 | F32 | 航向目标 / 实际 / 误差，单位 rad |
| 741 | U32 | 当前步骤经过时间，单位 ms |
| 742 | U32 | 故障次数 |
| 743 | F32 | 最大允许占空比 |

## 常用接口

```c
knx_chassis_enable();
knx_chassis_set_velocity(linear_mps, angular_radps);
knx_chassis_set_wheel_speed(left_mps, right_mps);
knx_chassis_set_position(position_m);
knx_chassis_set_angle(heading_rad);
knx_chassis_stop();
knx_chassis_disable();
```
