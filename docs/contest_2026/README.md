# Knexus 2026 电赛底盘固件

本目录只描述 `KNEXUS_APP=contest_2026`。该应用包含双电机底盘、BMI088、
灰度循迹、可选路口识别、双 FDCAN、上位机/板间/车间通信和分层实时任务；不包含
云台、JC 云台电机或 DM-IMU-L1。

建议阅读顺序：`00_knexus_config_使用教程` → `01_quick_start` → `02_architecture` →
`09_user_task_development`，需要哪个外设再查对应章节。

新底板验收参见 `11_board_test.md`。
DJI M3508/C620驱动与测试参见 `12_dji_m3508.md`。
任务周期、优先级和超期诊断参见 `13_task_scheduling.md`。
H题环形巡线、A线停车及球控接口参见 `14_h_balance_ball.md`。

## 最短入口

上层只需要使用：

```c
knx_chassis_enable();
knx_chassis_set_velocity(0.30f, 0.0f);
knx_chassis_stop();
```

工作模式和参数统一在 `common/User/config/knexus_config.h` 选择。模式实现位于
`common/User/app/contest_2026/modes`，不要在用户逻辑中直接调用 HAL。
