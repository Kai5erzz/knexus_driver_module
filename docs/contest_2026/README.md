# Knexus 2026 电赛底盘固件

本目录只描述 `KNEXUS_APP=contest_2026`。该应用包含双电机底盘、BMI088、
灰度循迹、路口识别、双 FDCAN、上位机/板间/车间通信和六个主任务；不包含
云台、JC 云台电机或 DM-IMU-L1。

建议阅读顺序：`01_quick_start` → `02_architecture` →
`09_user_task_development`，需要哪个外设再查对应章节。

## 最短入口

上层只需要使用：

```c
knx_chassis_enable();
knx_chassis_set_velocity(0.30f, 0.0f);
knx_chassis_stop();
```

比赛逻辑放在 `common/User/app/contest_2026/knx26_user.c` 的四个弱回调中，
不要在用户逻辑中直接调用 HAL。
