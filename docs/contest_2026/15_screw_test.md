# 丝杆与DM-IMU-L1测试模式

在 `common/User/config/knexus_config.h` 中只启用：

```c
#define KNEXUS_MODE_SCREW_TEST
```

该组合仅支持STM32版本：FDCAN2连接C620/M3508，FDCAN1连接DM-IMU-L1。

- 按住KEY0：目标速度 `+3000 rpm`，松开立即停止。
- 按住KEY1：目标速度 `-3000 rpm`，松开立即停止。
- KEY0和KEY1同时按下：停止。
- LED1亮：3508当前已使能。
- LED2亮：DM-IMU-L1的加速度、角速度、欧拉角和四元数均已接收。

速度可修改 `KNEXUS_SCREW_TEST_SPEED_RPM`，也可运行时修改
`knexus_screw_test_speed_rpm`。

## OctoLink变量

高频控制量：

- 900：方向（-1/0/+1）
- 901~905：目标、斜坡目标、反馈转速、电流、PID输出
- 906~909：外置IMU的roll、pitch、yaw、累计yaw
- 910：三轴角速度数组

低频诊断量：

- 916：平台是否支持
- 917/918：KEY0/KEY1状态
- 919：3508是否使能
- 920~926：3508初始化、收发计数和反馈年龄
- 927~929：DM-IMU路由及初始化状态
- 932~946：完整 `DM_IMU_L1_DebugOcto` 数据
