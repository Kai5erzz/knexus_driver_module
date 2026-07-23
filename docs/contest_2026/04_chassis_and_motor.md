# 底盘与双电机

常用 API：

```c
knx_chassis_init();
knx_chassis_enable();
knx_chassis_set_velocity(linear_mps, angular_radps);
knx_chassis_set_wheel_speed(left_mps, right_mps);
knx_chassis_stop();
knx_chassis_disable();
knx_chassis_snapshot(&state);
```

`set_velocity` 使用差速运动学和速度闭环；`set_wheel_speed` 适合标定左右轮。
快速电流/电机更新只能由 `knx26_fast` 调用，底盘控制只能由 `knx26_ctrl`
调用。用户任务只写目标值。

方向、轮距、最大速度、加速度和 PID 参数位于 `knx_params`。换车后先校验：
正占空比方向、电流符号、编码器符号、左右轮速度比例。
