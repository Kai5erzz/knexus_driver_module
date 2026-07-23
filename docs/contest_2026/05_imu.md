# BMI088 与航向

运行时在控制任务中调用 `knx_imu_update()`，上层只读取：

```c
knx_imu_data_t imu;
knx_imu_snapshot(&imu);
```

重点字段：`accel[]`、`gyro[]`、`roll/pitch/yaw`、`yaw_total`、
`temperature`、`accel_ok`、`gyro_ok`、`last_update_ms`。

BMI088 每次有效温度读取后自动更新加热 PWM。OctoLink 的 706/707 分别为
连续航向和温度。比赛前在静止、水平、温度稳定条件下完成零偏校准。
