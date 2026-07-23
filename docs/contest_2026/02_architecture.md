# 架构

依赖只允许从上向下：

```text
knx26_user / 比赛策略
        ↓
contest_2026 runtime + 六任务
        ↓
chassis / track / intersection / can_bus / comm / safety
        ↓
motor / drive / imu / grayscale / host_comm
        ↓
drv8701e / bmi088 / encoder / track_sensor
        ↓
platform API + board binding
```

约束：

- `common/User` 不包含 STM32 HAL 或 TI DriverLib 调用。
- `board` 只绑定引脚和外设实例，不实现比赛策略。
- 路口识别只输出结果，不直接控制电机。
- 通信回调只更新命令/状态，不执行阻塞流程。
- 所有长时间运行逻辑由固定周期任务调用。
