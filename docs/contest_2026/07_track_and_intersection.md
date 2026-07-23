# 循迹与路口识别

循迹模块负责采集、校准和线偏差：

```c
knx_track_capture_black();
knx_track_capture_white();
knx_track_update();
knx_track_snapshot(&track);
```

标定时分别把全部探头放在黑线和白底上调用对应函数。输出包含 8 路原始值、
归一化值、数字位图、连续 `line_error`、`line_strength` 和 `line_lost`。

路口模块累计 64×8 窗口。STM32 使用 TinyCNN；MSPM0 为节省 RAM 使用宽线
启发式分类，但保持相同 API。识别结果通过
`knx26_user_on_intersection()` 通知上层，模块本身不会转向或停车。
