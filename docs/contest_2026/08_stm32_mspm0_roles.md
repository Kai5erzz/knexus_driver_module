# STM32 与 MSPM0 角色

默认节点：STM32=`1`，MSPM0=`2`。

- STM32：主控、双 FDCAN、TinyCNN、上层策略和完整 OctoLink。
- MSPM0：执行/采集节点，也可独立运行底盘、BMI088、循迹和通信模块。
- 两端共享 `knx_chassis`、`knx_track`、`knx_comm` 和消息结构。

板间推荐传递目标速度、编码器/IMU状态、循迹结果和健康心跳；不要跨板共享
C 结构体内存布局。所有线上消息使用明确长度和版本号。

如果 MSPM0 只作为从板，上层回调保持为空，由 BOARD 命令消息驱动底盘即可。
