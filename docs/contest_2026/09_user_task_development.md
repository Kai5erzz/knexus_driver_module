# 上层开发

比赛策略只实现四个入口：

```c
void knx26_user_init(void);
void knx26_user_update(const knx26_context_t *ctx);
void knx26_user_on_intersection(const knx_intersection_result_t *result);
void knx26_user_on_peer_message(const knx_comm_message_t *message);
```

`ctx` 已包含底盘、IMU、循迹、路口、两路 CAN 和三类链路状态。建议用户状态机
只做模式切换和目标生成：

```text
IDLE → CALIBRATE → FOLLOW_LINE → HANDLE_INTERSECTION → FINISHED/FAULT
```

禁止在回调中延时、轮询串口、等待 CAN 邮箱或直接更新电机。耗时流程拆成状态，
每次 `knx26_user_update` 推进一步。
