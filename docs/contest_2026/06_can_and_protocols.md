# 双 CAN 与通信协议

## CAN

```c
knx_can_bus_subscribe(KNX_CAN_BUS_1, first_id, last_id, callback, user);
knx_can_bus_send(KNX_CAN_BUS_1, id, data, length);
knx_can_bus_snapshot(KNX_CAN_BUS_1, &state);
```

STM32 两路总线每 100 ms 发送测试心跳：BUS1=`0x601`，BUS2=`0x602`，载荷
以 `KNX2` 开头。发送等待被限制为 1 ms，未接分析仪或未 ACK 时不会无限阻塞。

## HOST / BOARD / PEER

三类链路共用 `knx_comm` 消息层，底层使用 `knx_host_comm` 的 CRC 帧。

```text
version, type, source, destination, sequence, flags, length, payload
```

`destination=0xFF` 表示广播。消息类型包括心跳、命令、状态、循迹、路口和
用户消息。速度命令载荷固定为：`float linear` + `float angular` + `uint8 enable`。

当前 STM32 USART2 自动绑定 HOST。BOARD 和 PEER 需要板级提供额外 UART 后调用：

```c
knx26_attach_board_link(&board_transport);
knx26_attach_peer_link(&wireless_transport);
```
