# 板级配置

## STM32H743 主板

| 功能 | 外设 |
|---|---|
| 电机 PWM | TIM2 CH1/CH3 |
| 电流采样 | ADC1 + DMA |
| 编码器 | TIM1、LPTIM1 |
| BMI088 | SPI1，PH8/PI3 片选 |
| 加热 | TIM15 CH1 |
| 循迹 ADC | ADC2 + PB4/PB5/PB7 MUX |
| 调试 | USART1 / OctoLink |
| HOST | USART2 |
| CAN BUS 1 | FDCAN1 PA11/PA12 |
| CAN BUS 2 | FDCAN2 PB12/PB13 |

## MSPM0G3507

MSPM0 使用相同模块 API。芯片当前只有一个已绑定的 `CANFD0`，因此
`KNX_CAN_BUS_1` 可用，`KNX_CAN_BUS_2` 返回 `KNX_NOT_READY`。这是硬件能力
差异，不在公共模块中伪造第二控制器。

新增 UART 时，在各自 `knx_board.c` 中创建 `knx_uart_t` 和
`knx_host_comm_t`，然后调用 `knx26_attach_board_link()` 或
`knx26_attach_peer_link()`。
