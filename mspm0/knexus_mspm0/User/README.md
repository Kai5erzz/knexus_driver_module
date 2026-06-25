# MSPM0 User Layer

This MSPM0 target is intentionally kept as a structured skeleton until the
SysConfig peripheral layout is finalized.

## Current Runtime

- `main.c`
  - Calls `SYSCFG_DL_init()`.
  - Calls `knx_mspm0_app_init()`.
  - Creates tasks through `knx_mspm0_tasks_init()`.
  - Starts FreeRTOS.

- `User/app`
  - Owns board/module initialization for MSPM0.

- `User/tasks`
  - `heartbeat`: toggles LED0 every 500 ms.
  - `app`: scans keys every 10 ms; KEY0 toggles LED1, KEY1 beeps.
  - `beep`: updates beep driver every 1 ms.

- `User/board/knexus_mspm0`
  - Binds SysConfig LED/KEY/BEEP pins to common drivers.
  - `knx_board_config.h` marks the target as `KNX_TARGET_MSPM0`.

- `User/platform/mspm0`
  - Implements the common platform interfaces.
  - GPIO, time, port, and blocking UART have initial implementations.
  - CAN, PWM, ADC, and SPI are compile-ready stubs.

## Files To Fill After SysConfig

- `knx_uart_mspm0.c`
  - Set board-level `knx_uart_t.handle` to the generated UART instance.
  - Add interrupt or DMA RX later if needed.

- `knx_can_mspm0.c`
  - Replace stub with MSPM0 CAN/MCAN DriverLib calls.
  - Call `knx_can_mspm0_dispatch_rx()` from the CAN ISR after parsing frames.

- `knx_pwm_mspm0.c`
  - Bind `knx_pwm_channel_t.timer/channel` to SysConfig timer outputs.

- `knx_adc_mspm0.c`
  - Bind ADC12 channels and DMA sequence after SysConfig generation.

- `knx_spi_mspm0.c`
  - Bind SPI instance and implement blocking or DMA transfers.

## Build

Configure:

```powershell
cmake -S D:\User\Project\knexus_driver_module `
      -B D:\User\Project\knexus_driver_module\build\mspm0-debug `
      -DKNEXUS_TARGET=mspm0
```

Build:

```powershell
cmake --build D:\User\Project\knexus_driver_module\build\mspm0-debug
```
