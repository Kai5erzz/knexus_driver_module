# Gimbal Runtime Notes

This document records the current STM32 runtime layout for the JC4310/JC2804
standard gimbal with DM-IMU-L1 and vision input.

## Hardware Roles

- Yaw motor: JC4310, CAN ID 3.
- Pitch motor: JC2804, CAN ID 1.
- IMU: DM-IMU-L1 on FDCAN1, active report mode.
- Vision link: USART2 host protocol.

Motor direction conventions:

- Positive yaw speed moves the camera left.
- Positive pitch angle now needs negative pitch motor speed because the pitch
  motor is installed reversed.
- Vision `err_x > 0` means target is right of center.
- Vision `err_y > 0` means target is below center.

## FreeRTOS Tasks

| Task | Period | Priority | Responsibility |
| --- | ---: | --- | --- |
| `defaultTask` | CubeMX default | normal | Kept empty. |
| `knx_app` | 1 ms loop shell | normal | Initializes board/modules and runs lightweight app/test shell. |
| `knx_comm` | 1 ms | above normal | Drains USART2 RX ring buffer and feeds host protocol parser. |
| `knx_safety` | 1 ms | high | Checks gimbal safety and latches faults. |
| `knx_ctrl` | 1 ms | above normal | Runs gimbal controller unless safety is FAULT. |
| `knx_ui` | 1 ms | below normal | Beep, key scan, grayscale update, debug LED. |
| `knx_telemetry` | 50 ms | low | Sends OctoLink debug variables. |

## Control Flow

`KNX_ACTIVE_TEST_MODE` is currently `KNX_ACTIVE_TEST_MODE_NONE`, so the formal
gimbal path is active.

Startup sequence:

1. Wait `knx_gimbal_ctrl_boot_delay_ms`.
2. Send `JC_EnterServo()` to yaw ID 3 and pitch ID 1.
3. Send `JC_SwitchMode(..., JC_MODE_SPEED)`.
4. Enter `CONTROL`.

Control behavior:

- If no fresh vision frame is available, hold yaw and pitch at 0 deg using IMU
  angle feedback.
- If a fresh frame arrives, run pixel-error PID centering.
- Yaw command uses `-err_x` because positive yaw moves left.
- Pitch command uses `err_y` as the desired pitch-angle direction, then applies
  `knx_gimbal_ctrl_pitch_speed_to_angle_sign = -1.0f` before sending motor RPM.
- Software control limits are `+-90 deg` yaw and `+-45 deg` pitch.
- The control task runs at 1 kHz; PID output is generated every
  `knx_gimbal_ctrl_control_period_ms` ms, default 10 ms.

## Safety

With `KNX_MODULE_GIMBAL_EN = 1`, safety uses the gimbal-specific branch.

Faults latch until `knx_safety_clear_faults()` is called.

Fault conditions:

- IMU missing/stale after `knx_safety_gimbal_startup_grace_ms`.
- Yaw or pitch exceeds the safety fault angle.
- Commanded yaw or pitch speed exceeds the safety command RPM threshold.
- JC command status is still negative after startup grace once servo entry has
  begun.

Warnings do not stop the controller:

- Startup grace active while IMU is not ready.
- Vision frame is missing/stale.
- Safety check is disabled by `knx_safety_gimbal_enable <= 0`.

When safety enters FAULT, `knx_ctrl` stops calling the controller update and
only asks the gimbal module to send throttled zero-speed commands.
If `knx_safety_gimbal_enable <= 0`, no new gimbal safety fault is generated, but
the warning bit stays set. Already-latched faults still need
`knx_safety_clear_faults()`.

## OctoLink Variables

Gimbal base: `280`

| ID | Type | Meaning |
| ---: | --- | --- |
| 280 | U32 | controller clock ms |
| 281 | F32 | gimbal state: 0 boot, 1 enter servo, 2 speed mode, 3 control |
| 282 | F32 | vision err_x |
| 283 | F32 | vision err_y |
| 284 | F32 | vision confidence |
| 285 | F32 | vision valid/fresh |
| 286 | F32 | yaw PID error px |
| 287 | F32 | pitch PID error px |
| 288 | F32 | yaw command rpm |
| 289 | F32 | pitch command rpm |
| 290 | U32 | vision rx count |
| 291 | F32 | vision age ms |
| 292 | I32 | last JC status |
| 293 | U32 | vision payload length errors |
| 294 | U32 | vision msg_id errors |
| 295 | U32 | gimbal update count |
| 298 | F32 | IMU yaw_total deg |
| 299 | F32 | IMU pitch deg |
| 300 | F32 | yaw hold error deg |
| 301 | F32 | pitch hold error deg |
| 302 | F32 | tracking active |
| 303 | F32 | yaw limit active |
| 304 | F32 | pitch limit active |
| 305 | F32 | yaw speed-to-angle sign |
| 306 | F32 | pitch speed-to-angle sign, currently -1 |

Vision base: `320`

| ID | Type | Meaning |
| ---: | --- | --- |
| 320 | F32 | err_x |
| 321 | F32 | err_y |
| 322 | F32 | confidence |
| 323 | F32 | valid |
| 324 | F32 | age ms |
| 325 | U32 | valid payload rx count |
| 326 | U32 | payload length errors |
| 327 | U32 | msg_id errors |
| 328 | U32 | host protocol frames |
| 329 | U32 | host protocol CRC errors |
| 330 | U32 | host protocol sync losses |

Host USART2 base: `340`

| ID | Type | Meaning |
| ---: | --- | --- |
| 340 | U32 | UART RX interrupt byte count |
| 341 | U32 | UART RX ring overflow count |
| 342 | U32 | UART RX restart errors |

Safety base: `360`

| ID | Type | Meaning |
| ---: | --- | --- |
| 360 | F32 | safety level: 0 ok, 1 warn, 2 fault |
| 361 | U32 | latched fault flags |
| 362 | U32 | warning flags |
| 363 | F32 | IMU age ms |
| 364 | F32 | vision age ms |
| 365 | U32 | safety update count |
| 366 | F32 | startup grace ms |
| 367 | F32 | IMU timeout ms |
| 368 | F32 | yaw safety fault deg |
| 369 | F32 | pitch safety fault deg |

Fault flags:

- Bit 0: overcurrent, used by chassis branch.
- Bit 1: IMU fail.
- Bit 2: encoder fail, reserved.
- Bit 3: watchdog / JC command status fault.
- Bit 4: tilt / angle fault.
- Bit 5: overspeed.
- Bit 6: gimbal software angle limit exceeded.
- Bit 7: gimbal command exceeds safety RPM threshold.

Warning flags:

- Bit 0: startup grace.
- Bit 1: vision stale or not received.
- Bit 2: gimbal safety disabled.
