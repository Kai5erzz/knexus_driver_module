# Common Runtime Modules

This file tracks common-layer runtime helpers used by the STM32 gimbal build.

## CAN Router

`modules/can_router` owns one low-level CAN RX callback per bus and dispatches
frames to registered ID ranges.

Current board routing:

- FDCAN2: JC driver replies, `0x581..0x5FF`
- FDCAN1: DM-IMU-L1 active reports, `0x11`

The JC and DM-IMU drivers keep their legacy `*_Init()` APIs. Board init uses
the external RX init variants so the router owns `knx_can_set_rx_callback()`.

## Ring Buffer

`util/knx_ringbuf` is the shared byte FIFO. USART2 host RX uses it instead of a
board-local head/tail implementation.

Board host Octo base `340`:

| Offset | Meaning |
| ---: | --- |
| 0 | RX IRQ push count |
| 1 | RX overflow count |
| 2 | RX restart error count |

## Health

`modules/health` stores the latest state of each important subsystem.

Source IDs:

| ID | Source |
| ---: | --- |
| 0 | sys |
| 1 | safety |
| 2 | vision |
| 3 | DM-IMU-L1 |
| 4 | gimbal |
| 5 | JC driver |
| 6 | host comm |
| 7 | parameter host |

State values:

| Value | State |
| ---: | --- |
| 0 | unknown |
| 1 | ok |
| 2 | warn |
| 3 | fault |
| 4 | stale |
| 5 | disabled |

Octo debug base `400`:

| Offset | Meaning |
| ---: | --- |
| 0 | overall state |
| 1 | source count |
| 10 + source | source state |
| 30 + source | source age in ms |
| 50 + source | source status |
| 70 + source | source flags |
| 90 + source | source error count |
| 110 + source | source update count |

## Blackbox

`modules/blackbox` keeps the last 32 events in RAM. It logs boot, health
changes, CAN unmatched frames, and safety clear operations.

Event codes:

| Code | Event |
| ---: | --- |
| 1 | boot |
| 2 | health change |
| 3 | CAN unmatched frame |
| 4 | safety clear |

Octo debug base `540`:

| Offset | Meaning |
| ---: | --- |
| 0 | stored event count |
| 1 | overwritten event count |
| 2 | write index |
| 3 | latest timestamp |
| 4 | latest code |
| 5 | latest source |
| 6 | latest status |
| 7 | latest arg0 |
| 8 | latest arg1 |
| 20..23 | latest 4 event codes |
| 30..33 | latest 4 event sources |
| 40..43 | latest 4 event statuses |

## Control Utilities

`util/knx_control` provides small reusable control helpers:

- absolute float limiting
- float-to-ms period conversion with fallback
- lightweight PID state for modules that do not need the full legacy PID stack

Current users:

- gimbal control PID loops
- safety timeout conversion
- drive command limiting
