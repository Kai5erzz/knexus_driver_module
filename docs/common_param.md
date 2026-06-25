# Common Parameter Registry

`knx_param` is a small common-layer registry for tunable runtime parameters.
Modules register static tables. Callers can then read, write, clamp, and reset
parameters by stable numeric ID.

## API

- `knx_param_init()`
- `knx_param_register_table(entries, count)`
- `knx_param_find_by_id(id)`
- `knx_param_find_by_name(name)`
- `knx_param_get_*()`
- `knx_param_set_*()`
- `knx_param_reset(id)`
- `knx_param_reset_all()`

Supported types:

- `F32`
- `U32`
- `I32`
- `U8`

Each entry has:

- stable ID
- name
- storage pointer
- default value
- min/max clamp
- flags

## Gimbal Parameter IDs

Base: `1000`

| ID | Name | Meaning |
| ---: | --- | --- |
| 1000 | `gimbal.enable` | controller enable |
| 1001 | `gimbal.boot_delay_ms` | boot delay before servo commands |
| 1002 | `gimbal.mode_gap_ms` | delay between JC mode commands |
| 1003 | `gimbal.control_period_ms` | PID output period |
| 1004 | `gimbal.vision_timeout_ms` | vision freshness timeout |
| 1005 | `gimbal.min_conf` | minimum vision confidence |
| 1006 | `gimbal.yaw.kp` | visual yaw PID Kp |
| 1007 | `gimbal.yaw.ki` | visual yaw PID Ki |
| 1008 | `gimbal.yaw.kd` | visual yaw PID Kd |
| 1009 | `gimbal.pitch.kp` | visual pitch PID Kp |
| 1010 | `gimbal.pitch.ki` | visual pitch PID Ki |
| 1011 | `gimbal.pitch.kd` | visual pitch PID Kd |
| 1012 | `gimbal.hold_yaw.kp` | zero-hold yaw PID Kp |
| 1013 | `gimbal.hold_yaw.ki` | zero-hold yaw PID Ki |
| 1014 | `gimbal.hold_yaw.kd` | zero-hold yaw PID Kd |
| 1015 | `gimbal.hold_pitch.kp` | zero-hold pitch PID Kp |
| 1016 | `gimbal.hold_pitch.ki` | zero-hold pitch PID Ki |
| 1017 | `gimbal.hold_pitch.kd` | zero-hold pitch PID Kd |
| 1018 | `gimbal.output_limit_rpm` | visual tracking speed limit |
| 1019 | `gimbal.hold_output_limit_rpm` | zero-hold speed limit |
| 1020 | `gimbal.integral_limit` | shared PID integral clamp |
| 1021 | `gimbal.yaw_target_deg` | hold target yaw |
| 1022 | `gimbal.pitch_target_deg` | hold target pitch |
| 1023 | `gimbal.yaw_limit_deg` | yaw software limit |
| 1024 | `gimbal.pitch_limit_deg` | pitch software limit |
| 1025 | `gimbal.stop_period_ms` | fault stop command period |
| 1026 | `gimbal.yaw_speed_to_angle_sign` | yaw motor direction sign |
| 1027 | `gimbal.pitch_speed_to_angle_sign` | pitch motor direction sign |

## Host Protocol

Parameter access is exposed through `knx_param_host` on top of the fixed
host frame format.

Requests:

- `0x30` GET: `[msg, id_lo, id_hi]`
- `0x31` SET: `[msg, id_lo, id_hi, type, raw0, raw1, raw2, raw3]`
- `0x32` RESET: `[msg, id_lo, id_hi]`
- `0x33` INFO: `[msg, index_lo, index_hi]`

Responses:

- `0xB0` VALUE
- `0xB1` INFO
- `0xB2` ACK

Octo debug base `380`:

| Offset | Meaning |
| ---: | --- |
| 0 | parameter host RX count |
| 1 | parameter host TX count |
| 2 | parameter host error count |
| 3 | last status |
| 4 | last parameter ID |
| 5 | last message ID |
