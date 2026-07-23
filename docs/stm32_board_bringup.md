# STM32 New-Baseboard Bring-Up Test

The STM32 build currently selects `KNX_ACTIVE_TEST_MODE_STM32_BOARD_BRINGUP`.
This is a board-acceptance firmware image, not the normal gimbal application.

## Safety and controls

- Motor outputs are stopped after boot.
- Hold **KEY0** for one second to arm or disarm the motor test.
- Press **KEY1** to stop and disarm the motors immediately.
- The default maximum duty is 30%; it is capped in firmware at 30%.
- When armed, the forward-only test steps through left 10/20/30%, right
  10/20/30%, then both 20% and 30%. Each segment lasts two seconds.

## CAN analyser test

Both interfaces use classic CAN, 1 Mbps, 8-byte frames every 100 ms.

| Bus | Pins | Standard ID | Payload |
| --- | --- | ---: | --- |
| FDCAN1 | PA11 / PA12 | `0x701` | `4B 4E 58 42 seq_lo seq_hi armed 01` |
| FDCAN2 | PB12 / PB13 | `0x702` | `4B 4E 58 42 seq_lo seq_hi armed 02` |

`4B 4E 58 42` is ASCII `KNXB`. A CAN analyser should provide a correctly
terminated bus and acknowledge frames; otherwise the TX FIFO will eventually
fill and the failure counters will rise.

## OctoLink diagnostics

The existing BMI088 driver publishes IDs `20..25`:

| ID | Value |
| ---: | --- |
| 20..22 | Roll, pitch, yaw |
| 23 | BMI088 temperature (C) |
| 24 | Heater target temperature (C) |
| 25 | Heater PWM duty |

The board test publishes these IDs every 100 ms:

| ID | Value |
| ---: | --- |
| 600 | Uptime (ms) |
| 601 | Motor armed |
| 602..604 | Left duty, right duty, configured maximum duty |
| 605..606 | KEY0 / KEY1 pressed |
| 607..608 | LED1 / LED2 state |
| 609..611 | FDCAN1 last status, TX OK count, TX failure count |
| 612..614 | FDCAN2 last status, TX OK count, TX failure count |
| 615..617 | BMI088 accelerometer OK, gyroscope OK, frame count |
| 618 | CAN sequence number |
| 619 | BMI088 temperature (C), duplicated for the test dashboard |
