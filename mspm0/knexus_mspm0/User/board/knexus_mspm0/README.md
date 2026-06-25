# knexus_mspm0 Board Layer

Placeholder for the MSPM0-based knexus board.

## Required Files

- `knx_board_config.h` - target MCU, clock, and peripheral pin mapping
- `knx_board.h` - board init/post-init declarations
- `knx_board.c` - board init implementation

## Notes

- Pin mapping will differ from the STM32H743 variant.
- Both boards expose the same `knx_board_init()` and `knx_board_post_init()` API.
- Peripheral handles use MSPM0 DriverLib/SysConfig types.
