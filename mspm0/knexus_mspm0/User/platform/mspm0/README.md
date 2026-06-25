# MSPM0 Platform Layer

Placeholder for the MSPM0 platform implementation.

## Required Files

- `knx_port_mspm0.c` - critical section and system reset
- `knx_time_mspm0.c` - SysTick or FreeRTOS-backed millis/micros
- `knx_gpio_mspm0.c` - GPIO via TI DriverLib
- `knx_pwm_mspm0.c` - PWM via timer/PWM peripherals
- `knx_adc_mspm0.c` - ADC12 implementation
- `knx_uart_mspm0.c` - UART implementation
- `knx_spi_mspm0.c` - SPI implementation

## Notes

- MSPM0 uses TI DriverLib, not STM32 HAL.
- Platform files implement the interfaces in `common/User/platform/include/`.
- Board-specific pin mapping goes in `targets/mspm0/User/board/knexus_mspm0/`.
