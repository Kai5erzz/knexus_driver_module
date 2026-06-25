/**
 * @file    knx_gpio_mspm0.c
 * @brief   GPIO platform implementation for MSPM0 (TI DriverLib)
 */

#include "knx_gpio.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

void knx_gpio_write(knx_gpio_t gpio, knx_gpio_state_t state)
{
    if (gpio.port == NULL) return;
    GPIO_Regs *port = (GPIO_Regs *)gpio.port;
    uint32_t pin = (uint32_t)gpio.pin;

    if (state == KNX_GPIO_HIGH) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

knx_gpio_state_t knx_gpio_read(knx_gpio_t gpio)
{
    if (gpio.port == NULL) return KNX_GPIO_LOW;
    GPIO_Regs *port = (GPIO_Regs *)gpio.port;
    uint32_t pin = (uint32_t)gpio.pin;

    return (DL_GPIO_readPins(port, pin) != 0U)
           ? KNX_GPIO_HIGH : KNX_GPIO_LOW;
}

void knx_gpio_toggle(knx_gpio_t gpio)
{
    if (gpio.port == NULL) return;
    GPIO_Regs *port = (GPIO_Regs *)gpio.port;
    uint32_t pin = (uint32_t)gpio.pin;

    DL_GPIO_togglePins(port, pin);
}
