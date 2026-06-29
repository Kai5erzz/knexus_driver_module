/* GPIO implementation for STM32H743 */
#include "knx_gpio.h"
#include "stm32h7xx_hal.h"

void knx_gpio_write(knx_gpio_t gpio, knx_gpio_state_t state)
{
    if (gpio.port == NULL) return;
    GPIO_TypeDef *port = (GPIO_TypeDef *)gpio.port;
    HAL_GPIO_WritePin(port, gpio.pin,
                      (state == KNX_GPIO_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

knx_gpio_state_t knx_gpio_read(knx_gpio_t gpio)
{
    if (gpio.port == NULL) return KNX_GPIO_LOW;
    GPIO_TypeDef *port = (GPIO_TypeDef *)gpio.port;
    return (HAL_GPIO_ReadPin(port, gpio.pin) == GPIO_PIN_SET)
           ? KNX_GPIO_HIGH : KNX_GPIO_LOW;
}

void knx_gpio_toggle(knx_gpio_t gpio)
{
    if (gpio.port == NULL) return;
    GPIO_TypeDef *port = (GPIO_TypeDef *)gpio.port;
    HAL_GPIO_TogglePin(port, gpio.pin);
}
