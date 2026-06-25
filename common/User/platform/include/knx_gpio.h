#ifndef KNX_GPIO_H
#define KNX_GPIO_H

#include "knx_types.h"

typedef enum {
    KNX_GPIO_LOW  = 0,
    KNX_GPIO_HIGH = 1,
} knx_gpio_state_t;

typedef struct {
    void     *port;
    uint32_t  pin;      /**< Platform-specific pin bitmask (uint16_t for STM32, uint32_t for MSPM0) */
} knx_gpio_t;

void             knx_gpio_write(knx_gpio_t gpio, knx_gpio_state_t state);
knx_gpio_state_t knx_gpio_read(knx_gpio_t gpio);
void             knx_gpio_toggle(knx_gpio_t gpio);

#endif /* KNX_GPIO_H */
