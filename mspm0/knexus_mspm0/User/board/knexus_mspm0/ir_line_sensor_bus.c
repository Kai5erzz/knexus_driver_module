#include "ir_line_sensor_bus.h"

#include "knx_custom_i2c_guard.h"
#include "ti_msp_dl_config.h"
#include <ti/driverlib/driverlib.h>
#include <stdbool.h>

#define BUS_PORT       OLED_SCL_PORT
#define BUS_SCL_PIN    OLED_SCL_SCL_PIN_PIN
#define BUS_SDA_PIN    OLED_SDA_SDA_PIN_PIN
#define BUS_SCL_IOMUX  OLED_SCL_SCL_PIN_IOMUX
#define BUS_SDA_IOMUX  OLED_SDA_SDA_PIN_IOMUX

static void delay_us(uint32_t us)
{
    volatile uint32_t loops = us * (CPUCLK_FREQ / 6000000U);
    if (loops == 0U) loops = 1U;
    while (loops-- > 0U) __NOP();
}

static void scl(bool high)
{
    if (high) DL_GPIO_disableOutput(BUS_PORT, BUS_SCL_PIN);
    else DL_GPIO_enableOutput(BUS_PORT, BUS_SCL_PIN);
}

static void sda(bool high)
{
    if (high) DL_GPIO_disableOutput(BUS_PORT, BUS_SDA_PIN);
    else DL_GPIO_enableOutput(BUS_PORT, BUS_SDA_PIN);
}

static bool read_sda(void)
{
    return DL_GPIO_readPins(BUS_PORT, BUS_SDA_PIN) != 0U;
}

static void start_condition(void)
{
    sda(true); scl(true); delay_us(2U);
    sda(false); delay_us(2U); scl(false);
}

static void stop_condition(void)
{
    sda(false); delay_us(2U); scl(true); delay_us(2U);
    sda(true); delay_us(2U);
}

static bool write_byte(uint8_t byte)
{
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
        sda((byte & 0x80U) != 0U); delay_us(1U);
        scl(true); delay_us(2U); scl(false);
        byte <<= 1U;
    }
    sda(true); delay_us(1U); scl(true); delay_us(2U);
    bool ack = !read_sda();
    scl(false);
    return ack;
}

static uint8_t read_byte(bool acknowledge)
{
    uint8_t byte = 0U;
    sda(true);
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
        byte <<= 1U;
        scl(true); delay_us(2U);
        if (read_sda()) byte |= 1U;
        scl(false); delay_us(1U);
    }
    sda(!acknowledge); delay_us(1U);
    scl(true); delay_us(2U); scl(false);
    sda(true);
    return byte;
}

void ir_line_sensor_board_i2c_init(void)
{
    DL_GPIO_initDigitalInput(BUS_SCL_IOMUX);
    DL_GPIO_initDigitalInput(BUS_SDA_IOMUX);
    DL_GPIO_clearPins(BUS_PORT, BUS_SCL_PIN | BUS_SDA_PIN);
    DL_GPIO_disableOutput(BUS_PORT, BUS_SCL_PIN | BUS_SDA_PIN);
}

knx_status_t ir_line_sensor_board_i2c_read(
    void *context, uint8_t address_7bit, uint8_t reg,
    uint8_t *data, size_t length, uint32_t timeout_ms)
{
    (void)context;
    (void)timeout_ms;
    if (data == NULL || length == 0U) return KNX_INVALID_ARG;
    if (!knx_custom_i2c_try_lock()) return KNX_BUSY;

    bool ack = true;
    start_condition();
    ack = write_byte((uint8_t)(address_7bit << 1U)) && ack;
    ack = write_byte(reg) && ack;
    start_condition();
    ack = write_byte((uint8_t)((address_7bit << 1U) | 1U)) && ack;
    if (ack) {
        for (size_t i = 0U; i < length; ++i) {
            data[i] = read_byte(i + 1U < length);
        }
    }
    stop_condition();
    knx_custom_i2c_unlock();
    return ack ? KNX_OK : KNX_ERROR;
}
