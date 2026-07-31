#include "OLED.h"
#include "OLED_Font.h"
#include "knexus_config.h"
#include "knx_custom_i2c_guard.h"
#include "knx_time.h"
#include <stddef.h>
#include <string.h>

#define OLED_ADDRESS_8BIT       ((uint8_t)(KNEXUS_H_OLED_I2C_ADDRESS << 1U))
#define OLED_WIDTH              128U
#define OLED_PAGES              8U
#define OLED_FONT_WIDTH         8U
#define OLED_FONT_HEIGHT_PAGES  2U

static bool s_ready;
static uint32_t s_error_count;
static uint8_t s_transfer[1U + OLED_WIDTH];

#if defined(KNX_PLATFORM_STM32)

#include "i2c.h"

static void oled_bus_init(void)
{
    /* I2C1 and PB8/PB9 are initialized before the RTOS starts. */
}

static bool oled_bus_write(uint8_t control,
                           const uint8_t *data,
                           uint16_t length)
{
    if (data == NULL || length == 0U || length > OLED_WIDTH) return false;
    if (!knx_custom_i2c_try_lock()) return false;
    s_transfer[0] = control;
    memcpy(&s_transfer[1], data, length);
    if (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS_8BIT,
                                s_transfer, (uint16_t)(length + 1U),
                                KNEXUS_H_OLED_I2C_TIMEOUT_MS) != HAL_OK) {
        knx_custom_i2c_unlock();
        s_error_count++;
        return false;
    }
    knx_custom_i2c_unlock();
    return true;
}

#elif defined(KNX_PLATFORM_MSPM0)

#include "ti_msp_dl_config.h"
#include <ti/driverlib/driverlib.h>

/* The carrier I2C_CUSTOM net maps to PA4=SCL and PA3=SDA on MSPM0. */
#define OLED_MSP_PORT       OLED_SCL_PORT
#define OLED_MSP_SCL_PIN    OLED_SCL_SCL_PIN_PIN
#define OLED_MSP_SDA_PIN    OLED_SDA_SDA_PIN_PIN
#define OLED_MSP_SCL_IOMUX  OLED_SCL_SCL_PIN_IOMUX
#define OLED_MSP_SDA_IOMUX  OLED_SDA_SDA_PIN_IOMUX

static void oled_delay_us(uint32_t us)
{
    volatile uint32_t loops = us * (CPUCLK_FREQ / 6000000U);
    if (loops == 0U) loops = 1U;
    while (loops-- > 0U) __NOP();
}

/* Open-drain emulation: output latch stays low; disable output to release high. */
static void oled_scl(bool high)
{
    if (high) DL_GPIO_disableOutput(OLED_MSP_PORT, OLED_MSP_SCL_PIN);
    else DL_GPIO_enableOutput(OLED_MSP_PORT, OLED_MSP_SCL_PIN);
}

static void oled_sda(bool high)
{
    if (high) DL_GPIO_disableOutput(OLED_MSP_PORT, OLED_MSP_SDA_PIN);
    else DL_GPIO_enableOutput(OLED_MSP_PORT, OLED_MSP_SDA_PIN);
}

static void oled_start(void)
{
    oled_sda(true);
    oled_scl(true);
    oled_delay_us(2U);
    oled_sda(false);
    oled_delay_us(2U);
    oled_scl(false);
}

static void oled_stop(void)
{
    oled_sda(false);
    oled_delay_us(2U);
    oled_scl(true);
    oled_delay_us(2U);
    oled_sda(true);
    oled_delay_us(2U);
}

static bool oled_send_byte(uint8_t byte)
{
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
        oled_sda((byte & 0x80U) != 0U);
        oled_delay_us(1U);
        oled_scl(true);
        oled_delay_us(2U);
        oled_scl(false);
        byte <<= 1U;
    }
    oled_sda(true);
    oled_delay_us(1U);
    oled_scl(true);
    oled_delay_us(2U);
    bool ack = (DL_GPIO_readPins(OLED_MSP_PORT, OLED_MSP_SDA_PIN) == 0U);
    oled_scl(false);
    return ack;
}

static void oled_bus_init(void)
{
    /* Input buffer is required for ACK sampling; DOE is toggled for OD drive. */
    DL_GPIO_initDigitalInput(OLED_MSP_SCL_IOMUX);
    DL_GPIO_initDigitalInput(OLED_MSP_SDA_IOMUX);
    DL_GPIO_clearPins(OLED_MSP_PORT, OLED_MSP_SCL_PIN | OLED_MSP_SDA_PIN);
    DL_GPIO_disableOutput(OLED_MSP_PORT, OLED_MSP_SCL_PIN | OLED_MSP_SDA_PIN);
}

static bool oled_bus_write(uint8_t control,
                           const uint8_t *data,
                           uint16_t length)
{
    if (data == NULL || length == 0U || length > OLED_WIDTH) return false;
    if (!knx_custom_i2c_try_lock()) return false;
    bool ack = true;
    oled_start();
    ack = oled_send_byte(OLED_ADDRESS_8BIT) && ack;
    ack = oled_send_byte(control) && ack;
    for (uint16_t i = 0U; i < length; ++i) {
        ack = oled_send_byte(data[i]) && ack;
    }
    oled_stop();
    knx_custom_i2c_unlock();
    if (!ack) s_error_count++;
    return ack;
}

#else
#error "OLED platform is not supported"
#endif

static bool oled_commands(const uint8_t *commands, uint16_t count)
{
    return oled_bus_write(0x00U, commands, count);
}

static bool oled_set_cursor(uint8_t page, uint8_t x)
{
    const uint8_t commands[3] = {
        (uint8_t)(0xB0U | page),
        (uint8_t)(0x10U | ((x & 0xF0U) >> 4U)),
        (uint8_t)(x & 0x0FU),
    };
    return oled_commands(commands, 3U);
}

bool OLED_Init(void)
{
#if !KNEXUS_H_OLED_ENABLE
    return false;
#else
    static const uint8_t init_commands[] = {
        0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
        0xA1U, 0xC8U, 0xDAU, 0x12U, 0x81U, 0xCFU, 0xD9U, 0xF1U,
        0xDBU, 0x30U, 0xA4U, 0xA6U, 0x8DU, 0x14U, 0xAFU,
    };

    s_ready = false;
    s_error_count = 0U;
    oled_bus_init();
    knx_delay_ms(20U);
    if (!oled_commands(init_commands, (uint16_t)sizeof(init_commands))) {
        return false;
    }
    s_ready = true;
    OLED_Clear();
    return s_ready;
#endif
}

void OLED_Clear(void)
{
    if (!s_ready) return;
    memset(&s_transfer[1], 0, OLED_WIDTH);
    for (uint8_t page = 0U; page < OLED_PAGES; ++page) {
        if (!oled_set_cursor(page, 0U) ||
            !oled_bus_write(0x40U, &s_transfer[1], OLED_WIDTH)) {
            s_ready = false;
            return;
        }
    }
}

void OLED_ShowChar(uint8_t line, uint8_t column, char ch)
{
    if (!s_ready || line < 1U || line > 4U ||
        column < 1U || column > 16U) return;
    if (ch < ' ' || ch > '~') ch = '?';

    uint8_t page = (uint8_t)((line - 1U) * OLED_FONT_HEIGHT_PAGES);
    uint8_t x = (uint8_t)((column - 1U) * OLED_FONT_WIDTH);
    const uint8_t *glyph = OLED_F8x16[(uint8_t)ch - (uint8_t)' '];
    if (!oled_set_cursor(page, x) ||
        !oled_bus_write(0x40U, glyph, OLED_FONT_WIDTH) ||
        !oled_set_cursor((uint8_t)(page + 1U), x) ||
        !oled_bus_write(0x40U, &glyph[OLED_FONT_WIDTH], OLED_FONT_WIDTH)) {
        s_ready = false;
    }
}

void OLED_ShowString(uint8_t line, uint8_t column, const char *text)
{
    if (text == NULL) return;
    while (*text != '\0' && column <= 16U) {
        OLED_ShowChar(line, column, *text++);
        column++;
    }
}

void OLED_ShowNum(uint8_t line, uint8_t column,
                  uint32_t number, uint8_t length)
{
    char digits[10];
    if (length == 0U) return;
    if (length > sizeof(digits)) length = sizeof(digits);
    for (uint8_t i = 0U; i < length; ++i) {
        digits[length - 1U - i] = (char)('0' + (number % 10U));
        number /= 10U;
    }
    for (uint8_t i = 0U; i < length; ++i) {
        OLED_ShowChar(line, (uint8_t)(column + i), digits[i]);
    }
}

bool OLED_IsReady(void)
{
    return s_ready;
}

uint32_t OLED_GetErrorCount(void)
{
    return s_error_count;
}
