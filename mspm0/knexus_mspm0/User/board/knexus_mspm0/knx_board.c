#include "knx_board.h"
#include "knx_beep.h"
#include "knx_key.h"
#include "knx_led.h"
#include "knx_uart.h"
#include "ti_msp_dl_config.h"
#include "drv8701e.h"
#include "knx_spi.h"
#include "knx_can.h"
#include "track_sensor.h"

/* ── LED ports ── */
static const knx_led_port_t s_led_ports[KNX_LED_MAX_LEDS] = {
    [KNX_LED_0] = {
        .gpio       = { .port = (void *)LED0_PORT, .pin = LED0_LED0_PIN_PIN },
        .active_low = 1U,
    },
    [KNX_LED_1] = {
        .gpio       = { .port = (void *)LED1_PORT, .pin = LED1_LED1_PIN_PIN },
        .active_low = 1U,
    },
    [KNX_LED_2] = {
        .gpio       = { .port = (void *)LED2_PORT, .pin = LED2_LED2_PIN_PIN },
        .active_low = 1U,
    },
};

/* ── KEY ports ── */
static const knx_key_port_t s_key_ports[2] = {
    [KNX_KEY_0] = {
        .gpio       = { .port = (void *)KEY0_PORT, .pin = KEY0_KEY0_PIN_PIN },
        .active_low = 0U,
    },
    [KNX_KEY_1] = {
        .gpio       = { .port = (void *)KEY1_PORT, .pin = KEY1_KEY1_PIN_PIN },
        .active_low = 0U,
    },
};

/* ── BEEP ── */
static const knx_gpio_t s_beep_gpio = {
    .port = (void *)BEEP_PORT,
    .pin  = BEEP_BEEP0_PIN_PIN,
};

/* ── Debug LED ── */
static const knx_gpio_t s_debug_led = {
    .port = (void *)LED0_PORT,
    .pin  = LED0_LED0_PIN_PIN,
};

/* ── Left encoder: hardware QEI on TIMG8 ── */
static const knx_encoder_port_t s_encoder_left = {
    .timer   = (void *)QEI_LEFT_INST,
    .type    = KNX_ENCODER_MSPM0_QEI,
    .channel = 0U,
    .period  = 65535U,
};

/* ── Right encoder: software quadrature via GPIO EXTI ── */
static const knx_encoder_port_t s_encoder_right = {
    .timer   = (void *)1,       /* non-NULL sentinel — not dereferenced */
    .type    = KNX_ENCODER_MSPM0_EXTI,
    .channel = 0U,
    .period  = 0U,
};

/* ── Motor driver ports (DRV8701E) ── */
static drv8701e_port_t s_drv_left = {
    .pwm        = { .timer = (void *)PWM_L_INST,        .channel = 0, .arr = 4000 },
    .phase_gpio = { .port  = (void *)MOTOR_L_PH_PORT,   .pin     = MOTOR_L_PH_ML_PHASE_PIN },
    .current_adc = { .adc  = (void *)ADC_MOTOR_INST,    .timeout_ms = 1 },
    .invert     = 0,
};

static drv8701e_port_t s_drv_right = {
    .pwm        = { .timer = (void *)PWM_R_INST,        .channel = 0, .arr = 4000 },
    .phase_gpio = { .port  = (void *)MOTOR_R_PH_PORT,   .pin     = MOTOR_R_PH_MR_PHASE_PIN },
    .current_adc = { .adc  = (void *)ADC_MOTOR_INST,    .timeout_ms = 1 },
    .invert     = 0,
};

/* ── UART for OctoLink ── */
static knx_uart_t s_debug_uart = {
    .handle   = (void *)UART_DEBUG_INST,
    .baudrate = 921600U,
};

static Octolinker_Instance_t s_octolinker;

/* ── BMI088 SPI ── */
static knx_spi_t s_bmi088_accel_spi = {
    .handle  = (void *)SPI0,
    .cs_gpio = { .port = (void *)GPIOA, .pin = DL_GPIO_PIN_25 },
};

static knx_spi_t s_bmi088_gyro_spi = {
    .handle  = (void *)SPI0,
    .cs_gpio = { .port = (void *)GPIOA, .pin = DL_GPIO_PIN_24 },
};

/* ── MCAN ── */
static knx_can_t s_can = {
    .handle = (void *)CANFD0,
};

/* ── Track sensor ── */
static track_sensor_port_t s_track_sensor = {
    .mux_ad0 = { .port = (void *)GPIOB, .pin = DL_GPIO_PIN_4 },
    .mux_ad1 = { .port = (void *)GPIOB, .pin = DL_GPIO_PIN_5 },
    .mux_ad2 = { .port = (void *)GPIOB, .pin = DL_GPIO_PIN_7 },
    .adc     = { .adc  = (void *)ADC0,  .timeout_ms = 1 },
};

/* ── init ── */
knx_status_t knx_board_init(void)
{
    knx_led_attach_ports(s_led_ports, KNX_LED_MAX_LEDS);
    knx_led_init();

    knx_key_attach_ports(s_key_ports, 2U);
    knx_key_init();

    knx_beep_attach_port(s_beep_gpio);
    knx_beep_init();

    Octolinker_Init(&s_octolinker, &s_debug_uart);

    return KNX_OK;
}

void knx_board_post_init(void)
{
    /* Enable GPIOA GROUP1 interrupt for right-encoder edges.
     * The encoder GPIO + polarity was configured in SYSCFG_DL_GPIO_init(). */
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

/* ── getters ── */
const knx_gpio_t *knx_board_get_debug_led(void)        { return &s_debug_led; }
const knx_encoder_port_t *knx_board_get_encoder_left(void)  { return &s_encoder_left; }
const knx_encoder_port_t *knx_board_get_encoder_right(void) { return &s_encoder_right; }
Octolinker_Instance_t    *knx_board_get_octolinker(void)    { return &s_octolinker; }
const drv8701e_port_t *knx_board_get_drv_left_port(void)  { return &s_drv_left; }
const drv8701e_port_t *knx_board_get_drv_right_port(void) { return &s_drv_right; }

/* ── New peripheral getters ── */
const knx_spi_t *knx_board_get_bmi088_accel_spi(void) { return &s_bmi088_accel_spi; }
const knx_spi_t *knx_board_get_bmi088_gyro_spi(void)  { return &s_bmi088_gyro_spi; }
knx_can_t       *knx_board_get_can(void)              { return &s_can; }
const track_sensor_port_t *knx_board_get_track_sensor_port(void) { return &s_track_sensor; }

/* ── STM32 API parity stubs — MSPM0 has a single MCAN instance ── */
knx_can_t       *knx_board_get_jc_can(void)           { return &s_can; }
knx_can_t       *knx_board_get_dm_imu_l1_can(void)    { return &s_can; }
