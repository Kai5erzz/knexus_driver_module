#include "knx_board.h"
#include "knx_project_config.h"
#include "bmi088.h"
#include "encoder.h"
#include "knx_beep.h"
#include "knx_key.h"
#include "knx_led.h"
#include "knx_uart.h"
#include "ti_msp_dl_config.h"
#include "drv8701e.h"
#include "knx_spi.h"
#include "knx_can.h"
#include "ir_line_sensor.h"
#include "ir_line_sensor_bus.h"
#include "knx_grayscale.h"

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

/* Left encoder: software quadrature via GPIO EXTI on PA8/PA9. */
static const knx_encoder_port_t s_encoder_left = {
    .timer   = (void *)1,
    .type    = KNX_ENCODER_MSPM0_EXTI,
    .channel = 0U,
    .period  = 0U,
};

/* Right encoder: hardware QEI on TIMG8, PA21/PA22. */
static const knx_encoder_port_t s_encoder_right = {
    .timer   = (void *)QEI_RIGHT_INST,
    .type    = KNX_ENCODER_MSPM0_QEI,
    .channel = 0U,
    .period  = 65535U,
};

/* ── Motor driver ports (DRV8701E) ── */
/* Reworked carrier wiring: left EN=PB8/SPI_CUSTOM_CS1, PHASE=PA7;
 * right EN=PA15, PHASE=PA30.  Isolate the original PA0/L1_EN connection
 * before flywiring PB8 to L1_EN. */
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

/* PB17/TIMA0 CC2 drives the active-high BMI088 heater MOSFET. */
static const knx_pwm_channel_t s_bmi088_heater_pwm = {
    .timer = (void *)PWM_HEATER_INST,
    .channel = 2U,
    .arr = 4000U,
};

/* ── UART for OctoLink ── */
static knx_uart_t s_debug_uart = {
    .handle   = (void *)UART_DEBUG_INST,
    .baudrate = 921600U,
};

static Octolinker_Instance_t s_octolinker;
static ir_line_sensor_t s_ir_line_sensor;

static const ir_line_sensor_config_t s_ir_line_sensor_config = {
    .link = IR_LINE_SENSOR_LINK_I2C,
    .context = NULL,
    .timeout_ms = 10U,
    .io.i2c.read_register = ir_line_sensor_board_i2c_read,
};

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

volatile uint32_t knx_mspm0_board_init_stage;

/* ── init ── */
knx_status_t knx_board_init(void)
{
    knx_mspm0_board_init_stage = 1U;
    knx_led_attach_ports(s_led_ports, KNX_LED_MAX_LEDS);
    knx_led_init();

    knx_mspm0_board_init_stage = 2U;
    knx_key_attach_ports(s_key_ports, 2U);
    knx_key_init();

    knx_beep_attach_port(s_beep_gpio);
    knx_beep_init();

    Octolinker_Init(&s_octolinker, &s_debug_uart);
    knx_mspm0_board_init_stage = 3U;

    /* Available in every application profile, not only contest_2026. */
    ir_line_sensor_board_i2c_init();
    (void)ir_line_sensor_init(&s_ir_line_sensor, &s_ir_line_sensor_config);
    knx_grayscale_attach_sensor(&s_ir_line_sensor);

#if KNX_APP_CONTEST_2026
    /* Contest runtime expects board_init() to complete all physical bindings,
     * matching the STM32 board contract. Module control loops start later. */
    DRV8701E_AttachPorts(&s_drv_left, &s_drv_right);
    DRV8701E_Init();
    knx_mspm0_board_init_stage = 4U;
    (void)Encoder_AttachPorts(&s_encoder_left, &s_encoder_right);
    (void)Encoder_Init();
    knx_mspm0_board_init_stage = 5U;
    knx_mspm0_board_init_stage = 6U;
    (void)knx_spi_init(&s_bmi088_accel_spi);
    (void)knx_spi_init(&s_bmi088_gyro_spi);
    knx_mspm0_board_init_stage = 7U;
    /* Start conservatively on the 12 V heater rail; tune upward from data. */
    bmi088_heater_enable = 1.0f;
    bmi088_temp_target_c = 40.0f;
    bmi088_heater_duty_max = 0.18f;
    BMI088_AttachHeater(&s_bmi088_heater_pwm);
    BMI088_Init(&s_bmi088_accel_spi, &s_bmi088_gyro_spi);
    knx_mspm0_board_init_stage = 8U;
#endif

    knx_mspm0_board_init_stage = 9U;
    return KNX_OK;
}

void knx_board_post_init(void)
{
    /* Enable GPIOA GROUP1 interrupt for left-encoder edges.
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
knx_can_t       *knx_board_get_can_bus(uint8_t index) { return (index == 0U) ? &s_can : NULL; }
ir_line_sensor_t *knx_board_get_ir_line_sensor(void) { return &s_ir_line_sensor; }

/* ── STM32 API parity stubs — MSPM0 has a single MCAN instance ── */
knx_can_t       *knx_board_get_jc_can(void)           { return &s_can; }
knx_can_t       *knx_board_get_dm_imu_l1_can(void)    { return &s_can; }
