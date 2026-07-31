/* Board implementation for knexus_stm32h743 */
#include "knx_board.h"
#include "knx_board_config.h"
#include "knx_project_config.h"
#include "stm32h7xx_hal.h"
#include "drv8701e.h"
#include "bmi088.h"
#include "encoder.h"
#include "knx_encoder.h"
#include "ir_line_sensor.h"
#include "ir_line_sensor_bus.h"
#include "knx_grayscale.h"
#include "knx_led.h"
#include "knx_key.h"
#include "knx_beep.h"
#include "knx_ringbuf.h"
#if KNX_MODULE_GIMBAL_EN
#include "knx_can_router.h"
#include "jc_driver.h"
#include "dm_imu_l1.h"
#endif
#include "tim.h"
#include "lptim.h"
#include "adc.h"
#include "usart.h"
#include "spi.h"
#include "fdcan.h"

static const knx_gpio_t board_debug_led = {
    .port = GPIOH,
    .pin  = GPIO_PIN_7,
};

/* ---- BMI088 SPI platform-backed port definitions ----
 *
 * Both accel and gyro share SPI1 (hspi1), with separate CS lines:
 *   Accel CS: PH8  (SPI_BMI088_CS0)
 *   Gyro  CS: PI3  (SPI_BMI088_CS1)
 */

static knx_spi_t bmi088_accel_spi = {
    .handle  = &hspi1,
    .cs_gpio = { .port = GPIOH, .pin = GPIO_PIN_8 },
};

static knx_spi_t bmi088_gyro_spi = {
    .handle  = &hspi1,
    .cs_gpio = { .port = GPIOI, .pin = GPIO_PIN_3 },
};

/* OctoLink instance �?owned by board, shared with app/test modes */
static knx_uart_t board_debug_uart = {
    .handle = &huart1,
    .baudrate = 0,
};

static Octolinker_Instance_t board_octolinker;
static ir_line_sensor_t board_ir_line_sensor;

static const ir_line_sensor_config_t board_ir_line_sensor_config = {
    .link = IR_LINE_SENSOR_LINK_I2C,
    .context = NULL,
    .timeout_ms = 10U,
    .io.i2c.read_register = ir_line_sensor_board_i2c_read,
};

/* Host communication link: USART2 on PD5/PD6. */
static knx_uart_t board_host_uart = {
    .handle = &huart2,
    .baudrate = 0,
};

static knx_host_comm_t board_host_comm;
static uint8_t board_host_rx_byte;
static volatile uint8_t board_host_rx_last_byte;
static volatile uint32_t board_host_rx_restart_errors;
static volatile uint32_t board_host_uart_error_count;
static volatile uint32_t board_host_uart_error_code;
static volatile uint32_t board_host_uart_rx_state;
static volatile uint32_t board_host_uart_start_status;
volatile uint32_t knx_stm32_reset_cause;

#define BOARD_HOST_RX_RING_SIZE 512U
static uint8_t board_host_rx_ring[BOARD_HOST_RX_RING_SIZE];
static knx_ringbuf_t board_host_rx_rb;

static knx_can_t board_can1 = {
    .handle = &hfdcan1,
};
static knx_can_t board_can2 = {
    .handle = &hfdcan2,
};
#define board_jc_can board_can2

#if KNX_MODULE_GIMBAL_EN
static knx_can_router_t board_gimbal_can_router;

/* Kept as a semantic alias for callers that request the IMU CAN port. */
#define board_dm_imu_l1_can board_jc_can
#endif

/* ---- DRV8701E platform-backed port definitions ----
 *
 * Hardware topology: PHASE/EN mode
 *   Left motor : PWM=PA15(TIM2_CH1), PHASE=PB0,  current=PA4(ADC1 Rank2 CH18)
 *   Right motor: PWM=PA2(TIM2_CH3),  PHASE=PB11, current=PC0(ADC1 Rank1 CH10)
 *
 * ADC1 configuration: scan mode, 2 ranks, TIM2_TRGO external trigger, DMA circular.
 *   Rank 1: CH10 (PC0) - right motor current
 *   Rank 2: CH18 (PA4) - left motor current
 * Track sensing uses PC5 / ADC2_INP8 and must not be part of ADC1 current DMA.
 */

static const drv8701e_port_t drv_left_port = {
    .pwm = {
        .timer   = &htim2,
        .channel = TIM_CHANNEL_1,
        .arr     = 0,
    },
    .phase_gpio = {
        .port = GPIOB,
        .pin  = GPIO_PIN_0,
    },
    .current_adc = {
        .adc        = &hadc1,
        .timeout_ms = 1,
    },
    .invert = 0,
};

static const drv8701e_port_t drv_right_port = {
    .pwm = {
        .timer   = &htim2,
        .channel = TIM_CHANNEL_3,
        .arr     = 0,
    },
    .phase_gpio = {
        .port = GPIOB,
        .pin  = GPIO_PIN_11,
    },
    .current_adc = {
        .adc        = &hadc1,
        .timeout_ms = 1,
    },
    .invert = 0,
};

static const knx_pwm_channel_t drv_current_sample_trigger = {
    .timer   = &htim2,
    .channel = TIM_CHANNEL_4,
    .arr     = 0,
};

/* ---- Encoder platform-backed port definitions ----
 *
 * Left encoder : TIM1 - PA8 (CH1), PA9 (CH2) - 4x quadrature
 * Right encoder: LPTIM1 - PG12 (IN1), PG11 (IN2) - 2x external count
 */

static const knx_encoder_port_t enc_left_port = {
    .timer   = &htim1,
    .type    = KNX_ENCODER_TIM,
    .channel = TIM_CHANNEL_ALL,
    .period  = 0,
};

static const knx_encoder_port_t enc_right_port = {
    .timer   = &hlptim1,
    .type    = KNX_ENCODER_LPTIM,
    .channel = 0,
    .period  = 0xFFFF,
};

/* ---- BMI088 Heater PWM ----
 * TIM15 CH1 -> PE5, 高电平使能加热电�? * ARR=65535 (CubeMX 配置), .arr=0 �?knx_pwm_set_duty 从硬件读�? */
static const knx_pwm_channel_t bmi088_heater_pwm = {
    .timer   = &htim15,
    .channel = TIM_CHANNEL_1,
    .arr     = 0,
};

/* ---- LED port definitions ----
 *
 * LED0: PH7 - 心跳指示 (app 层管�? 也注册到驱动供统一控制)
 * LED1: PD3
 * LED2: PD4
 * 共阳设计, 低电平点�? */
static const knx_led_port_t led_ports[] = {
    [KNX_LED_0] = { .gpio = { .port = GPIOH, .pin = GPIO_PIN_7 },  .active_low = 1 },
    [KNX_LED_1] = { .gpio = { .port = GPIOD, .pin = GPIO_PIN_3 },  .active_low = 1 },
    [KNX_LED_2] = { .gpio = { .port = GPIOD, .pin = GPIO_PIN_4 },  .active_low = 1 },
};

/* ---- KEY port definitions ----
 *
 * KEY0: PE3  - 下拉, 高电平有�?(按下=HIGH)
 * KEY1: PG10 - 上拉, 低电平有�?(按下=LOW)
 */
static const knx_key_port_t key_ports[] = {
    [KNX_KEY_0] = { .gpio = { .port = GPIOE, .pin = GPIO_PIN_3  }, .active_low = 0 },  /* 下拉, 高有�? 按下=HIGH */
    [KNX_KEY_1] = { .gpio = { .port = GPIOG, .pin = GPIO_PIN_10 }, .active_low = 0 },  /* 下拉, 高有�? 按下=HIGH */
};

/* ---- BEEP ----
 * PA10, 推挽输出, 外部上拉
 * HIGH = �? LOW = 不响
 */
static const knx_gpio_t beep_gpio = {
    .port = GPIOA,
    .pin  = GPIO_PIN_10,
};

/* ============================================================ */

/* IWDG handled via direct register access — no HAL dependency needed */

static void knx_board_host_comm_start_rx(void)
{
    HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart2, &board_host_rx_byte, 1U);
    board_host_uart_start_status = (uint32_t)status;
    board_host_uart_rx_state = (uint32_t)huart2.RxState;
    board_host_uart_error_code = huart2.ErrorCode;

    if (status != HAL_OK) {
        board_host_rx_restart_errors++;
    }
}

static void knx_board_host_rx_push(uint8_t byte)
{
    board_host_rx_last_byte = byte;
    (void)knx_ringbuf_push_byte(&board_host_rx_rb, byte);
}

static void knx_board_force_fdcan1_pins(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void knx_board_recover_fdcan1_if_used(void)
{
    if (board_jc_can.handle != &hfdcan1) {
        return;
    }

    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_FDCAN_FORCE_RESET();
    __DSB();
    __HAL_RCC_FDCAN_RELEASE_RESET();
    __DSB();

    hfdcan1.State = HAL_FDCAN_STATE_RESET;
    hfdcan1.ErrorCode = 0U;
    hfdcan1.Instance = FDCAN1;
    knx_board_force_fdcan1_pins();
    MX_FDCAN1_Init();
    knx_board_force_fdcan1_pins();
}

#if KNX_MODULE_GIMBAL_EN
static void knx_board_jc_can_route(uint32_t std_id,
                                   const uint8_t *data,
                                   uint8_t len,
                                   void *user)
{
    (void)user;
    JC_RxHandler(std_id, data, len);
}

static void knx_board_dm_imu_l1_can_route(uint32_t std_id,
                                          const uint8_t *data,
                                          uint8_t len,
                                          void *user)
{
    (void)user;
    DM_IMU_L1_UpdateData(std_id, data, len);
}
#endif

knx_status_t knx_board_init(void)
{
    /* RSR会保留上一次复位来源。先保存供OctoLink诊断，再清标志。 */
    knx_stm32_reset_cause = RCC->RSR;
    RCC->RSR |= RCC_RSR_RMVF;
    /* IWDG在看门狗复位后仍可能继续运行，尽早重装计数器，避免传感器
     * 初始化期间再次复位。首次上电尚未启动IWDG时写该键无副作用。 */
    IWDG1->KR = 0xAAAAU;
    /* Init debug output */
    Octolinker_Init(&board_octolinker, &board_debug_uart);

    /* Init upper/lower host communication link. */
    knx_host_comm_init(&board_host_comm, &board_host_uart);
    (void)knx_ringbuf_init(&board_host_rx_rb,
                           board_host_rx_ring,
                           BOARD_HOST_RX_RING_SIZE);
    knx_board_host_comm_start_rx();

    /* Legacy gimbal ownership is excluded from the contest profile. */
#if KNX_MODULE_GIMBAL_EN
    knx_board_recover_fdcan1_if_used();
    JC_AttachCAN(&board_jc_can);
    DM_IMU_L1_AttachCAN(&board_dm_imu_l1_can, DM_IMU_L1_DEFAULT_CAN_ID);
    (void)knx_can_router_init(&board_gimbal_can_router, &board_jc_can);
    (void)knx_can_router_add_range(&board_gimbal_can_router,
                                   (uint16_t)(JC_RX_ID_BASE + JC_MOTOR_ID_MIN),
                                   (uint16_t)(JC_RX_ID_BASE + JC_MOTOR_ID_MAX),
                                   knx_board_jc_can_route,
                                   NULL);
    (void)knx_can_router_add_id(&board_gimbal_can_router,
                                DM_IMU_L1_ACTIVE_REPORT_CAN_ID,
                                knx_board_dm_imu_l1_can_route,
                                NULL);
    (void)knx_can_router_add_id(&board_gimbal_can_router,
                                DM_IMU_L1_DEFAULT_CAN_ID,
                                knx_board_dm_imu_l1_can_route,
                                NULL);
    (void)knx_can_router_start(&board_gimbal_can_router);
    (void)JC_InitExternalRx();
    (void)DM_IMU_L1_InitExternalRx();
#endif

    /* Init sensors */
    BMI088_AttachHeater(&bmi088_heater_pwm);
    BMI088_Init(&bmi088_accel_spi, &bmi088_gyro_spi);

    /* Init motor drivers */
    DRV8701E_AttachPorts(&drv_left_port, &drv_right_port);
    DRV8701E_AttachCurrentSampleTrigger(&drv_current_sample_trigger);
    DRV8701E_Init();

    /* Init encoders — bind platform ports first */
    (void)Encoder_AttachPorts(&enc_left_port, &enc_right_port);
    (void)Encoder_Init();

    /* New MCU-based module: I2C1 on PB8/PB9, shared with the OLED. */
    (void)ir_line_sensor_init(&board_ir_line_sensor,
                              &board_ir_line_sensor_config);
    knx_grayscale_attach_sensor(&board_ir_line_sensor);

    /* Init LEDs */
    knx_led_attach_ports(led_ports, sizeof(led_ports) / sizeof(led_ports[0]));
    knx_led_init();

    /* Init keys */
    knx_key_attach_ports(key_ports, sizeof(key_ports) / sizeof(key_ports[0]));
    knx_key_init();

    /* Init beep */
    knx_beep_attach_port(beep_gpio);
    knx_beep_init();

    /* LSI约32kHz，/32后约1kHz，500计数约500ms。PR/RLR受写保护，
     * 必须先写0x5555解锁；旧代码遗漏该步骤，实际超时并不等于注释值。 */
    IWDG1->KR  = 0xCCCCU;   /* 启动IWDG */
    IWDG1->KR  = 0x5555U;   /* 允许修改PR/RLR */
    IWDG1->PR  = 3U;        /* 预分频/32 */
    IWDG1->RLR = 500U;      /* 约500ms超时 */
    while ((IWDG1->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0U) {}
    IWDG1->KR  = 0xAAAAU;   /* 首次喂狗 */

    return KNX_OK;
}

void knx_board_watchdog_refresh(void)
{
    IWDG1->KR = 0xAAAAU;  /* Reload IWDG counter */
}

void knx_board_post_init(void)
{
}

const knx_gpio_t *knx_board_get_debug_led(void)
{
    return &board_debug_led;
}

Octolinker_Instance_t *knx_board_get_octolinker(void)
{
    return &board_octolinker;
}

ir_line_sensor_t *knx_board_get_ir_line_sensor(void)
{
    return &board_ir_line_sensor;
}

const knx_encoder_port_t *knx_board_get_encoder_left(void)
{
    return &enc_left_port;
}

const knx_encoder_port_t *knx_board_get_encoder_right(void)
{
    return &enc_right_port;
}

knx_can_t *knx_board_get_can_bus(uint8_t index)
{
    if (index == 0U) return &board_can1;
    if (index == 1U) return &board_can2;
    return NULL;
}

knx_can_t *knx_board_get_jc_can(void)
{
    return &board_can2;
}

knx_can_t *knx_board_get_dm_imu_l1_can(void)
{
    return &board_can2;
}

knx_host_comm_t *knx_board_get_host_comm(void)
{
    return &board_host_comm;
}

uint16_t knx_board_host_comm_read(uint8_t *out, uint16_t max_len)
{
    if (out == NULL || max_len == 0U) {
        return 0U;
    }

    return knx_ringbuf_pop(&board_host_rx_rb, out, max_len);
}

uint32_t knx_board_host_comm_rx_irq_count(void)
{
    return knx_ringbuf_push_count(&board_host_rx_rb);
}

uint32_t knx_board_host_comm_rx_overflow_count(void)
{
    return knx_ringbuf_overflow_count(&board_host_rx_rb);
}

uint32_t knx_board_host_comm_rx_restart_errors(void)
{
    return board_host_rx_restart_errors;
}

uint32_t knx_board_host_comm_uart_error_count(void)
{
    return board_host_uart_error_count;
}

uint32_t knx_board_host_comm_uart_error_code(void)
{
    return board_host_uart_error_code;
}

uint32_t knx_board_host_comm_uart_rx_state(void)
{
    return board_host_uart_rx_state;
}

uint32_t knx_board_host_comm_uart_start_status(void)
{
    return board_host_uart_start_status;
}

uint8_t knx_board_host_comm_rx_last_byte(void)
{
    return board_host_rx_last_byte;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        knx_board_host_rx_push(board_host_rx_byte);
        knx_board_host_comm_start_rx();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        board_host_uart_error_count++;
        board_host_uart_error_code = huart->ErrorCode;
        board_host_uart_rx_state = (uint32_t)huart->RxState;
        knx_board_host_comm_start_rx();
    }
}


