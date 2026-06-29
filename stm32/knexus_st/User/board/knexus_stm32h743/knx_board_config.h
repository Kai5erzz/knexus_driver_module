#ifndef KNX_BOARD_CONFIG_H
#define KNX_BOARD_CONFIG_H

/* ---- Target MCU ---- */
#define KNX_TARGET_STM32H743IIT6       1

/* ---- Board identifier ---- */
#define KNX_BOARD_KNEXUS_STM32H743     1

/* ---- RTOS ---- */
#define KNX_RTOS_FREERTOS              1

/* ---- Clock ---- */
#define KNX_SYSCLK_HZ                 480000000UL
#define KNX_AHBCLK_HZ                 240000000UL
#define KNX_APB1CLK_HZ                120000000UL
#define KNX_APB2CLK_HZ                120000000UL

/* ---- Motor PWM (TIM2) ---- */
#define KNX_MOTOR_TIM                 TIM2
#define KNX_MOTOR_TIM_CH_LEFT         TIM_CHANNEL_1   /* PA15 */
#define KNX_MOTOR_TIM_CH_RIGHT        TIM_CHANNEL_3   /* PA2  */

/* ---- Encoder ---- */
#define KNX_ENC_LEFT_TIM              TIM1             /* PA8/PA9, 4x quadrature */
#define KNX_ENC_RIGHT_LPTIM           LPTIM1           /* PG12/PG11, 2x external */

/* ---- ADC ---- */
#define KNX_ADC_MOTOR_CURRENT         ADC1             /* PC0, PA4 */
#define KNX_ADC_TRACK_SENSOR          ADC2             /* PC5 */

/* ---- Debug UART ---- */
#define KNX_DEBUG_UART                USART1

#endif /* KNX_BOARD_CONFIG_H */
