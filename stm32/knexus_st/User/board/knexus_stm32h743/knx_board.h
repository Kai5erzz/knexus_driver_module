#ifndef KNX_BOARD_H
#define KNX_BOARD_H

#include "knx_types.h"
#include "knx_gpio.h"
#include "knx_spi.h"
#include "knx_encoder.h"
#include "knx_can.h"
#include "knx_host_comm.h"
#include "octolinker.h"

/* Board-level initialization.
 * Called once after all CubeMX MX_xxx_Init() are complete.
 * Does NOT re-initialize CubeMX peripherals. */
knx_status_t knx_board_init(void);

/* Post-init hook — called after all module inits are done.
 * Use for late-stage board setup (e.g. enable outputs, start timers). */
void knx_board_post_init(void);

const knx_gpio_t *knx_board_get_debug_led(void);

/* Get the OctoLink instance for debug output */
Octolinker_Instance_t *knx_board_get_octolinker(void);

/* Get encoder port descriptors (left=TIM1, right=LPTIM1) */
const knx_encoder_port_t *knx_board_get_encoder_left(void);
const knx_encoder_port_t *knx_board_get_encoder_right(void);

/* JC motors are mounted on FDCAN2. */
knx_can_t *knx_board_get_jc_can(void);

/* DM-IMU-L1 shares FDCAN2 with the JC motors. */
knx_can_t *knx_board_get_dm_imu_l1_can(void);

/* Upper/lower host communication link. USART2: PD5 TX, PD6 RX. */
knx_host_comm_t *knx_board_get_host_comm(void);
uint16_t knx_board_host_comm_read(uint8_t *out, uint16_t max_len);
uint32_t knx_board_host_comm_rx_irq_count(void);
uint32_t knx_board_host_comm_rx_overflow_count(void);
uint32_t knx_board_host_comm_rx_restart_errors(void);
uint32_t knx_board_host_comm_uart_error_count(void);
uint32_t knx_board_host_comm_uart_error_code(void);
uint32_t knx_board_host_comm_uart_rx_state(void);
uint32_t knx_board_host_comm_uart_start_status(void);
uint8_t knx_board_host_comm_rx_last_byte(void);

#endif /* KNX_BOARD_H */
