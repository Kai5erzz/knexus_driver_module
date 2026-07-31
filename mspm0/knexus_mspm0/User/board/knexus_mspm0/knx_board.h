#ifndef KNX_BOARD_H
#define KNX_BOARD_H

#include "knx_gpio.h"
#include "knx_types.h"
#include "knx_encoder.h"
#include "octolinker.h"
#include "drv8701e.h"
#include "knx_spi.h"
#include "knx_can.h"
#include "ir_line_sensor.h"

knx_status_t knx_board_init(void);
void         knx_board_post_init(void);

const knx_gpio_t        *knx_board_get_debug_led(void);
const knx_encoder_port_t *knx_board_get_encoder_left(void);
const knx_encoder_port_t *knx_board_get_encoder_right(void);
Octolinker_Instance_t    *knx_board_get_octolinker(void);
const drv8701e_port_t    *knx_board_get_drv_left_port(void);
const drv8701e_port_t    *knx_board_get_drv_right_port(void);

/* ── New peripheral bindings ── */
const knx_spi_t            *knx_board_get_bmi088_accel_spi(void);
const knx_spi_t            *knx_board_get_bmi088_gyro_spi(void);
knx_can_t                  *knx_board_get_can(void);
knx_can_t                  *knx_board_get_can_bus(uint8_t index);
ir_line_sensor_t           *knx_board_get_ir_line_sensor(void);

/* ── CAN getters aligned with STM32 board API (parity stubs) ──
 * MSPM0 has a single MCAN instance; these aliases map to knx_board_get_can().
 * Declared for API parity so shared code can call them without #ifdef. */
knx_can_t                  *knx_board_get_jc_can(void);
knx_can_t                  *knx_board_get_dm_imu_l1_can(void);

#endif /* KNX_BOARD_H */
