/**
 * @file ir_line_sensor.h
 * @brief Xiao-R 8-channel MCU infrared line-sensor driver.
 *
 * The replacement board contains its own MCU.  It must be read through I2C,
 * UART or eight digital GPIOs; it is not the old MUX + ADC board.
 */
#ifndef IR_LINE_SENSOR_H
#define IR_LINE_SENSOR_H

#include <stddef.h>
#include <stdint.h>
#include "knx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IR_LINE_SENSOR_CHANNEL_COUNT       8U
#define IR_LINE_SENSOR_I2C_ADDRESS_7BIT    0x30U
#define IR_LINE_SENSOR_I2C_STATE_REGISTER  0x50U
#define IR_LINE_SENSOR_UART_BAUDRATE       115200U
#define IR_LINE_SENSOR_UART_FRAME_SIZE     20U
#define IR_LINE_SENSOR_ADC_MAX             4096U
#define IR_LINE_SENSOR_UPDATE_PERIOD_MS    20U

typedef enum {
    IR_LINE_SENSOR_LINK_I2C = 0,
    IR_LINE_SENSOR_LINK_UART,
    IR_LINE_SENSOR_LINK_GPIO
} ir_line_sensor_link_t;

typedef enum {
    IR_LINE_SENSOR_DATA_DIGITAL = 0,
    IR_LINE_SENSOR_DATA_ANALOG
} ir_line_sensor_data_mode_t;

typedef knx_status_t (*ir_line_sensor_i2c_read_fn)(
    void *context, uint8_t address_7bit, uint8_t reg,
    uint8_t *data, size_t length, uint32_t timeout_ms);
typedef knx_status_t (*ir_line_sensor_uart_tx_fn)(
    void *context, const uint8_t *data, size_t length, uint32_t timeout_ms);
typedef knx_status_t (*ir_line_sensor_uart_rx_fn)(
    void *context, uint8_t *data, size_t length, uint32_t timeout_ms);
typedef knx_status_t (*ir_line_sensor_gpio_read_fn)(
    void *context, uint8_t channel, uint8_t *level);

typedef struct {
    ir_line_sensor_link_t link;
    void *context;
    uint32_t timeout_ms;
    union {
        struct { ir_line_sensor_i2c_read_fn read_register; } i2c;
        struct {
            ir_line_sensor_uart_tx_fn transmit;
            ir_line_sensor_uart_rx_fn receive;
        } uart;
        struct { ir_line_sensor_gpio_read_fn read; } gpio;
    } io;
} ir_line_sensor_config_t;

typedef struct {
    uint8_t digital_bits;                         /**< bit0..7 = channel1..8 */
    uint16_t analog[IR_LINE_SENSOR_CHANNEL_COUNT];
    ir_line_sensor_data_mode_t mode;
    uint8_t valid;
} ir_line_sensor_sample_t;

typedef struct {
    ir_line_sensor_config_t config;
    ir_line_sensor_sample_t sample;
    uint8_t initialized;
} ir_line_sensor_t;

knx_status_t ir_line_sensor_init(ir_line_sensor_t *sensor,
                                 const ir_line_sensor_config_t *config);
knx_status_t ir_line_sensor_set_uart_mode(ir_line_sensor_t *sensor,
                                          ir_line_sensor_data_mode_t mode);
knx_status_t ir_line_sensor_read(ir_line_sensor_t *sensor);
knx_status_t ir_line_sensor_parse_uart_frame(
    const uint8_t frame[IR_LINE_SENSOR_UART_FRAME_SIZE],
    ir_line_sensor_data_mode_t mode,
    ir_line_sensor_sample_t *sample);
const ir_line_sensor_sample_t *ir_line_sensor_get_sample(
    const ir_line_sensor_t *sensor);
uint8_t ir_line_sensor_channel_detected(const ir_line_sensor_t *sensor,
                                        uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif
