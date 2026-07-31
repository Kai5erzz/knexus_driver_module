#include "ir_line_sensor.h"

#include <string.h>

#define IR_FRAME_HEAD 0xAAU
#define IR_FRAME_TAIL 0xFFU

static knx_status_t validate_config(const ir_line_sensor_config_t *config)
{
    if (config == NULL) return KNX_INVALID_ARG;
    switch (config->link) {
    case IR_LINE_SENSOR_LINK_I2C:
        return config->io.i2c.read_register != NULL ? KNX_OK : KNX_INVALID_ARG;
    case IR_LINE_SENSOR_LINK_UART:
        return config->io.uart.transmit != NULL && config->io.uart.receive != NULL
                   ? KNX_OK : KNX_INVALID_ARG;
    case IR_LINE_SENSOR_LINK_GPIO:
        return config->io.gpio.read != NULL ? KNX_OK : KNX_INVALID_ARG;
    default:
        return KNX_INVALID_ARG;
    }
}

knx_status_t ir_line_sensor_init(ir_line_sensor_t *sensor,
                                 const ir_line_sensor_config_t *config)
{
    if (sensor == NULL) return KNX_INVALID_ARG;
    knx_status_t status = validate_config(config);
    if (status != KNX_OK) return status;
    memset(sensor, 0, sizeof(*sensor));
    sensor->config = *config;
    sensor->sample.mode = IR_LINE_SENSOR_DATA_DIGITAL;
    sensor->initialized = 1U;
    return KNX_OK;
}

knx_status_t ir_line_sensor_set_uart_mode(ir_line_sensor_t *sensor,
                                          ir_line_sensor_data_mode_t mode)
{
    static const uint8_t digital_command[2] = {'D', 'S'};
    static const uint8_t analog_command[2] = {'A', 'D'};
    if (sensor == NULL || sensor->initialized == 0U) return KNX_NOT_READY;
    if (sensor->config.link != IR_LINE_SENSOR_LINK_UART ||
        (mode != IR_LINE_SENSOR_DATA_DIGITAL &&
         mode != IR_LINE_SENSOR_DATA_ANALOG)) return KNX_INVALID_ARG;
    const uint8_t *command = mode == IR_LINE_SENSOR_DATA_ANALOG
                                 ? analog_command : digital_command;
    knx_status_t status = sensor->config.io.uart.transmit(
        sensor->config.context, command, sizeof(digital_command),
        sensor->config.timeout_ms);
    if (status == KNX_OK) {
        sensor->sample.mode = mode;
        sensor->sample.valid = 0U;
    }
    return status;
}

knx_status_t ir_line_sensor_parse_uart_frame(
    const uint8_t frame[IR_LINE_SENSOR_UART_FRAME_SIZE],
    ir_line_sensor_data_mode_t mode,
    ir_line_sensor_sample_t *sample)
{
    if (frame == NULL || sample == NULL) return KNX_INVALID_ARG;
    if ((mode != IR_LINE_SENSOR_DATA_DIGITAL &&
         mode != IR_LINE_SENSOR_DATA_ANALOG) ||
        frame[0] != IR_FRAME_HEAD || frame[1] != IR_FRAME_HEAD ||
        frame[18] != IR_FRAME_TAIL || frame[19] != IR_FRAME_TAIL) {
        return KNX_ERROR;
    }
    ir_line_sensor_sample_t parsed;
    memset(&parsed, 0, sizeof(parsed));
    parsed.mode = mode;
    for (uint8_t i = 0U; i < IR_LINE_SENSOR_CHANNEL_COUNT; ++i) {
        uint16_t value = (uint16_t)(((uint16_t)frame[2U + i * 2U] << 8U) |
                                    frame[3U + i * 2U]);
        if ((mode == IR_LINE_SENSOR_DATA_DIGITAL && value > 1U) ||
            (mode == IR_LINE_SENSOR_DATA_ANALOG && value > IR_LINE_SENSOR_ADC_MAX)) {
            return KNX_ERROR;
        }
        parsed.analog[i] = value;
        if (mode == IR_LINE_SENSOR_DATA_DIGITAL && value != 0U) {
            parsed.digital_bits |= (uint8_t)(1U << i);
        }
    }
    parsed.valid = 1U;
    *sample = parsed;
    return KNX_OK;
}

static knx_status_t read_i2c(ir_line_sensor_t *sensor)
{
    uint8_t state = 0U;
    knx_status_t status = sensor->config.io.i2c.read_register(
        sensor->config.context, IR_LINE_SENSOR_I2C_ADDRESS_7BIT,
        IR_LINE_SENSOR_I2C_STATE_REGISTER, &state, 1U,
        sensor->config.timeout_ms);
    if (status == KNX_OK) {
        memset(sensor->sample.analog, 0, sizeof(sensor->sample.analog));
        sensor->sample.digital_bits = state;
        sensor->sample.mode = IR_LINE_SENSOR_DATA_DIGITAL;
        sensor->sample.valid = 1U;
    }
    return status;
}

static knx_status_t read_gpio(ir_line_sensor_t *sensor)
{
    uint8_t state = 0U;
    for (uint8_t i = 0U; i < IR_LINE_SENSOR_CHANNEL_COUNT; ++i) {
        uint8_t level = 0U;
        knx_status_t status = sensor->config.io.gpio.read(
            sensor->config.context, i, &level);
        if (status != KNX_OK) return status;
        if (level != 0U) state |= (uint8_t)(1U << i);
    }
    memset(sensor->sample.analog, 0, sizeof(sensor->sample.analog));
    sensor->sample.digital_bits = state;
    sensor->sample.mode = IR_LINE_SENSOR_DATA_DIGITAL;
    sensor->sample.valid = 1U;
    return KNX_OK;
}

static knx_status_t read_uart(ir_line_sensor_t *sensor)
{
    uint8_t frame[IR_LINE_SENSOR_UART_FRAME_SIZE];
    knx_status_t status = sensor->config.io.uart.receive(
        sensor->config.context, frame, sizeof(frame), sensor->config.timeout_ms);
    return status == KNX_OK
               ? ir_line_sensor_parse_uart_frame(frame, sensor->sample.mode,
                                                 &sensor->sample)
               : status;
}

knx_status_t ir_line_sensor_read(ir_line_sensor_t *sensor)
{
    if (sensor == NULL || sensor->initialized == 0U) return KNX_NOT_READY;
    switch (sensor->config.link) {
    case IR_LINE_SENSOR_LINK_I2C: return read_i2c(sensor);
    case IR_LINE_SENSOR_LINK_UART: return read_uart(sensor);
    case IR_LINE_SENSOR_LINK_GPIO: return read_gpio(sensor);
    default: return KNX_INVALID_ARG;
    }
}

const ir_line_sensor_sample_t *ir_line_sensor_get_sample(
    const ir_line_sensor_t *sensor)
{
    return sensor != NULL && sensor->initialized != 0U ? &sensor->sample : NULL;
}

uint8_t ir_line_sensor_channel_detected(const ir_line_sensor_t *sensor,
                                        uint8_t channel)
{
    if (sensor == NULL || sensor->initialized == 0U ||
        sensor->sample.valid == 0U || channel >= IR_LINE_SENSOR_CHANNEL_COUNT) {
        return 0U;
    }
    return (uint8_t)((sensor->sample.digital_bits >> channel) & 1U);
}
