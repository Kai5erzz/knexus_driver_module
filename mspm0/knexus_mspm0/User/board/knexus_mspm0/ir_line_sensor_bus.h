#ifndef IR_LINE_SENSOR_BUS_H
#define IR_LINE_SENSOR_BUS_H

#include <stddef.h>
#include <stdint.h>
#include "knx_types.h"

void ir_line_sensor_board_i2c_init(void);
knx_status_t ir_line_sensor_board_i2c_read(
    void *context, uint8_t address_7bit, uint8_t reg,
    uint8_t *data, size_t length, uint32_t timeout_ms);

#endif
