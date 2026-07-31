#include "ir_line_sensor_bus.h"

#include "i2c.h"
#include "knx_custom_i2c_guard.h"

knx_status_t ir_line_sensor_board_i2c_read(
    void *context, uint8_t address_7bit, uint8_t reg,
    uint8_t *data, size_t length, uint32_t timeout_ms)
{
    (void)context;
    if (data == NULL || length == 0U || length > UINT16_MAX) {
        return KNX_INVALID_ARG;
    }
    if (!knx_custom_i2c_try_lock()) return KNX_BUSY;
    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read(
        &hi2c1, (uint16_t)(address_7bit << 1U), reg,
        I2C_MEMADD_SIZE_8BIT, data, (uint16_t)length, timeout_ms);
    knx_custom_i2c_unlock();
    if (hal_status == HAL_OK) return KNX_OK;
    if (hal_status == HAL_TIMEOUT) return KNX_TIMEOUT;
    if (hal_status == HAL_BUSY) return KNX_BUSY;
    return KNX_ERROR;
}
