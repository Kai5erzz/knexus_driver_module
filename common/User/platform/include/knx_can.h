#ifndef KNX_CAN_H
#define KNX_CAN_H

#include "knx_types.h"
#include <stdint.h>

/**
 * @brief  CAN peripheral handle.
 *
 * @handle Pointer to the underlying hardware-specific context
 *         (e.g. FDCAN_HandleTypeDef* on STM32, MCAN_Regs* on MSPM0).
 *         Populated by the board layer before knx_can_start() is called.
 */
typedef struct {
    void *handle;
} knx_can_t;

/**
 * @brief  Callback invoked from the CAN receive path for each accepted frame.
 *
 * @param std_id  11-bit standard identifier of the received frame.
 * @param data    Pointer to the payload buffer (valid only for the duration
 *                of the callback; copy if persistence is needed).
 * @param len     Number of payload bytes (0–8 for classic CAN).
 * @param user    Opaque pointer registered via knx_can_set_rx_callback().
 */
typedef void (*knx_can_rx_callback_t)(uint32_t std_id,
                                      const uint8_t *data,
                                      uint8_t len,
                                      void *user);

/**
 * @brief   Configure the CAN peripheral and start it on the bus.
 *
 * Configures bit-timing, acceptance filters (if not already set by the
 * hardware init / SysConfig layer), and enables the peripheral for both
 * transmission and reception. After this call returns KNX_OK the CAN
 * instance is live on the bus.
 *
 * @param can  Pointer to a knx_can_t whose @ref handle has been populated
 *             by the board layer. Must not be NULL.
 *
 * @retval KNX_OK          Peripheral configured and started successfully.
 * @retval KNX_INVALID_ARG @p can is NULL or @p can->handle is NULL.
 * @retval KNX_ERROR       Hardware initialisation failed.
 */
knx_status_t knx_can_start(knx_can_t *can);

/**
 * @brief   Transmit a classic (11-bit standard ID) CAN frame.
 *
 * Copies @p data into the CAN TX FIFO / mailbox and blocks until either the
 * frame is accepted by the bus (TX mailbox drained) or @p timeout_ms elapses.
 *
 * @param can         Pointer to a started knx_can_t instance.
 * @param std_id      11-bit standard identifier (0–0x7FF).
 * @param data        Payload buffer; may be NULL only when @p len is 0.
 * @param len         Payload length in bytes (0–8 for classic CAN).
 * @param timeout_ms  Maximum time to wait for TX space. 0 means non-blocking
 *                    (returns immediately with KNX_TIMEOUT if no mailbox is
 *                    free). UINT32_MAX or a large value effectively blocks
 *                    indefinitely.
 *
 * @retval KNX_OK             Frame transmitted successfully.
 * @retval KNX_INVALID_ARG    @p can is NULL, @p std_id exceeds 0x7FF, or
 *                            @p len > 8.
 * @retval KNX_TIMEOUT        No TX mailbox/free slot within @p timeout_ms.
 * @retval KNX_NOT_READY      The CAN peripheral has not been started.
 * @retval KNX_ERROR          Lower-level hardware error (bus-off, error
 *                            passive, etc.).
 */
knx_status_t knx_can_transmit_std(knx_can_t *can,
                                  uint16_t std_id,
                                  const uint8_t *data,
                                  uint8_t len,
                                  uint32_t timeout_ms);

/**
 * @brief   Register a receive callback for this CAN instance.
 *
 * Only **one** callback per knx_can_t instance is supported; calling this
 * function again replaces the previous callback. Each knx_can_t instance
 * maintains its own independent callback and @p user pointer, so multiple
 * CAN peripherals can coexist with separate handlers.
 *
 * Passing @p callback as NULL unregisters any existing callback (received
 * frames will then be silently discarded).
 *
 * @param can       Pointer to a knx_can_t instance. Must not be NULL.
 * @param callback  Function called for each accepted RX frame, or NULL to
 *                  unregister.
 * @param user      Opaque context passed back as the @p user parameter of
 *                  @ref knx_can_rx_callback_t. May be NULL.
 *
 * @retval KNX_OK          Callback registered (or unregistered) successfully.
 * @retval KNX_INVALID_ARG @p can is NULL.
 */
knx_status_t knx_can_set_rx_callback(knx_can_t *can,
                                     knx_can_rx_callback_t callback,
                                     void *user);

#endif /* KNX_CAN_H */
