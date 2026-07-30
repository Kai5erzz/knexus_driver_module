#include "knx_can.h"
#include "knx_time.h"
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <stddef.h>
#include <string.h>

static knx_can_rx_callback_t s_rx_callback;
static void *s_rx_user;
static knx_can_t *s_active_can;

static uint8_t timeout_elapsed(uint32_t start_ms, uint32_t timeout_ms)
{
    if (timeout_ms == 0U) {
        return 0U;
    }

    return ((knx_millis() - start_ms) >= timeout_ms) ? 1U : 0U;
}

static uint8_t dlc_to_len(uint32_t dlc)
{
    return (dlc <= 8U) ? (uint8_t)dlc : 8U;
}

static uint32_t std_id_from_rx(const DL_MCAN_RxBufElement *rx)
{
    return (rx->id & 0x1FFC0000UL) >> 18U;
}

knx_status_t knx_can_start(knx_can_t *can)
{
    if (can == NULL || can->handle == NULL) {
        return KNX_NOT_READY;
    }

    MCAN_Regs *regs = (MCAN_Regs *)can->handle;
    if (DL_MCAN_getOpMode(regs) != DL_MCAN_OPERATION_MODE_NORMAL) {
        return KNX_NOT_READY;
    }

    s_active_can = can;
    return KNX_OK;
}

knx_status_t knx_can_transmit_std(knx_can_t *can,
                                  uint16_t std_id,
                                  const uint8_t *data,
                                  uint8_t len,
                                  uint32_t timeout_ms)
{
    if (can == NULL || can->handle == NULL) {
        return KNX_NOT_READY;
    }
    if (std_id > 0x7FFU || len > 8U || (data == NULL && len > 0U)) {
        return KNX_INVALID_ARG;
    }

    MCAN_Regs *regs = (MCAN_Regs *)can->handle;
    if (DL_MCAN_getOpMode(regs) != DL_MCAN_OPERATION_MODE_NORMAL) {
        return KNX_NOT_READY;
    }

    uint32_t start_ms = knx_millis();
    while ((DL_MCAN_getTxBufReqPend(regs) & 0x1U) != 0U) {
        if (timeout_ms == 0U) {
            return KNX_BUSY;
        }
        if (timeout_elapsed(start_ms, timeout_ms) != 0U) {
            return KNX_TIMEOUT;
        }
    }

    DL_MCAN_TxBufElement tx;
    memset(&tx, 0, sizeof(tx));
    tx.id = ((uint32_t)std_id) << 18U;
    tx.rtr = 0U;
    tx.xtd = 0U;
    tx.esi = 0U;
    tx.dlc = len;
    tx.brs = 0U;
    tx.fdf = 0U;
    tx.efc = 0U;
    tx.mm = 0U;
    if (len > 0U) {
        memcpy(tx.data, data, len);
    }

    DL_MCAN_writeMsgRam(regs, DL_MCAN_MEM_TYPE_BUF, 0U, &tx);
    if (DL_MCAN_TXBufAddReq(regs, 0U) != 0) {
        return KNX_ERROR;
    }

    /* Non-blocking mode reports success once the frame is queued. */
    if (timeout_ms == 0U) {
        return KNX_OK;
    }

    while ((DL_MCAN_getTxBufReqPend(regs) & 0x1U) != 0U) {
        if (timeout_elapsed(start_ms, timeout_ms) != 0U) {
            return KNX_TIMEOUT;
        }
    }

    return KNX_OK;
}

knx_status_t knx_can_set_rx_callback(knx_can_t *can,
                                     knx_can_rx_callback_t callback,
                                     void *user)
{
    if (can == NULL) {
        return KNX_INVALID_ARG;
    }

    s_rx_callback = callback;
    s_rx_user = user;
    return KNX_OK;
}

void knx_can_mspm0_dispatch_rx(uint32_t std_id, const uint8_t *data, uint8_t len)
{
    if (s_rx_callback != NULL) {
        s_rx_callback(std_id, data, len, s_rx_user);
    }
}

void knx_can_mspm0_poll(void)
{
    if (s_active_can == NULL || s_active_can->handle == NULL) {
        return;
    }

    MCAN_Regs *regs = (MCAN_Regs *)s_active_can->handle;
    if (DL_MCAN_getOpMode(regs) != DL_MCAN_OPERATION_MODE_NORMAL) {
        return;
    }

    DL_MCAN_RxFIFOStatus rx_status;
    DL_MCAN_RxBufElement rx;

    do {
        memset(&rx_status, 0, sizeof(rx_status));
        rx_status.num = DL_MCAN_RX_FIFO_NUM_0;
        DL_MCAN_getRxFIFOStatus(regs, &rx_status);
        if (rx_status.fillLvl == 0U) {
            break;
        }

        memset(&rx, 0, sizeof(rx));
        DL_MCAN_readMsgRam(regs,
                           DL_MCAN_MEM_TYPE_FIFO,
                           0U,
                           DL_MCAN_RX_FIFO_NUM_0,
                           &rx);
        (void)DL_MCAN_writeRxFIFOAck(regs,
                                     DL_MCAN_RX_FIFO_NUM_0,
                                     rx_status.getIdx);

        if (rx.xtd == 0U && rx.rtr == 0U) {
            knx_can_mspm0_dispatch_rx(std_id_from_rx(&rx),
                                      rx.data,
                                      dlc_to_len(rx.dlc));
        }
    } while (rx_status.fillLvl > 1U);
}
