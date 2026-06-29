#include "knx_can.h"
#include "knx_health.h"
#include "knx_time.h"
#include "stm32h7xx_hal.h"
#include <stddef.h>

#define KNX_CAN_MAX_BUSES 2U

typedef struct {
    knx_can_t *can;
    knx_can_rx_callback_t callback;
    void *user;
    uint8_t started;
} knx_can_slot_t;

static knx_can_slot_t s_slots[KNX_CAN_MAX_BUSES];
static volatile uint32_t s_can_error_count = 0U;

static knx_status_t knx_can_status(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) {
        return KNX_OK;
    }
    if (status == HAL_TIMEOUT) {
        return KNX_TIMEOUT;
    }
    if (status == HAL_BUSY) {
        return KNX_BUSY;
    }
    return KNX_ERROR;
}

static knx_can_slot_t *knx_can_find_slot_by_handle(FDCAN_HandleTypeDef *hfdcan)
{
    for (uint32_t i = 0U; i < KNX_CAN_MAX_BUSES; ++i) {
        if (s_slots[i].can != NULL && s_slots[i].can->handle == hfdcan) {
            return &s_slots[i];
        }
    }
    return NULL;
}

static knx_can_slot_t *knx_can_get_slot(knx_can_t *can)
{
    if (can == NULL || can->handle == NULL) {
        return NULL;
    }

    knx_can_slot_t *empty = NULL;
    for (uint32_t i = 0U; i < KNX_CAN_MAX_BUSES; ++i) {
        if (s_slots[i].can == can || s_slots[i].can == NULL) {
            if (s_slots[i].can == NULL && empty == NULL) {
                empty = &s_slots[i];
            }
            if (s_slots[i].can == can) {
                return &s_slots[i];
            }
        }
    }

    if (empty != NULL) {
        empty->can = can;
    }
    return empty;
}

static uint8_t knx_can_dlc_to_len(uint32_t dlc)
{
    switch (dlc) {
    case FDCAN_DLC_BYTES_0: return 0U;
    case FDCAN_DLC_BYTES_1: return 1U;
    case FDCAN_DLC_BYTES_2: return 2U;
    case FDCAN_DLC_BYTES_3: return 3U;
    case FDCAN_DLC_BYTES_4: return 4U;
    case FDCAN_DLC_BYTES_5: return 5U;
    case FDCAN_DLC_BYTES_6: return 6U;
    case FDCAN_DLC_BYTES_7: return 7U;
    case FDCAN_DLC_BYTES_8: return 8U;
    default: return 8U;
    }
}

static uint32_t knx_can_len_to_dlc(uint8_t len)
{
    switch (len) {
    case 0U: return FDCAN_DLC_BYTES_0;
    case 1U: return FDCAN_DLC_BYTES_1;
    case 2U: return FDCAN_DLC_BYTES_2;
    case 3U: return FDCAN_DLC_BYTES_3;
    case 4U: return FDCAN_DLC_BYTES_4;
    case 5U: return FDCAN_DLC_BYTES_5;
    case 6U: return FDCAN_DLC_BYTES_6;
    case 7U: return FDCAN_DLC_BYTES_7;
    default: return FDCAN_DLC_BYTES_8;
    }
}

knx_status_t knx_can_set_rx_callback(knx_can_t *can,
                                     knx_can_rx_callback_t callback,
                                     void *user)
{
    knx_can_slot_t *slot = knx_can_get_slot(can);
    if (slot == NULL) {
        return KNX_INVALID_ARG;
    }

    slot->callback = callback;
    slot->user = user;
    return KNX_OK;
}

knx_status_t knx_can_start(knx_can_t *can)
{
    knx_can_slot_t *slot = knx_can_get_slot(can);
    if (slot == NULL) {
        return KNX_INVALID_ARG;
    }

    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)can->handle;

    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000U;
    filter.FilterID2 = 0x000U;

    HAL_StatusTypeDef status = HAL_FDCAN_ConfigFilter(hfdcan, &filter);
    if (status != HAL_OK) {
        return knx_can_status(status);
    }

    status = HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                          FDCAN_ACCEPT_IN_RX_FIFO0,
                                          FDCAN_REJECT,
                                          FDCAN_REJECT_REMOTE,
                                          FDCAN_REJECT_REMOTE);
    if (status != HAL_OK) {
        return knx_can_status(status);
    }

    status = HAL_FDCAN_Start(hfdcan);
    if (status != HAL_OK) {
        return knx_can_status(status);
    }

    status = HAL_FDCAN_ActivateNotification(hfdcan,
                                            FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                            FDCAN_IT_BUS_OFF |
                                            FDCAN_IT_ERROR_PASSIVE |
                                            FDCAN_IT_ERROR_WARNING |
                                            FDCAN_IT_RX_FIFO0_FULL |
                                            FDCAN_IT_RX_FIFO0_MESSAGE_LOST,
                                            0U);
    if (status == HAL_OK) {
        slot->started = 1U;
    }

    return knx_can_status(status);
}

knx_status_t knx_can_transmit_std(knx_can_t *can,
                                  uint16_t std_id,
                                  const uint8_t *data,
                                  uint8_t len,
                                  uint32_t timeout_ms)
{
    if (can == NULL || can->handle == NULL || data == NULL || len > 8U || std_id > 0x7FFU) {
        return KNX_INVALID_ARG;
    }

    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)can->handle;

    /* Wait for TX FIFO to have free space, with timeout */
    uint32_t start = knx_millis();
    while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U) {
        if (knx_millis() - start >= timeout_ms) {
            return KNX_TIMEOUT;
        }
    }

    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = std_id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = knx_can_len_to_dlc(len);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    return knx_can_status(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, (uint8_t *)data));
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    /* Check and clear FIFO message lost condition */
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U) {
        __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST);
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    knx_can_slot_t *slot = knx_can_find_slot_by_handle(hfdcan);
    if (slot == NULL || slot->callback == NULL) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        FDCAN_RxHeaderTypeDef header = {0};
        uint8_t data[8] = {0};

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
            break;
        }

        if (header.IdType == FDCAN_STANDARD_ID && header.RxFrameType == FDCAN_DATA_FRAME) {
            slot->callback(header.Identifier,
                           data,
                           knx_can_dlc_to_len(header.DataLength),
                           slot->user);
        }
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t error = HAL_FDCAN_GetError(hfdcan);
    s_can_error_count++;

    /* Detect bus-off via status flag (no HAL_FDCAN_ERROR_BUS_OFF exists) */
    uint32_t bus_off = __HAL_FDCAN_GET_FLAG(hfdcan, FDCAN_FLAG_BUS_OFF);

    (void)knx_health_report(KNX_HEALTH_SOURCE_JC,
                            bus_off ? KNX_HEALTH_STATE_FAULT :
                                      KNX_HEALTH_STATE_WARN,
                            KNX_ERROR,
                            error,
                            s_can_error_count);

    /* Bus-off recovery: stop and restart FDCAN */
    if (bus_off != 0U) {
        (void)HAL_FDCAN_Stop(hfdcan);
        (void)HAL_FDCAN_Start(hfdcan);
    }
}
