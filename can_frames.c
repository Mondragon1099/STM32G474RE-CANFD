/* ============================================================
 * can_frames.c
 * Builds and transmits all CAN FD frames.
 *
 * Each function:
 *   1. Receives sensor values as parameters from main.c
 *   2. Packs them into a byte array (big-endian)
 *   3. Transmits the frame via HAL_FDCAN_AddMessageToTxFifoQ
 *   4. Prints to UART if TX fails — does NOT call Error_Handler
 *      so the main loop always continues regardless of CAN status
 *
 * To add a new sensor frame:
 *   1. Add a new TX header and data buffer below
 *   2. Add a new function following the same pattern
 *   3. Declare it in can_frames.h
 *   4. Call it from main.c while loop
 * ============================================================ */

#include "can_frames.h"
#include "string.h"
#include "stdio.h"

/* ============================================================
 * TX headers — one per frame ID
 * Defined here, used only inside this file
 * ============================================================ */

/* ID 0x000 — AS5600 steering angle, 2 bytes */
static FDCAN_TxHeaderTypeDef TxHeader_0x000 = {
    .Identifier          = 0x000,
    .IdType              = FDCAN_STANDARD_ID,
    .TxFrameType         = FDCAN_DATA_FRAME,
    .DataLength          = FDCAN_DLC_BYTES_2,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch       = FDCAN_BRS_OFF,
    .FDFormat            = FDCAN_FD_CAN,
    .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
    .MessageMarker       = 0
};

/* ID 0x001 — dummy frame, 2 bytes */
static FDCAN_TxHeaderTypeDef TxHeader_0x001 = {
    .Identifier          = 0x001,
    .IdType              = FDCAN_STANDARD_ID,
    .TxFrameType         = FDCAN_DATA_FRAME,
    .DataLength          = FDCAN_DLC_BYTES_2,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch       = FDCAN_BRS_OFF,
    .FDFormat            = FDCAN_FD_CAN,
    .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
    .MessageMarker       = 0
};

/* ============================================================
 * TX data buffers — one per frame ID
 * ============================================================ */
static uint8_t TxData_0x000[2];
static uint8_t TxData_0x001[2];

/* ============================================================
 * Frame functions
 * ============================================================ */

/* ----------------------------------------------------------
 * can_tx_steering
 * ID 0x000 — AS5600 magnetic steering angle
 *
 * Payload (2 bytes, big-endian):
 *   [0-1]  deg * 10 as uint16
 *          e.g. 359.9 deg → 3599 → 0x0E 0x0F
 *
 * Decode on Pi:
 *   deg_int = (data[0] << 8) | data[1]
 *   degrees = deg_int / 10.0
 * ---------------------------------------------------------- */
void can_tx_steering(float deg)
{
    /* Multiply by 10 to keep 1 decimal place as integer
     * e.g. 270.5 deg → 2705 stored as uint16 */
    uint16_t deg_int = (uint16_t)(deg * 10.0f);

    /* Pack big-endian: high byte first, low byte second */
    TxData_0x000[0] = (deg_int >> 8) & 0xFF;  /* high byte */
    TxData_0x000[1] =  deg_int        & 0xFF;  /* low byte  */

    /* Transmit — print to UART on failure, do not halt */
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x000, TxData_0x000) != HAL_OK)
    {
        printf("TX 0x000 failed\r\n");
    }
}

/* ----------------------------------------------------------
 * can_tx_dummy
 * ID 0x001 — dummy frame (proof of concept)
 *
 * Payload (2 bytes):
 *   [0] 0xDE
 *   [1] 0xAD
 * ---------------------------------------------------------- */
void can_tx_dummy(void)
{
    TxData_0x001[0] = 0xDE;
    TxData_0x001[1] = 0xAD;

    /* Transmit — print to UART on failure, do not halt */
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x001, TxData_0x001) != HAL_OK)
    {
        printf("TX 0x001 failed\r\n");
    }
}
