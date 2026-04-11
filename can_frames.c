/* ============================================================
 * can_frames.c
 * Builds and transmits all CAN FD frames.
 *
 * Each function:
 * 1. Receives sensor values as parameters from main.c
 * 2. Packs them into a byte array (big-endian)
 * 3. Transmits the frame via HAL_FDCAN_AddMessageToTxFifoQ
 * 4. Prints to UART if TX fails — does NOT call Error_Handler
 * so the main loop always continues regardless of CAN status
 *
 * Encoding conventions:
 * - All multi-byte values are big-endian (high byte first)
 * - Floats with decimals are scaled to integers before packing:
 * degrees  → x10   as uint16  (e.g. 270.5 → 2705)
 * RPM      → x10   as uint16  (e.g. 123.4 → 1234)
 * voltage  → x1000 as uint16  (e.g. 2.500 → 2500)
 * - Vibration logic now uses a 1-byte Boolean flag
 * ============================================================ */

#include "can_frames.h"
#include "string.h"
#include "stdio.h"

/* ============================================================
 * TX headers — one per frame ID
 * ============================================================ */

static FDCAN_TxHeaderTypeDef TxHeader_0x000 = {
    .Identifier = 0x000, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_2, .FDFormat = FDCAN_FD_CAN
};

static FDCAN_TxHeaderTypeDef TxHeader_0x001 = {
    .Identifier = 0x001, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_2, .FDFormat = FDCAN_FD_CAN
};

static FDCAN_TxHeaderTypeDef TxHeader_0x002 = {
    .Identifier = 0x002, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_1, .FDFormat = FDCAN_FD_CAN
};

static FDCAN_TxHeaderTypeDef TxHeader_0x003 = {
    .Identifier = 0x003, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_1, .FDFormat = FDCAN_FD_CAN
};

static FDCAN_TxHeaderTypeDef TxHeader_0x004 = {
    .Identifier = 0x004, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_2, .FDFormat = FDCAN_FD_CAN
};

static FDCAN_TxHeaderTypeDef TxHeader_0x006 = {
    .Identifier = 0x006, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_2, .FDFormat = FDCAN_FD_CAN
};

static FDCAN_TxHeaderTypeDef TxHeader_0x007 = {
    .Identifier = 0x007, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_2, .FDFormat = FDCAN_FD_CAN
};

static FDCAN_TxHeaderTypeDef TxHeader_0x008 = {
    .Identifier = 0x008, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_2, .FDFormat = FDCAN_FD_CAN
};

static FDCAN_TxHeaderTypeDef TxHeader_0x009 = {
    .Identifier = 0x009, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_2, .FDFormat = FDCAN_FD_CAN
};

/* --- Shared local buffers --- */
static uint8_t TxData_2B[2];
static uint8_t TxData_1B[1];

void can_tx_steering1(float deg) {
    uint16_t val = (uint16_t)(deg * 10.0f);
    TxData_2B[0] = (val >> 8) & 0xFF;
    TxData_2B[1] = val & 0xFF;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x000, TxData_2B) != HAL_OK)
        printf("TX 0x000 failed\r\n");
}

void can_tx_dummy(void) {
    TxData_2B[0] = 0xDE;
    TxData_2B[1] = 0xAD;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x001, TxData_2B) != HAL_OK)
        printf("TX 0x001 failed\r\n");
}

void can_tx_adxl1(uint8_t unsafe) {
    TxData_1B[0] = unsafe;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x002, TxData_1B) != HAL_OK)
        printf("TX 0x002 failed\r\n");
}

void can_tx_adxl2(uint8_t unsafe) {
    TxData_1B[0] = unsafe;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x003, TxData_1B) != HAL_OK)
        printf("TX 0x003 failed\r\n");
}

void can_tx_rpm(float rpm) {
    uint16_t val = (uint16_t)(rpm * 10.0f);
    TxData_2B[0] = (val >> 8) & 0xFF;
    TxData_2B[1] = val & 0xFF;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x004, TxData_2B) != HAL_OK)
        printf("TX 0x004 failed\r\n");
}

void can_tx_steering2(float deg) {
    uint16_t val = (uint16_t)(deg * 10.0f);
    TxData_2B[0] = (val >> 8) & 0xFF;
    TxData_2B[1] = val & 0xFF;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x006, TxData_2B) != HAL_OK)
        printf("TX 0x006 failed\r\n");
}

void can_tx_steering3(float deg) {
    uint16_t val = (uint16_t)(deg * 10.0f);
    TxData_2B[0] = (val >> 8) & 0xFF;
    TxData_2B[1] = val & 0xFF;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x007, TxData_2B) != HAL_OK)
        printf("TX 0x007 failed\r\n");
}

void can_tx_rpm2(float rpm) {
    uint16_t val = (uint16_t)(rpm * 10.0f);
    TxData_2B[0] = (val >> 8) & 0xFF;
    TxData_2B[1] = val & 0xFF;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x008, TxData_2B) != HAL_OK)
        printf("TX 0x008 failed\r\n");
}

void can_tx_throttle(float voltage) {
    uint16_t val = (uint16_t)(voltage * 1000.0f);
    TxData_2B[0] = (val >> 8) & 0xFF;
    TxData_2B[1] = val & 0xFF;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x009, TxData_2B) != HAL_OK)
        printf("TX 0x009 failed\r\n");
}
