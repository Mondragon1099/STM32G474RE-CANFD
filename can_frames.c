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
 * Encoding conventions:
 *   - All multi-byte values are big-endian (high byte first)
 *   - Floats with decimals are scaled to integers before packing:
 *       degrees  → x10   as uint16  (e.g. 270.5 → 2705)
 *       g-force  → x100  as int16   (e.g. -0.98 → -98)
 *       RPM      → x10   as uint16  (e.g. 123.4 → 1234)
 *       voltage  → x1000 as uint16  (e.g. 3.312 → 3312)
 *
 * To add a new sensor frame:
 *   1. Add TX header + data buffer below
 *   2. Add a new function following the same pattern
 *   3. Declare it in can_frames.h
 *   4. Call it from main.c while loop
 * ============================================================ */

#include "can_frames.h"
#include "string.h"
#include "stdio.h"

/* ============================================================
 * TX headers — one per frame ID
 * static = only visible inside this file
 * ============================================================ */

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

static FDCAN_TxHeaderTypeDef TxHeader_0x002 = {
    .Identifier          = 0x002,
    .IdType              = FDCAN_STANDARD_ID,
    .TxFrameType         = FDCAN_DATA_FRAME,
    .DataLength          = FDCAN_DLC_BYTES_8,   /* 6 bytes data + 2 padding */
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch       = FDCAN_BRS_OFF,
    .FDFormat            = FDCAN_FD_CAN,
    .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
    .MessageMarker       = 0
};

static FDCAN_TxHeaderTypeDef TxHeader_0x003 = {
    .Identifier          = 0x003,
    .IdType              = FDCAN_STANDARD_ID,
    .TxFrameType         = FDCAN_DATA_FRAME,
    .DataLength          = FDCAN_DLC_BYTES_8,   /* 6 bytes data + 2 padding */
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch       = FDCAN_BRS_OFF,
    .FDFormat            = FDCAN_FD_CAN,
    .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
    .MessageMarker       = 0
};

static FDCAN_TxHeaderTypeDef TxHeader_0x004 = {
    .Identifier          = 0x004,
    .IdType              = FDCAN_STANDARD_ID,
    .TxFrameType         = FDCAN_DATA_FRAME,
    .DataLength          = FDCAN_DLC_BYTES_2,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch       = FDCAN_BRS_OFF,
    .FDFormat            = FDCAN_FD_CAN,
    .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
    .MessageMarker       = 0
};

static FDCAN_TxHeaderTypeDef TxHeader_0x006 = {
    .Identifier          = 0x006,
    .IdType              = FDCAN_STANDARD_ID,
    .TxFrameType         = FDCAN_DATA_FRAME,
    .DataLength          = FDCAN_DLC_BYTES_2,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch       = FDCAN_BRS_OFF,
    .FDFormat            = FDCAN_FD_CAN,
    .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
    .MessageMarker       = 0
};

static FDCAN_TxHeaderTypeDef TxHeader_0x007 = {
    .Identifier          = 0x007,
    .IdType              = FDCAN_STANDARD_ID,
    .TxFrameType         = FDCAN_DATA_FRAME,
    .DataLength          = FDCAN_DLC_BYTES_2,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch       = FDCAN_BRS_OFF,
    .FDFormat            = FDCAN_FD_CAN,
    .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
    .MessageMarker       = 0
};

static FDCAN_TxHeaderTypeDef TxHeader_0x008 = {
    .Identifier          = 0x008,
    .IdType              = FDCAN_STANDARD_ID,
    .TxFrameType         = FDCAN_DATA_FRAME,
    .DataLength          = FDCAN_DLC_BYTES_2,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch       = FDCAN_BRS_OFF,
    .FDFormat            = FDCAN_FD_CAN,
    .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
    .MessageMarker       = 0
};

static FDCAN_TxHeaderTypeDef TxHeader_0x009 = {
    .Identifier          = 0x009,
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
static uint8_t TxData_0x002[8];
static uint8_t TxData_0x003[8];
static uint8_t TxData_0x004[2];
static uint8_t TxData_0x006[2];
static uint8_t TxData_0x007[2];
static uint8_t TxData_0x008[2];
static uint8_t TxData_0x009[2];

/* ============================================================
 * Frame functions
 * ============================================================ */

/* ----------------------------------------------------------
 * can_tx_steering1 — ID 0x000
 * AS5600 #1 magnetic steering angle (I2C3)
 *
 * Encoding: deg * 10 → uint16, big-endian
 * Example:  270.5 deg → 2705 → [0x0A, 0x91]
 *
 * Node-RED decode:
 *   let val = (data[0] << 8) | data[1];
 *   let deg = val / 10.0;
 * ---------------------------------------------------------- */
void can_tx_steering1(float deg)
{
    uint16_t val = (uint16_t)(deg * 10.0f);
    TxData_0x000[0] = (val >> 8) & 0xFF;
    TxData_0x000[1] =  val       & 0xFF;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x000, TxData_0x000) != HAL_OK)
        printf("TX 0x000 failed\r\n");
}

/* ----------------------------------------------------------
 * can_tx_dummy — ID 0x001
 * Proof of concept frame, fixed payload
 * ---------------------------------------------------------- */
void can_tx_dummy(void)
{
    TxData_0x001[0] = 0xDE;
    TxData_0x001[1] = 0xAD;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x001, TxData_0x001) != HAL_OK)
        printf("TX 0x001 failed\r\n");
}

/* ----------------------------------------------------------
 * can_tx_adxl1 — ID 0x002
 * ADXL345 #1 motor 1 vibration (calibrated g values)
 *
 * Encoding: each axis * 100 → int16, big-endian
 * Example:  -0.98g → -98 → [0xFF, 0x9E]
 * Bytes 6-7 are zero padding.
 *
 * Node-RED decode (signed int16 needs sign extension):
 *   let x_raw = (data[0] << 8) | data[1];
 *   if (x_raw > 32767) x_raw -= 65536;
 *   let x = x_raw / 100.0;
 *   // repeat for y (data[2-3]) and z (data[4-5])
 * ---------------------------------------------------------- */
void can_tx_adxl1(float x, float y, float z)
{
    int16_t xi = (int16_t)(x * 100.0f);
    int16_t yi = (int16_t)(y * 100.0f);
    int16_t zi = (int16_t)(z * 100.0f);

    memset(TxData_0x002, 0, sizeof(TxData_0x002));
    TxData_0x002[0] = (uint8_t)((xi >> 8) & 0xFF);
    TxData_0x002[1] = (uint8_t)( xi       & 0xFF);
    TxData_0x002[2] = (uint8_t)((yi >> 8) & 0xFF);
    TxData_0x002[3] = (uint8_t)( yi       & 0xFF);
    TxData_0x002[4] = (uint8_t)((zi >> 8) & 0xFF);
    TxData_0x002[5] = (uint8_t)( zi       & 0xFF);
    /* bytes 6-7 = 0x00 padding */

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x002, TxData_0x002) != HAL_OK)
        printf("TX 0x002 failed\r\n");
}

/* ----------------------------------------------------------
 * can_tx_adxl2 — ID 0x003
 * ADXL345 #2 motor 2 vibration (calibrated g values)
 * Same encoding as can_tx_adxl1
 * ---------------------------------------------------------- */
void can_tx_adxl2(float x, float y, float z)
{
    int16_t xi = (int16_t)(x * 100.0f);
    int16_t yi = (int16_t)(y * 100.0f);
    int16_t zi = (int16_t)(z * 100.0f);

    memset(TxData_0x003, 0, sizeof(TxData_0x003));
    TxData_0x003[0] = (uint8_t)((xi >> 8) & 0xFF);
    TxData_0x003[1] = (uint8_t)( xi       & 0xFF);
    TxData_0x003[2] = (uint8_t)((yi >> 8) & 0xFF);
    TxData_0x003[3] = (uint8_t)( yi       & 0xFF);
    TxData_0x003[4] = (uint8_t)((zi >> 8) & 0xFF);
    TxData_0x003[5] = (uint8_t)( zi       & 0xFF);
    /* bytes 6-7 = 0x00 padding */

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x003, TxData_0x003) != HAL_OK)
        printf("TX 0x003 failed\r\n");
}

/* ----------------------------------------------------------
 * can_tx_rpm — ID 0x004
 * Hall sensor #1 averaged RPM
 *
 * Encoding: rpm * 10 → uint16, big-endian
 * Example:  123.4 RPM → 1234 → [0x04, 0xD2]
 *
 * Node-RED decode:
 *   let val = (data[0] << 8) | data[1];
 *   let rpm = val / 10.0;
 * ---------------------------------------------------------- */
void can_tx_rpm(float rpm)
{
    uint16_t val = (uint16_t)(rpm * 10.0f);
    TxData_0x004[0] = (val >> 8) & 0xFF;
    TxData_0x004[1] =  val       & 0xFF;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x004, TxData_0x004) != HAL_OK)
        printf("TX 0x004 failed\r\n");
}

/* ----------------------------------------------------------
 * can_tx_steering2 — ID 0x006
 * AS5600 #2 magnetic steering angle (I2C4)
 * Same encoding as can_tx_steering1
 * ---------------------------------------------------------- */
void can_tx_steering2(float deg)
{
    uint16_t val = (uint16_t)(deg * 10.0f);
    TxData_0x006[0] = (val >> 8) & 0xFF;
    TxData_0x006[1] =  val       & 0xFF;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x006, TxData_0x006) != HAL_OK)
        printf("TX 0x006 failed\r\n");
}

/* ----------------------------------------------------------
 * can_tx_steering3 — ID 0x007
 * AS5600 #3 magnetic steering angle (I2C2)
 * Same encoding as can_tx_steering1
 *
 * Node-RED decode:
 *   let val = (data[0] << 8) | data[1];
 *   let deg = val / 10.0;
 * ---------------------------------------------------------- */
void can_tx_steering3(float deg)
{
    uint16_t val = (uint16_t)(deg * 10.0f);
    TxData_0x007[0] = (val >> 8) & 0xFF;
    TxData_0x007[1] =  val       & 0xFF;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x007, TxData_0x007) != HAL_OK)
        printf("TX 0x007 failed\r\n");
}

/* ----------------------------------------------------------
 * can_tx_rpm2 — ID 0x008
 * Hall sensor #2 averaged RPM
 * Same encoding as can_tx_rpm
 *
 * Node-RED decode:
 *   let val = (data[0] << 8) | data[1];
 *   let rpm = val / 10.0;
 * ---------------------------------------------------------- */
void can_tx_rpm2(float rpm)
{
    uint16_t val = (uint16_t)(rpm * 10.0f);
    TxData_0x008[0] = (val >> 8) & 0xFF;
    TxData_0x008[1] =  val       & 0xFF;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x008, TxData_0x008) != HAL_OK)
        printf("TX 0x008 failed\r\n");
}

/* ----------------------------------------------------------
 * can_tx_throttle — ID 0x009
 * Throttle 0-5V ADC reading
 *
 * Encoding: voltage * 1000 → uint16, big-endian
 * Example:  3.312V → 3312 → [0x0C, 0xF0]
 *
 * Node-RED decode:
 *   let val = (data[0] << 8) | data[1];
 *   let voltage = val / 1000.0;
 * ---------------------------------------------------------- */
void can_tx_throttle(float voltage)
{
    uint16_t val = (uint16_t)(voltage * 1000.0f);
    TxData_0x009[0] = (val >> 8) & 0xFF;
    TxData_0x009[1] =  val       & 0xFF;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x009, TxData_0x009) != HAL_OK)
        printf("TX 0x009 failed\r\n");
}
