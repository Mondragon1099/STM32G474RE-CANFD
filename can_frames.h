#ifndef CAN_FRAMES_H
#define CAN_FRAMES_H

#include "main.h"

/* ============================================================
 * can_frames.h
 * Declarations for all CAN FD TX frame functions.
 *
 * Each function takes sensor values as parameters,
 * builds the payload, and transmits the frame.
 *
 * Frame ID map:
 *   0x000 — AS5600 #1 steering angle     2 bytes
 *   0x001 — dummy frame                  2 bytes
 *   0x002 — ADXL345 #1 (motor 1)         8 bytes (6 data + 2 pad)
 *   0x003 — ADXL345 #2 (motor 2)         8 bytes (6 data + 2 pad)
 *   0x004 — Hall sensor #1 RPM           2 bytes
 *   0x006 — AS5600 #2 steering angle     2 bytes
 *   0x007 — AS5600 #3 steering angle     2 bytes
 *   0x008 — Hall sensor #2 RPM           2 bytes
 *   0x009 — Throttle 0-5V               2 bytes
 * ============================================================ */

/* --- FDCAN handle — defined in main.c, used here --- */
extern FDCAN_HandleTypeDef hfdcan1;

/* ----------------------------------------------------------
 * ID 0x000 — AS5600 #1 steering angle (I2C3)
 * Parameters:
 *   deg : steering angle in degrees (0.0 to 360.0)
 * Payload: deg * 10 as uint16, big-endian
 * ---------------------------------------------------------- */
void can_tx_steering1(float deg);

/* ----------------------------------------------------------
 * ID 0x001 — dummy frame (proof of concept)
 * No parameters — sends fixed 0xDE 0xAD
 * ---------------------------------------------------------- */
void can_tx_dummy(void);

/* ----------------------------------------------------------
 * ID 0x002 — ADXL345 #1 vibration (motor 1)
 * Parameters:
 *   x, y, z : calibrated acceleration in g (-16.0 to +16.0)
 * Payload: each axis * 100 as int16, big-endian (8 bytes, 6 data + 2 pad)
 * ---------------------------------------------------------- */
void can_tx_adxl1(float x, float y, float z);

/* ----------------------------------------------------------
 * ID 0x003 — ADXL345 #2 vibration (motor 2)
 * Parameters:
 *   x, y, z : calibrated acceleration in g (-16.0 to +16.0)
 * Payload: each axis * 100 as int16, big-endian (8 bytes, 6 data + 2 pad)
 * ---------------------------------------------------------- */
void can_tx_adxl2(float x, float y, float z);

/* ----------------------------------------------------------
 * ID 0x004 — Hall sensor #1 RPM
 * Parameters:
 *   rpm : averaged RPM value (0.0 to 9999.9)
 * Payload: rpm * 10 as uint16, big-endian
 * ---------------------------------------------------------- */
void can_tx_rpm(float rpm);

/* ----------------------------------------------------------
 * ID 0x006 — AS5600 #2 steering angle (I2C4)
 * Parameters:
 *   deg : steering angle in degrees (0.0 to 360.0)
 * Payload: deg * 10 as uint16, big-endian
 * ---------------------------------------------------------- */
void can_tx_steering2(float deg);

/* ----------------------------------------------------------
 * ID 0x007 — AS5600 #3 steering angle (I2C2)
 * Parameters:
 *   deg : steering angle in degrees (0.0 to 360.0)
 * Payload: deg * 10 as uint16, big-endian
 * ---------------------------------------------------------- */
void can_tx_steering3(float deg);

/* ----------------------------------------------------------
 * ID 0x008 — Hall sensor #2 RPM
 * Parameters:
 *   rpm : averaged RPM value (0.0 to 9999.9)
 * Payload: rpm * 10 as uint16, big-endian
 * ---------------------------------------------------------- */
void can_tx_rpm2(float rpm);

/* ----------------------------------------------------------
 * ID 0x009 — Throttle 0-5V
 * Parameters:
 *   voltage : throttle voltage (0.0 to 5.0)
 * Payload: voltage * 1000 as uint16, big-endian
 * Example: 3.312V → 3312 → [0x0C, 0xF0]
 *
 * Node-RED decode:
 *   let val = (data[0] << 8) | data[1];
 *   let voltage = val / 1000.0;
 * ---------------------------------------------------------- */
void can_tx_throttle(float voltage);

#endif /* CAN_FRAMES_H */
