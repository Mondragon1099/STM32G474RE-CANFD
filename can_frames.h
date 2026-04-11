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
 * 0x000 — AS5600 #1 steering angle     2 bytes
 * 0x001 — dummy frame                  2 bytes
 * 0x002 — ADXL345 #1 (motor 1)         1 byte (0=safe, 1=unsafe)
 * 0x003 — ADXL345 #2 (motor 2)         1 byte (0=safe, 1=unsafe)
 * 0x004 — Hall sensor #1 RPM           2 bytes
 * 0x006 — AS5600 #2 steering angle     2 bytes
 * 0x007 — AS5600 #3 steering angle     2 bytes
 * 0x008 — Hall sensor #2 RPM           2 bytes
 * 0x009 — Throttle 0-3.3V              2 bytes
 * ============================================================ */

/* --- FDCAN handle — defined in main.c, used here --- */
extern FDCAN_HandleTypeDef hfdcan1;

/* ----------------------------------------------------------
 * ID 0x000 — AS5600 #1 steering angle (I2C3)
 * Parameters:
 * deg : steering angle in degrees (0.0 to 360.0)
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
 * unsafe : status flag (0 = safe, 1 = unsafe vibration)
 * Payload: 1 byte
 * ---------------------------------------------------------- */
void can_tx_adxl1(uint8_t unsafe);

/* ----------------------------------------------------------
 * ID 0x003 — ADXL345 #2 vibration (motor 2)
 * Parameters:
 * unsafe : status flag (0 = safe, 1 = unsafe vibration)
 * Payload: 1 byte
 * ---------------------------------------------------------- */
void can_tx_adxl2(uint8_t unsafe);

/* ----------------------------------------------------------
 * ID 0x004 — Hall sensor #1 RPM
 * Parameters:
 * rpm : averaged RPM value (0.0 to 9999.9)
 * Payload: rpm * 10 as uint16, big-endian
 * ---------------------------------------------------------- */
void can_tx_rpm(float rpm);

/* ----------------------------------------------------------
 * ID 0x006 — AS5600 #2 steering angle (I2C4)
 * Parameters:
 * deg : steering angle in degrees (0.0 to 360.0)
 * Payload: deg * 10 as uint16, big-endian
 * ---------------------------------------------------------- */
void can_tx_steering2(float deg);

/* ----------------------------------------------------------
 * ID 0x007 — AS5600 #3 steering angle (I2C2)
 * Parameters:
 * deg : steering angle in degrees (0.0 to 360.0)
 * Payload: deg * 10 as uint16, big-endian
 * ---------------------------------------------------------- */
void can_tx_steering3(float deg);

/* ----------------------------------------------------------
 * ID 0x008 — Hall sensor #2 RPM
 * Parameters:
 * rpm : averaged RPM value (0.0 to 9999.9)
 * Payload: rpm * 10 as uint16, big-endian
 * ---------------------------------------------------------- */
void can_tx_rpm2(float rpm);

/* ----------------------------------------------------------
 * ID 0x009 — Throttle 0-3.3V
 * Parameters:
 * voltage : throttle voltage (0.0 to 3.3)
 * Payload: voltage * 1000 as uint16, big-endian
 * Example: 2.500V → 2500 → [0x09, 0xC4]
 * ---------------------------------------------------------- */
void can_tx_throttle(float voltage);

#endif /* CAN_FRAMES_H */
