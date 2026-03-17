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
 *   0x000 — AS5600 steering angle
 *   0x001 — dummy frame (proof of concept)
 * ============================================================ */

/* --- FDCAN handle — defined in main.c, used here --- */
extern FDCAN_HandleTypeDef hfdcan1;

/* --- Frame functions --- */

/* ID 0x000 — AS5600 steering angle
 * Parameters:
 *   deg  : steering angle in degrees (0.0 to 360.0)
 */
void can_tx_steering(float deg);

/* ID 0x001 — dummy frame (proof of concept)
 * No parameters — sends fixed 0xDE 0xAD
 */
void can_tx_dummy(void);

#endif /* CAN_FRAMES_H */
