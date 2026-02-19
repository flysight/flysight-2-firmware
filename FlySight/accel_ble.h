/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2025 Bionic Avionics Inc.                                   **
**                                                                        **
**  This program is free software: you can redistribute it and/or modify  **
**  it under the terms of the GNU General Public License as published by  **
**  the Free Software Foundation, either version 3 of the License, or     **
**  (at your option) any later version.                                   **
**                                                                        **
**  This program is distributed in the hope that it will be useful,       **
**  but WITHOUT ANY WARRANTY; without even the implied warranty of        **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         **
**  GNU General Public License for more details.                          **
**                                                                        **
**  You should have received a copy of the GNU General Public License     **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>. **
**                                                                        **
****************************************************************************
**  Contact: Bionic Avionics Inc.                                         **
**  Website: http://flysight.ca/                                          **
****************************************************************************/

#ifndef ACCEL_BLE_H_
#define ACCEL_BLE_H_

#include <stdint.h>
#include "stm32wbxx_hal.h"
#include "imu.h"

/* ------------------------------------------------------------------ */
/* Packet layout control                                              */
/* ------------------------------------------------------------------ */
#define ACCEL_BLE_MAX_LEN           20u

/* Bit-layout of mask byte (MSB first) */
#define ACCEL_BLE_BIT_TIME          0x80u
#define ACCEL_BLE_BIT_ACCEL         0x40u
#define ACCEL_BLE_BIT_TEMPERATURE   0x20u

/* Default mask: all fields enabled */
#define ACCEL_BLE_DEFAULT_MASK      0xE0u

/* ------------------------------------------------------------------ */
/* Public API                                                         */
/* ------------------------------------------------------------------ */
void    ACCEL_BLE_Init(void);
uint8_t ACCEL_BLE_GetMask(void);
void    ACCEL_BLE_SetMask(uint8_t mask);
uint8_t ACCEL_BLE_GetDivider(void);
void    ACCEL_BLE_SetDivider(uint8_t divider);
uint8_t ACCEL_BLE_Build(const FS_IMU_Data_t *src, uint8_t *dst);

#endif /* ACCEL_BLE_H_ */
