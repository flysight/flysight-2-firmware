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

#ifndef MAG_BLE_H_
#define MAG_BLE_H_

#include <stdint.h>
#include "stm32wbxx_hal.h"
#include "mag.h"

/* ------------------------------------------------------------------ */
/* Packet layout control                                              */
/* ------------------------------------------------------------------ */
#define MAG_BLE_MAX_LEN            14u

/* Bit-layout of mask byte (MSB first) */
#define MAG_BLE_BIT_TIME           0x80u
#define MAG_BLE_BIT_MAG            0x40u
#define MAG_BLE_BIT_TEMPERATURE    0x20u

/* Default mask: all fields enabled */
#define MAG_BLE_DEFAULT_MASK       0xE0u

/* ------------------------------------------------------------------ */
/* Public API                                                         */
/* ------------------------------------------------------------------ */
void    MAG_BLE_Init(void);
uint8_t MAG_BLE_GetMask(void);
void    MAG_BLE_SetMask(uint8_t mask);
uint8_t MAG_BLE_GetDivider(void);
void    MAG_BLE_SetDivider(uint8_t divider);
uint8_t MAG_BLE_Build(const FS_Mag_Data_t *src, uint8_t *dst);

#endif /* MAG_BLE_H_ */
