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

#ifndef GNSS_BLE_H_
#define GNSS_BLE_H_

#include <stdint.h>
#include "gnss.h"

/* ------------------------------------------------------------------ */
/* Packet layout control                                              */
/* ------------------------------------------------------------------ */
#define GNSS_BLE_MAX_LEN            44u

/* Bit-layout of mask byte (MSB first) */
#define GNSS_BLE_BIT_TOW            0x80u
#define GNSS_BLE_BIT_WEEK           0x40u
#define GNSS_BLE_BIT_POSITION       0x20u
#define GNSS_BLE_BIT_VELOCITY       0x10u
#define GNSS_BLE_BIT_ACCURACY       0x08u
#define GNSS_BLE_BIT_NUM_SV         0x04u

/* ------------------------------------------------------------------ */
/* Public API                                                         */
/* ------------------------------------------------------------------ */
void    GNSS_BLE_Init(void);
uint8_t GNSS_BLE_GetMask(void);
void    GNSS_BLE_SetMask(uint8_t mask);
uint8_t GNSS_BLE_Build(const FS_GNSS_Data_t *src, uint8_t *dst);

#endif /* GNSS_BLE_H_ */
