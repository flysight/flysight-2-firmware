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

#include <string.h>

#include "gnss_ble.h"

static uint8_t s_mask = 0xB0u;

void GNSS_BLE_Init(void)
{
    s_mask = 0xB0u;
}

uint8_t GNSS_BLE_GetMask(void)
{
    return s_mask;
}

void GNSS_BLE_SetMask(uint8_t mask)
{
    s_mask = mask;
}

uint8_t GNSS_BLE_Build(const FS_GNSS_Data_t *src, uint8_t *dst)
{
    uint8_t *p = dst;

    *p++ = s_mask;                                    /* byte 0 : mask        */

    if (s_mask & GNSS_BLE_BIT_TOW) {                  /* time of week         */
        memcpy(p, &src->iTOW, sizeof(src->iTOW));   p += 4;
    }

    if (s_mask & GNSS_BLE_BIT_WEEK) {                 /* week number          */
    	// Not yet implemented
    }

    if (s_mask & GNSS_BLE_BIT_POSITION) {             /* position             */
        memcpy(p, &src->lon,  sizeof(src->lon));    p += 4;
        memcpy(p, &src->lat,  sizeof(src->lat));    p += 4;
        memcpy(p, &src->hMSL, sizeof(src->hMSL));   p += 4;
    }

    if (s_mask & GNSS_BLE_BIT_VELOCITY) {             /* velocity             */
        memcpy(p, &src->velN, sizeof(src->velN));   p += 4;
        memcpy(p, &src->velE, sizeof(src->velE));   p += 4;
        memcpy(p, &src->velD, sizeof(src->velD));   p += 4;
    }

    if (s_mask & GNSS_BLE_BIT_ACCURACY) {             /* accuracy             */
        memcpy(p, &src->hAcc, sizeof(src->hAcc));   p += 4;
        memcpy(p, &src->vAcc, sizeof(src->vAcc));   p += 4;
        memcpy(p, &src->sAcc, sizeof(src->sAcc));   p += 4;
    }

    if (s_mask & GNSS_BLE_BIT_NUM_SV) {               /* number of satellites */
        memcpy(p, &src->numSV, sizeof(src->numSV));  p += 1;
    }

    return (uint8_t)(p - dst);                        /* total payload length */
}
