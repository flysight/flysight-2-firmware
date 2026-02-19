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

#include "accel_ble.h"

static uint8_t s_mask = ACCEL_BLE_DEFAULT_MASK;
static uint8_t s_divider = 1u;  /* 1 = every sample, 2 = every 2nd sample, etc. */

void ACCEL_BLE_Init(void)
{
    s_mask = ACCEL_BLE_DEFAULT_MASK;
    s_divider = 1u;
}

uint8_t ACCEL_BLE_GetMask(void)
{
    return s_mask;
}

void ACCEL_BLE_SetMask(uint8_t mask)
{
    s_mask = mask;
}

uint8_t ACCEL_BLE_GetDivider(void)
{
    return s_divider;
}

void ACCEL_BLE_SetDivider(uint8_t divider)
{
    if (divider == 0) divider = 1;  /* Minimum divider is 1 */
    s_divider = divider;
}

uint8_t ACCEL_BLE_Build(const FS_IMU_Data_t *src, uint8_t *dst)
{
    uint8_t *p = dst;

    *p++ = s_mask;                                          /* byte 0 : mask        */

    if (s_mask & ACCEL_BLE_BIT_TIME) {                      /* time (ms)            */
        memcpy(p, &src->time, sizeof(src->time));         p += 4;
    }

    if (s_mask & ACCEL_BLE_BIT_ACCEL) {                     /* accel (g * 100000)   */
        memcpy(p, &src->ax, sizeof(src->ax));             p += 4;
        memcpy(p, &src->ay, sizeof(src->ay));             p += 4;
        memcpy(p, &src->az, sizeof(src->az));             p += 4;
    }

    if (s_mask & ACCEL_BLE_BIT_TEMPERATURE) {               /* temperature (C * 100)*/
        memcpy(p, &src->temperature, sizeof(src->temperature)); p += 2;
    }

    return (uint8_t)(p - dst);                              /* total payload length */
}
