/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2023 Bionic Avionics Inc.                                   **
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

#include "main.h"
#include "app_common.h"

char *writeInt32ToBuf(char *ptr, int32_t val, int8_t dec, int8_t dot, char delimiter)
{
    int32_t value = val > 0 ? val : -val;

    *--ptr = delimiter;
    while (value > 0 || dec > 0)
    {
        ldiv_t res = ldiv(value, 10);
        *--ptr = res.rem + '0';
        value = res.quot;
        if (--dec == 0 && dot)
        {
            *--ptr = '.';
        }
    }
    if (*ptr == '.' || *ptr == delimiter)
    {
        *--ptr = '0';
    }
    if (val < 0)
    {
        *--ptr = '-';
    }

	return ptr;
}
