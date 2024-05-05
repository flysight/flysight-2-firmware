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

#define TIMEOUT_VALUE 100

extern RNG_HandleTypeDef hrng;

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

void FS_Common_GetRandomBytes(uint32_t *buf, uint32_t count)
{
	HAL_StatusTypeDef res;
	uint32_t counter;
	uint32_t tickstart;

	/* Algorithm to use RNG on CPU1 comes from AN5289 Figure 8 */

	/* Poll Sem0 until granted */
	LL_HSEM_1StepLock(HSEM, CFG_HW_RNG_SEMID);

	/* Configure and switch on RNG clock*/
	MX_RNG_Init();

	/* Generate random session ID */
	for (counter = 0; counter < count; ++counter)
	{
	    tickstart = HAL_GetTick();

	    res = HAL_ERROR;
		while ((res != HAL_OK) && (HAL_GetTick() - tickstart < TIMEOUT_VALUE))
		{
			res = HAL_RNG_GenerateRandomNumber(&hrng, &buf[counter]);
		}

		if (res != HAL_OK)
		{
			Error_Handler();
		}
	}

	/* Switch off RNG IP and clock */
	HAL_RNG_DeInit(&hrng);

	/* Set RNGSEL = CLK48 */
    LL_RCC_SetRNGClockSource(RCC_RNGCLKSOURCE_CLK48);

	/* Release Sem0 */
	LL_HSEM_ReleaseLock(HSEM, CFG_HW_RNG_SEMID, 0);
}
