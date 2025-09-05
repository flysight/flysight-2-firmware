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
#include "hts221.h"
#include "hum.h"
#include "log.h"
#include "sht4x.h"

#define HUM_INIT_TIMEOUT 1000

typedef struct
{
	HAL_StatusTypeDef (*Start)(void);
	HAL_StatusTypeDef (*Stop)(void);
} FS_Hum_Interface_t;

static FS_Hum_Interface_t humInterface;
static FS_Hum_Data_t humData;

typedef enum {
    HUM_STATE_UNINITIALIZED = 0,
    HUM_STATE_INIT_FAILED,
    HUM_STATE_READY,
    HUM_STATE_ACTIVE
} FS_Hum_State_t;

static FS_Hum_State_t humState = HUM_STATE_UNINITIALIZED;

void FS_Hum_Init(void)
{
	uint32_t timeout;

	timeout = HAL_GetTick() + HUM_INIT_TIMEOUT;
	while (1)
	{
		if (HAL_GetTick() > timeout)
		{
			humState = HUM_STATE_INIT_FAILED;
			return;
		}

		// Check for SHT4x
		if (FS_SHT4X_Init(&humData) == HAL_OK)
		{
			humInterface.Start = &FS_SHT4X_Start;
			humInterface.Stop = &FS_SHT4X_Stop;
			break;
		}

		// Check for HTS221
		if (FS_HTS221_Init(&humData) == HAL_OK)
		{
			humInterface.Start = &FS_HTS221_Start;
			humInterface.Stop = &FS_HTS221_Stop;
			break;
		}
	}

	humState = HUM_STATE_READY;
}

HAL_StatusTypeDef FS_Hum_Start(void)
{
	if (humState != HUM_STATE_READY)
	{
		return HAL_ERROR;
	}

	if ((*humInterface.Start)() != HAL_OK)
	{
		FS_Log_WriteEvent("Couldn't start humidity sensor");
		return HAL_ERROR;
	}

	humState = HUM_STATE_ACTIVE;
	return HAL_OK;
}

void FS_Hum_Stop(void)
{
	if ((*humInterface.Stop)() != HAL_OK)
	{
		FS_Log_WriteEvent("Couldn't stop humidity sensor");
	}

	humState = HUM_STATE_READY;
}

void FS_Hum_Read(void)
{
	if (humState != HUM_STATE_ACTIVE)
	{
		return;
	}

	FS_HTS221_Read();
}

const FS_Hum_Data_t *FS_Hum_GetData(void)
{
	return &humData;
}

__weak void FS_Hum_DataReady_Callback(void)
{
  /* NOTE: This function should not be modified, when the callback is needed,
           the FS_Hum_DataReady_Callback could be implemented in the user file
   */
}
