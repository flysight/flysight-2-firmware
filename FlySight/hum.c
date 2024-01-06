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

typedef struct
{
	void (*Start)(void);
	void (*Stop)(void);
	void (*Read)(FS_Hum_Data_t *data);
} FS_Hum_Interface_t;

static FS_Hum_Interface_t humInterface;
static FS_Hum_Data_t humData;

void FS_Hum_Init(void)
{
	if (FS_HTS221_Init() == FS_HUM_OK)
	{
		humInterface.Start = &FS_HTS221_Start;
		humInterface.Stop = &FS_HTS221_Stop;
		humInterface.Read = &FS_HTS221_Read;
	}
	else
	{
		Error_Handler();	// Should never be called
	}
}

void FS_Hum_Start(void)
{
	(*humInterface.Start)();
}

void FS_Hum_Stop(void)
{
	(*humInterface.Stop)();
}

void FS_Hum_Read(void)
{
	(*humInterface.Read)(&humData);
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
