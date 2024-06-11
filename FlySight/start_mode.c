/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2024 Bionic Avionics Inc.                                   **
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
#include "app_fatfs.h"
#include "audio.h"
#include "config.h"
#include "gnss.h"
#include "log.h"
#include "resource_manager.h"
#include "start_control.h"
#include "state.h"

extern UART_HandleTypeDef huart1;

void FS_StartMode_Init(void)
{
	/* Initialize FatFS */
	FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS);

	/* Read persistent state */
	FS_State_NextSession();

	/* Initialize controller */
	FS_StartControl_Init();

	/* Initialize configuration */
	FS_Config_Init();
	if (FS_Config_Read("/config.txt") != FS_CONFIG_OK)
	{
		FS_Config_Write("/config.txt");
	}

	/* Read selectable config */
	if (f_chdir("/config") == FR_OK)
	{
		FS_Config_Read(FS_State_Get()->config_filename);
	}

	if (FS_Config_Get()->enable_logging)
	{
		// Enable logging
		FS_Log_Init(FS_State_Get()->temp_folder, FS_LOG_ENABLE_EVENT);

		// Log timer usage adjusted for:
		//   - FS_StartControl_Init (2)
		//   - FS_Log_Init (1)
		FS_Log_WriteEvent("%lu/%lu timers used before start mode initialization",
				HW_TS_CountUsed() - 3, CFG_HW_TS_MAX_NBR_CONCURRENT_TIMER);
	}

	if (FS_Config_Get()->enable_audio)
	{
		/* Initialize audio */
		FS_Audio_Init();
	}

	/* Enable USART */
	MX_USART1_UART_Init();

	/* Initialize GNSS */
	FS_GNSS_Init();

	/* Start GNSS */
	FS_GNSS_Start();
}

void FS_StartMode_DeInit(void)
{
	/* Disable controller */
	FS_StartControl_DeInit();

	/* Disable GNSS */
	FS_GNSS_DeInit();

	/* Disable USART */
	HAL_UART_DeInit(&huart1);

	if (FS_Config_Get()->enable_audio)
	{
		// Disable audio
		FS_Audio_DeInit();
	}

	if (FS_Config_Get()->enable_logging)
	{
		// Log timer usage adjusted for:
		//   - FS_Log_DeInit (1)
		FS_Log_WriteEvent("----------");
		FS_Log_WriteEvent("%lu/%lu timers used after start mode de-initialization",
				HW_TS_CountUsed() - 1, CFG_HW_TS_MAX_NBR_CONCURRENT_TIMER);

		// Disable logging
		FS_Log_DeInit(FS_State_Get()->temp_folder);
	}

	/* De-initialize FatFS */
	FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
}
