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
#include "active_control.h"
#include "app_common.h"
#include "app_fatfs.h"
#include "audio.h"
#include "audio_control.h"
#include "baro.h"
#include "config.h"
#include "gnss.h"
#include "hum.h"
#include "imu.h"
#include "log.h"
#include "mag.h"
#include "resource_manager.h"
#include "sensor.h"
#include "state.h"
#include "stm32_seq.h"
#include "vbat.h"

extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

void FS_ActiveMode_Init(void)
{
	uint8_t enable_flags;

	/* Start scanning for BLE peripherals */
    UTIL_SEQ_SetTask(1 << CFG_TASK_START_SCAN_ID, CFG_SCH_PRIO_0);

	/* Initialize FatFS */
	FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS);

	/* Read persistent state */
	FS_State_NextSession();

	/* Initialize controller */
	FS_ActiveControl_Init();

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
		// Initialize enable flags
		enable_flags = FS_LOG_ENABLE_EVENT;
		if (FS_Config_Get()->enable_gnss) enable_flags |= FS_LOG_ENABLE_GNSS;
		if (FS_Config_Get()->enable_baro) enable_flags |= FS_LOG_ENABLE_SENSOR;
		if (FS_Config_Get()->enable_gnss) enable_flags |= FS_LOG_ENABLE_SENSOR;
		if (FS_Config_Get()->enable_hum)  enable_flags |= FS_LOG_ENABLE_SENSOR;
		if (FS_Config_Get()->enable_imu)  enable_flags |= FS_LOG_ENABLE_SENSOR;
		if (FS_Config_Get()->enable_mag)  enable_flags |= FS_LOG_ENABLE_SENSOR;
		if (FS_Config_Get()->enable_vbat) enable_flags |= FS_LOG_ENABLE_SENSOR;
		if (FS_Config_Get()->enable_raw)  enable_flags |= FS_LOG_ENABLE_RAW;

		// Enable logging
		FS_Log_Init(FS_State_Get()->temp_folder, enable_flags);

		// Log timer usage adjusted for:
		//   - FS_ActiveControl_Init (1)
		//   - FS_Log_Init (1)
		FS_Log_WriteEvent("%lu/%lu timers used before active mode initialization",
				HW_TS_CountUsed() - 2, CFG_HW_TS_MAX_NBR_CONCURRENT_TIMER);
		FS_Log_WriteEvent("----------");
	}

	if (FS_Config_Get()->enable_audio)
	{
		/* Initialize audio */
		FS_Audio_Init();

		// Enable audio control
		FS_AudioControl_Init();
	}

	if (FS_Config_Get()->enable_vbat || FS_Config_Get()->enable_mic)
	{
		// Enable ADC
		MX_ADC1_Init();

		// Run the ADC calibration in single-ended mode
		if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
		{
			// Calibration Error
			Error_Handler();
		}
	}

	if (FS_Config_Get()->enable_vbat)
	{
		// Enable battery measurement
		FS_VBAT_Init();
	}

	if (FS_Config_Get()->enable_mic)
	{
		// Enable microphone
		HAL_GPIO_WritePin(MIC_EN_GPIO_Port, MIC_EN_Pin, GPIO_PIN_SET);
	}

	if (FS_Config_Get()->enable_imu)
	{
		/* Start IMU */
		FS_IMU_Start();
	}

	/* Enable USART */
	MX_USART1_UART_Init();

	/* Initialize GNSS */
	FS_GNSS_Init();

	if (FS_Config_Get()->enable_gnss)
	{
		/* Start GNSS */
		FS_GNSS_Start();
	}
	else
	{
		/* Stop GNSS */
		FS_GNSS_Stop();
	}

	if (FS_Config_Get()->enable_baro)
	{
		/* Start barometer */
		FS_Baro_Start();
	}

	if (FS_Config_Get()->enable_hum)
	{
		/* Start humidity and temperature */
		FS_Hum_Start();
	}

	if (FS_Config_Get()->enable_mag)
	{
		/* Start magnetometer */
		FS_Mag_Start();
	}

	if (FS_Config_Get()->enable_baro || FS_Config_Get()->enable_hum || FS_Config_Get()->enable_mag)
	{
		/* Start reading sensors */
		FS_Sensor_Start();
	}
}

void FS_ActiveMode_DeInit(void)
{
	/* Disable controller */
	FS_ActiveControl_DeInit();

	if (FS_Config_Get()->enable_baro || FS_Config_Get()->enable_hum || FS_Config_Get()->enable_mag)
	{
		/* Stop reading sensors */
		FS_Sensor_Stop();
	}

	if (FS_Config_Get()->enable_mag)
	{
		/* Stop magnetometer */
		FS_Mag_Stop();
	}

	if (FS_Config_Get()->enable_hum)
	{
		/* Stop humidity and temperature */
		FS_Hum_Stop();
	}

	if (FS_Config_Get()->enable_baro)
	{
		/* Stop barometer */
		FS_Baro_Stop();
	}

	/* Disable GNSS */
	FS_GNSS_DeInit();

	/* Disable USART */
	HAL_UART_DeInit(&huart1);

	if (FS_Config_Get()->enable_imu)
	{
		/* Stop IMU */
		FS_IMU_Stop();
	}

	if (FS_Config_Get()->enable_mic)
	{
		// Disable microphone
		HAL_GPIO_WritePin(MIC_EN_GPIO_Port, MIC_EN_Pin, GPIO_PIN_RESET);
	}

	if (FS_Config_Get()->enable_vbat)
	{
		// Disable battery measurement
		FS_VBAT_DeInit();
	}

	if (FS_Config_Get()->enable_vbat || FS_Config_Get()->enable_mic)
	{
		// Disable ADC
		if (HAL_ADC_DeInit(&hadc1) != HAL_OK)
		{
			Error_Handler();
		}
	}

	if (FS_Config_Get()->enable_audio)
	{
		// Disable audio control
		FS_AudioControl_DeInit();

		// Disable audio
		FS_Audio_DeInit();
	}

	if (FS_Config_Get()->enable_logging)
	{
		// Log timer usage adjusted for:
		//   - FS_Log_DeInit (1)
		FS_Log_WriteEvent("----------");
		FS_Log_WriteEvent("%lu/%lu timers used after active mode de-initialization",
				HW_TS_CountUsed() - 1, CFG_HW_TS_MAX_NBR_CONCURRENT_TIMER);

		// Disable logging
		FS_Log_DeInit(FS_State_Get()->temp_folder);
	}

	/* De-initialize FatFS */
	FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
}
