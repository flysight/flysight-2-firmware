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
#include "app_fatfs.h"
#include "audio.h"
#include "audio_control.h"
#include "baro.h"
#include "config.h"
#include "control.h"
#include "gnss.h"
#include "hum.h"
#include "imu.h"
#include "log.h"
#include "mag.h"
#include "resource_manager.h"
#include "sensor.h"
#include "state.h"
#include "vbat.h"

extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

void FS_ActiveMode_Init(void)
{
	/* Initialize controller */
	FS_Control_Init();

	/* Enable charging */
	HAL_GPIO_WritePin(CHG_EN_LO_GPIO_Port, CHG_EN_LO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CHG_EN_HI_GPIO_Port, CHG_EN_HI_Pin, GPIO_PIN_SET);

	/* Initialize FatFS */
	FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS);

	/* Read persistent state */
	FS_State_Init();

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
		FS_Log_Init(FS_State_Get()->temp_folder);
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
	FS_Control_DeInit();

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
		// Disable logging
		FS_Log_DeInit(FS_State_Get()->temp_folder);
	}

	/* De-initialize FatFS */
	FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);

	/* Disable charging */
	HAL_GPIO_WritePin(CHG_EN_LO_GPIO_Port, CHG_EN_LO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CHG_EN_HI_GPIO_Port, CHG_EN_HI_Pin, GPIO_PIN_SET);
}
