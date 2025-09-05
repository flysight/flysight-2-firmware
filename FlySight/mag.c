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

#include <stdbool.h>

#include "main.h"
#include "app_common.h"
#include "config.h"
#include "log.h"
#include "mag.h"
#include "sensor.h"

#define MAG_ADDR              0x3c
#define MAG_REG_WHO_AM_I      (0x4f | 0x80)
#define MAG_REG_CFG_REG_A     (0x60 | 0x80)
#define MAG_REG_CFG_REG_C     (0x62 | 0x80)
#define MAG_REG_OUTX_L_REG    (0x68 | 0x80)
#define MAG_TEMP_OUT_L_REG    (0x6e | 0x80)

#define MAG_INIT_TIMEOUT 1000

typedef enum {
	MAG_ODR_10  = 0,
	MAG_ODR_20  = 1,
	MAG_ODR_50  = 2,
	MAG_ODR_100 = 3
} FS_Mag_ODR_t;

static uint8_t dataBuf[8];
static bool magDataGood;
static FS_Mag_Data_t magData;

typedef enum {
    MAG_STATE_UNINITIALIZED = 0,
    MAG_STATE_INIT_FAILED,
    MAG_STATE_READY,
    MAG_STATE_ACTIVE
} FS_Mag_State_t;

static FS_Mag_State_t magState = MAG_STATE_UNINITIALIZED;

void FS_Mag_Init(void)
{
	uint8_t buf[1];
	HAL_StatusTypeDef result;
	uint32_t timeout;

	// Software reset
	timeout = HAL_GetTick() + MAG_INIT_TIMEOUT;
	do
	{
		buf[0] = 0x20;
		result = FS_Sensor_Write(MAG_ADDR, MAG_REG_CFG_REG_A, buf, 1);
		if (HAL_GetTick() > timeout)
		{
			magState = MAG_STATE_INIT_FAILED;
			return;
		}
	}
	while (result != HAL_OK);

	// Wait for reset
	timeout = HAL_GetTick() + MAG_INIT_TIMEOUT;
	do
	{
		result = FS_Sensor_Read(MAG_ADDR, MAG_REG_CFG_REG_A, buf, 1);
		if (HAL_GetTick() > timeout)
		{
			magState = MAG_STATE_INIT_FAILED;
			return;
		}
	}
	while ((result != HAL_OK) || (buf[0] & 0x20));

	// Check WHO_AM_I register value
	timeout = HAL_GetTick() + MAG_INIT_TIMEOUT;
	do
	{
		result = FS_Sensor_Read(MAG_ADDR, MAG_REG_WHO_AM_I, buf, 1);
		if (HAL_GetTick() > timeout)
		{
			magState = MAG_STATE_INIT_FAILED;
			return;
		}
	}
	while (result != HAL_OK);
	if (buf[0] != 0x40)
	{
		magState = MAG_STATE_INIT_FAILED;
		return;
	}

	// Throw out first temperature measurement
	timeout = HAL_GetTick() + MAG_INIT_TIMEOUT;
	do
	{
		result = FS_Sensor_Read(MAG_ADDR, MAG_REG_OUTX_L_REG, dataBuf, 8);
		if (HAL_GetTick() > timeout)
		{
			magState = MAG_STATE_INIT_FAILED;
			return;
		}
	}
	while (result != HAL_OK);

	magState = MAG_STATE_READY;
}

HAL_StatusTypeDef FS_Mag_Start(void)
{
	const FS_Config_Data_t *config = FS_Config_Get();
	uint8_t buf[1];

	if (magState != MAG_STATE_READY)
	{
		return HAL_ERROR;
	}

	// Enable EXTI pin
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);

	// Temperature compensation; output data rate 10 Hz; continuous mode
	buf[0] = (config->mag_odr << 2) | 0x80;
	if (FS_Sensor_Write(MAG_ADDR, MAG_REG_CFG_REG_A, buf, 1) != HAL_OK)
	{
		FS_Log_WriteEvent("Couldn't start magnetometer");
		LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_6);
		return HAL_ERROR;
	}

	// Configure block data update and data ready on INT_DRDY pin
	buf[0] = 0x11;
	if (FS_Sensor_Write(MAG_ADDR, MAG_REG_CFG_REG_C, buf, 1) != HAL_OK)
	{
		FS_Log_WriteEvent("Couldn't start magnetometer");
		LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_6);
		return HAL_ERROR;
	}

	magState = MAG_STATE_ACTIVE;
	return HAL_OK;
}

void FS_Mag_Stop(void)
{
	uint8_t buf[1];

	// Disable EXTI pin
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_6);

	// Software reset
	buf[0] = 0x20;
	if (FS_Sensor_Write(MAG_ADDR, MAG_REG_CFG_REG_A, buf, 1) != HAL_OK)
	{
		FS_Log_WriteEvent("Couldn't stop magnetometer");
	}
}

static void FS_Mag_Read_Callback_1(HAL_StatusTypeDef result)
{
	if (result == HAL_OK)
	{
		magData.x = (((int16_t) ((dataBuf[1] << 8) | dataBuf[0])) * (int32_t) 3) / 2;
		magData.y = (((int16_t) ((dataBuf[3] << 8) | dataBuf[2])) * (int32_t) 3) / 2;
		magData.z = -(((int16_t) ((dataBuf[5] << 8) | dataBuf[4])) * (int32_t) 3) / 2;
		magDataGood = true;
	}
	else
	{
		FS_Log_WriteEvent("Error reading from magnetometer");
	}
}

static void FS_Mag_Read_Callback_2(HAL_StatusTypeDef result)
{
	if ((result == HAL_OK) && magDataGood)
	{
		magData.temperature = (((int16_t) ((dataBuf[1] << 8) | dataBuf[0])) * (int32_t) 10) / 8 + 250;

		FS_Mag_DataReady_Callback();
	}
	else
	{
		FS_Log_WriteEvent("Error reading from magnetometer");
	}
}

void FS_Mag_Read(void)
{
	if (magState != MAG_STATE_ACTIVE)
	{
		return;
	}

	magData.time = HAL_GetTick();
	magDataGood = false;

	FS_Sensor_ReadAsync(MAG_ADDR, MAG_REG_OUTX_L_REG, dataBuf, 6, FS_Mag_Read_Callback_1);
	FS_Sensor_ReadAsync(MAG_ADDR, MAG_TEMP_OUT_L_REG, dataBuf, 2, FS_Mag_Read_Callback_2);
}

const FS_Mag_Data_t *FS_Mag_GetData(void)
{
	return &magData;
}

__weak void FS_Mag_DataReady_Callback(void)
{
  /* NOTE: This function should not be modified, when the callback is needed,
           the FS_Baro_DataReady_Callback could be implemented in the user file
   */
}
