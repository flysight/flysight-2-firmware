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
#include "config.h"
#include "hts221.h"
#include "hum.h"
#include "log.h"
#include "sensor.h"

#define HTS221_ADDR               0xbe
#define HTS221_REG_WHO_AM_I       (0x0f | 0x80)
#define HTS221_REG_CTRL_REG1      (0x20 | 0x80)
#define HTS221_REG_CTRL_REG2      (0x21 | 0x80)
#define HTS221_REG_CTRL_REG3      (0x22 | 0x80)
#define HTS221_REG_HUMIDITY_OUT_L (0x28 | 0x80)
#define HTS221_REG_H0_RH_X2       (0x30 | 0x80)
#define HTS221_REG_T0_DEGC_X8     (0x32 | 0x80)
#define HTS221_REG_T0_T1_MSB      (0x35 | 0x80)
#define HTS221_REG_H0_T0_OUT      (0x36 | 0x80)
#define HTS221_REG_H1_T0_OUT      (0x3a | 0x80)
#define HTS221_REG_T0_OUT         (0x3c | 0x80)

typedef enum {
	HTS221_ODR_OS   = 0,
	HTS221_ODR_1    = 1,
	HTS221_ODR_7    = 2,
	HTS221_ODR_12_5 = 3
} FS_HTS221_ODR_t;

static uint8_t H0_rH_x2;
static uint8_t H1_rH_x2;

static int16_t H0_T0_out;
static int16_t H1_T0_out;

static uint16_t T0_degC_x8_u16;
static uint16_t T1_degC_x8_u16;

static int16_t T0_out;
static int16_t T1_out;

static uint8_t dataBuf[4];

static FS_Hum_Data_t *humData;

static volatile bool sensor_is_busy;

HAL_StatusTypeDef FS_HTS221_Init(FS_Hum_Data_t *data)
{
	uint8_t buf[4];

	uint8_t temp;
	HAL_StatusTypeDef result;

	// Keep local pointer to humidity data
	humData = data;

	// Read WHO_AM_I register value
	if (FS_Sensor_Read(HTS221_ADDR, HTS221_REG_WHO_AM_I, buf, 1) != HAL_OK)
	{
		return HAL_ERROR;
	}

	// Check WHO_AM_I register value
	if (buf[0] != 0xbc)
	{
		return HAL_ERROR;
	}

	// Software reset
	do
	{
		buf[0] = 0x80;
		result = FS_Sensor_Write(HTS221_ADDR, HTS221_REG_CTRL_REG2, buf, 1);
	}
	while (result != HAL_OK);

	// Wait for reset
	do
	{
		result = FS_Sensor_Read(HTS221_ADDR, HTS221_REG_CTRL_REG2, buf, 1);
	}
	while ((result != HAL_OK) || (buf[0] & 0x80));

	// Read humidity calibration coefficients
	do
	{
		result = FS_Sensor_Read(HTS221_ADDR, HTS221_REG_H0_RH_X2, buf, 2);
	}
	while (result != HAL_OK);

	H0_rH_x2 = buf[0];
	H1_rH_x2 = buf[1];

	do
	{
		result = FS_Sensor_Read(HTS221_ADDR, HTS221_REG_H0_T0_OUT, buf, 2);
	}
	while (result != HAL_OK);

	H0_T0_out = (int16_t) ((buf[1] << 8) | buf[0]);

	do
	{
		result = FS_Sensor_Read(HTS221_ADDR, HTS221_REG_H1_T0_OUT, buf, 2);
	}
	while (result != HAL_OK);

	H1_T0_out = (int16_t) ((buf[1] << 8) | buf[0]);

	// Read temperature calibration coefficients
	do
	{
		result = FS_Sensor_Read(HTS221_ADDR, HTS221_REG_T0_DEGC_X8, buf, 2);
	}
	while (result != HAL_OK);

	do
	{
		result = FS_Sensor_Read(HTS221_ADDR, HTS221_REG_T0_T1_MSB, &temp, 1);
	}
	while (result != HAL_OK);

	T0_degC_x8_u16 = (uint16_t) (((temp & 0x03) << 8) | buf[0]);
	T1_degC_x8_u16 = (uint16_t) (((temp & 0x0c) << 6) | buf[1]);

	do
	{
		result = FS_Sensor_Read(HTS221_ADDR, HTS221_REG_T0_OUT, buf, 4);
	}
	while (result != HAL_OK);

	T0_out = (int16_t) ((buf[1] << 8) | buf[0]);
	T1_out = (int16_t) ((buf[3] << 8) | buf[2]);

	// Throw out first temperature measurement
	do
	{
		result = FS_Sensor_Read(HTS221_ADDR, HTS221_REG_HUMIDITY_OUT_L, dataBuf, 4);
	}
	while (result != HAL_OK);

	return HAL_OK;
}

HAL_StatusTypeDef FS_HTS221_Start(void)
{
	const FS_Config_Data_t *config = FS_Config_Get();
	uint8_t buf[1];

	// Reset busy flag
	sensor_is_busy = false;

	// Enable EXTI pin
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_4);

	// Configure data ready on DRDY pin
	buf[0] = 0x04;
	if (FS_Sensor_Write(HTS221_ADDR, HTS221_REG_CTRL_REG3, buf, 1) != HAL_OK)
	{
		LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_4);
		return HAL_ERROR;
	}

	// Active mode; Output data rate 12.5 Hz; block data update
	buf[0] = 0x84 | config->hum_odr;
	if (FS_Sensor_Write(HTS221_ADDR, HTS221_REG_CTRL_REG1, buf, 1) != HAL_OK)
	{
		LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_4);
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef FS_HTS221_Stop(void)
{
	uint8_t buf[1];

	// Disable EXTI pin
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_4);

	// Disable data ready on DRDY pin
	buf[0] = 0x00;
	if (FS_Sensor_Write(HTS221_ADDR, HTS221_REG_CTRL_REG3, buf, 1) != HAL_OK)
	{
		return HAL_ERROR;
	}

	// Power down mode
	buf[0] = 0x00;
	if (FS_Sensor_Write(HTS221_ADDR, HTS221_REG_CTRL_REG1, buf, 1) != HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

static void FS_HTS221_Read_Callback(HAL_StatusTypeDef result)
{
	int16_t H_T_out, T_out;
	int32_t n, d;

	// Read raw measurements
	if (result == HAL_OK)
	{
		H_T_out = (int16_t) ((dataBuf[1] << 8) | dataBuf[0]);
		T_out = (int16_t) ((dataBuf[3] << 8) | dataBuf[2]);

		// Compute humidity
		n = (H1_rH_x2 - H0_rH_x2) * (H_T_out - H0_T0_out) * 10;
		d = H1_T0_out - H0_T0_out;

		humData->humidity = (n / d + H0_rH_x2 * 10) / 2;

		// Compute temperature
		n = (T1_degC_x8_u16 - T0_degC_x8_u16) * (T_out - T0_out) * 10;
		d = T1_out - T0_out;

		humData->temperature = (n / d + T0_degC_x8_u16 * 10) / 8;

		FS_Hum_DataReady_Callback();
	}
	else
	{
		FS_Log_WriteEventAsync("Error reading from humidity sensor");
	}

	// This measurement cycle is now complete, reset the busy flag.
	sensor_is_busy = false;
}

void FS_HTS221_Read(void)
{
	if (sensor_is_busy)
	{
		return;
	}

	sensor_is_busy = true;
	humData->time = HAL_GetTick();
	FS_Sensor_ReadAsync(HTS221_ADDR, HTS221_REG_HUMIDITY_OUT_L, dataBuf, 4, FS_HTS221_Read_Callback);
}
