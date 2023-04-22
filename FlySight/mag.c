/*
 * mag.c
 *
 *  Created on: Jun 10, 2020
 *      Author: Michael
 */

#include <stdbool.h>

#include "main.h"
#include "app_common.h"
#include "mag.h"
#include "sensor.h"

#define MAG_ADDR              0x3c
#define MAG_REG_WHO_AM_I      (0x4f | 0x80)
#define MAG_REG_CFG_REG_A     (0x60 | 0x80)
#define MAG_REG_CFG_REG_C     (0x62 | 0x80)
#define MAG_REG_OUTX_L_REG    (0x68 | 0x80)
#define MAG_TEMP_OUT_L_REG    (0x6e | 0x80)

static enum {
	MAG_ODR_10  = 0,
	MAG_ODR_20  = 1,
	MAG_ODR_50  = 2,
	MAG_ODR_100 = 3
} magODR = MAG_ODR_10;

static uint8_t dataBuf[8];
static bool magDataGood;
static FS_Mag_Data_t magData;

void FS_Mag_Init(void)
{
	uint8_t buf[1];
	HAL_StatusTypeDef result;

	// Software reset
	do
	{
		buf[0] = 0x20;
		result = FS_Sensor_Write(MAG_ADDR, MAG_REG_CFG_REG_A, buf, 1);
	}
	while (result != HAL_OK);

	// Wait for reset
	do
	{
		result = FS_Sensor_Read(MAG_ADDR, MAG_REG_CFG_REG_A, buf, 1);
	}
	while ((result != HAL_OK) || (buf[0] & 0x20));

	// Check WHO_AM_I register value
	do
	{
		result = FS_Sensor_Read(MAG_ADDR, MAG_REG_WHO_AM_I, buf, 1);
	}
	while (result != HAL_OK);
	if (buf[0] != 0x40)
		Error_Handler();

	// Throw out first temperature measurement
	do
	{
		result = FS_Sensor_Read(MAG_ADDR, MAG_REG_OUTX_L_REG, dataBuf, 8);
	}
	while (result != HAL_OK);
}

void FS_Mag_Start(void)
{
	uint8_t buf[1];

	// Enable EXTI pin
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);

	// Temperature compensation; output data rate 10 Hz; continuous mode
	buf[0] = (magODR << 2) | 0x80;
	if (FS_Sensor_Write(MAG_ADDR, MAG_REG_CFG_REG_A, buf, 1) != HAL_OK)
		Error_Handler();

	// Configure block data update and data ready on INT_DRDY pin
	buf[0] = 0x11;
	if (FS_Sensor_Write(MAG_ADDR, MAG_REG_CFG_REG_C, buf, 1) != HAL_OK)
		Error_Handler();
}

void FS_Mag_Stop(void)
{
	uint8_t buf[1];

	// Software reset
	buf[0] = 0x20;
	if (FS_Sensor_Write(MAG_ADDR, MAG_REG_CFG_REG_A, buf, 1) != HAL_OK)
		Error_Handler();
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
}

static void FS_Mag_Read_Callback_2(HAL_StatusTypeDef result)
{
	if ((result == HAL_OK) && magDataGood)
	{
		magData.temperature = (((int16_t) ((dataBuf[1] << 8) | dataBuf[0])) * (int32_t) 10) / 8 + 250;

		FS_Mag_DataReady_Callback();
	}
}

void FS_Mag_Read(void)
{
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
