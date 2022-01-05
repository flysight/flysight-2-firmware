/*
 * baro.c
 *
 *  Created on: Jun 10, 2020
 *      Author: Michael
 */

#include "main.h"
#include "app_common.h"
#include "baro.h"
#include "sensor.h"

#define BARO_ADDR             0xba
#define BARO_REG_WHO_AM_I     0x0f
#define BARO_REG_CTRL_REG1    0x10
#define BARO_REG_CTRL_REG2    0x11
#define BARO_REG_CTRL_REG3    0x12
#define BARO_REG_PRESS_OUT_XL 0x28
#define BARO_REG_TEMP_OUT_L   0x2b

static enum {
	BARO_ODR_OS   = 0,
	BARO_ODR_1    = 1,
	BARO_ODR_10   = 2,
	BARO_ODR_25   = 3,
	BARO_ODR_50   = 4,
	BARO_ODR_75   = 5,
	BARO_ODR_100  = 6,
	BARO_ODR_200  = 7
} baroODR = BARO_ODR_25;

static uint8_t dataBuf[5];
static FS_Baro_Data_t baroData;

void FS_Baro_Init(void)
{
	uint8_t buf[1];
	HAL_StatusTypeDef result;

	// Software reset
	do
	{
		buf[0] = 0x04;
		result = FS_Sensor_Write(BARO_ADDR, BARO_REG_CTRL_REG2, buf, 1);
	}
	while (result != HAL_OK);

	// Wait for reset
	do
	{
		result = FS_Sensor_Read(BARO_ADDR, BARO_REG_CTRL_REG2, buf, 1);
	}
	while ((result != HAL_OK) || (buf[0] & 0x04));

	// Check WHO_AM_I register value
	do
	{
		result = FS_Sensor_Read(BARO_ADDR, BARO_REG_WHO_AM_I, buf, 1);
	}
	while (result != HAL_OK);
	if (buf[0] != 0xb3)
		Error_Handler();
}

void FS_Baro_Start(void)
{
	uint8_t buf[1];

	// Enable EXTI pin
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_13);

	// Output data rate 25 Hz; block data update
	buf[0] = (baroODR << 4) | 0x02;
	if (FS_Sensor_Write(BARO_ADDR, BARO_REG_CTRL_REG1, buf, 1) != HAL_OK)
		Error_Handler();

	// Configure data ready on INT_DRDY pin
	buf[0] = 0x04;
	if (FS_Sensor_Write(BARO_ADDR, BARO_REG_CTRL_REG3, buf, 1) != HAL_OK)
		Error_Handler();
}

void FS_Baro_Stop(void)
{
	uint8_t buf[1];

	// Software reset
	buf[0] = 0x04;
	if (FS_Sensor_Write(BARO_ADDR, BARO_REG_CTRL_REG2, buf, 1) != HAL_OK)
		Error_Handler();
}

static void FS_Baro_Read_Callback(HAL_StatusTypeDef result)
{
	uint32_t temp;

	if (result != HAL_OK)
		Error_Handler();

	temp = (uint32_t) ((dataBuf[2] << 16) | (dataBuf[1] << 8) | dataBuf[0]);
	if (temp & 0x00800000)
		temp |= 0xff000000;

	baroData.pressure = (((int32_t) temp) * (625 / 2)) / (256 / 2);
	baroData.temperature = (int16_t) ((dataBuf[4] << 8) | dataBuf[3]);

	FS_Baro_DataReady_Callback();
}

void FS_Baro_Read(void)
{
	baroData.time = HAL_GetTick();

	FS_Sensor_ReadAsync(BARO_ADDR, BARO_REG_PRESS_OUT_XL, dataBuf, 5, FS_Baro_Read_Callback);
}

const FS_Baro_Data_t *FS_Baro_GetData(void)
{
	return &baroData;
}

__weak void FS_Baro_DataReady_Callback(void)
{
  /* NOTE: This function should not be modified, when the callback is needed,
           the FS_Baro_DataReady_Callback could be implemented in the user file
   */
}
