/*
 * hum.c
 *
 *  Created on: Jun 10, 2020
 *      Author: Michael
 */

#include "main.h"
#include "app_common.h"
#include "hum.h"
#include "sensor.h"

#define HUM_ADDR               0xbe
#define HUM_REG_WHO_AM_I       (0x0f | 0x80)
#define HUM_REG_CTRL_REG1      (0x20 | 0x80)
#define HUM_REG_CTRL_REG2      (0x21 | 0x80)
#define HUM_REG_CTRL_REG3      (0x22 | 0x80)
#define HUM_REG_HUMIDITY_OUT_L (0x28 | 0x80)
#define HUM_REG_H0_RH_X2       (0x30 | 0x80)
#define HUM_REG_T0_DEGC_X8     (0x32 | 0x80)
#define HUM_REG_T0_T1_MSB      (0x35 | 0x80)
#define HUM_REG_H0_T0_OUT      (0x36 | 0x80)
#define HUM_REG_H1_T0_OUT      (0x3a | 0x80)
#define HUM_REG_T0_OUT         (0x3c | 0x80)

static enum {
	HUM_ODR_OS   = 0,
	HUM_ODR_1    = 1,
	HUM_ODR_7    = 2,
	HUM_ODR_12_5 = 3
} humODR = HUM_ODR_12_5;

static FS_Hum_Data_t humData;

static uint8_t H0_rH_x2;
static uint8_t H1_rH_x2;

static int16_t H0_T0_out;
static int16_t H1_T0_out;

static uint16_t T0_degC_x8_u16;
static uint16_t T1_degC_x8_u16;

static int16_t T0_out;
static int16_t T1_out;

static uint8_t dataBuf[4];

void FS_Hum_Init(void)
{
	uint8_t buf[4];

	uint8_t temp;
	HAL_StatusTypeDef result;

	// Software reset
	do
	{
		buf[0] = 0x80;
		result = FS_Sensor_Write(HUM_ADDR, HUM_REG_CTRL_REG2, buf, 1);
	}
	while (result != HAL_OK);

	// Wait for reset
	do
	{
		result = FS_Sensor_Read(HUM_ADDR, HUM_REG_CTRL_REG2, buf, 1);
	}
	while ((result != HAL_OK) || (buf[0] & 0x80));

	// Check WHO_AM_I register value
	do
	{
		result = FS_Sensor_Read(HUM_ADDR, HUM_REG_WHO_AM_I, buf, 1);
	}
	while (result != HAL_OK);
	if (buf[0] != 0xbc)
		Error_Handler();

	// Read humidity calibration coefficients
	do
	{
		result = FS_Sensor_Read(HUM_ADDR, HUM_REG_H0_RH_X2, buf, 2);
	}
	while (result != HAL_OK);

	H0_rH_x2 = buf[0];
	H1_rH_x2 = buf[1];

	do
	{
		result = FS_Sensor_Read(HUM_ADDR, HUM_REG_H0_T0_OUT, buf, 2);
	}
	while (result != HAL_OK);

	H0_T0_out = (int16_t) ((buf[1] << 8) | buf[0]);

	do
	{
		result = FS_Sensor_Read(HUM_ADDR, HUM_REG_H1_T0_OUT, buf, 2);
	}
	while (result != HAL_OK);

	H1_T0_out = (int16_t) ((buf[1] << 8) | buf[0]);

	// Read temperature calibration coefficients
	do
	{
		result = FS_Sensor_Read(HUM_ADDR, HUM_REG_T0_DEGC_X8, buf, 2);
	}
	while (result != HAL_OK);

	do
	{
		result = FS_Sensor_Read(HUM_ADDR, HUM_REG_T0_T1_MSB, &temp, 1);
	}
	while (result != HAL_OK);

	T0_degC_x8_u16 = (uint16_t) (((temp & 0x03) << 8) | buf[0]);
	T1_degC_x8_u16 = (uint16_t) (((temp & 0x0c) << 6) | buf[1]);

	do
	{
		result = FS_Sensor_Read(HUM_ADDR, HUM_REG_T0_OUT, buf, 4);
	}
	while (result != HAL_OK);

	T0_out = (int16_t) ((buf[1] << 8) | buf[0]);
	T1_out = (int16_t) ((buf[3] << 8) | buf[2]);

	// Throw out first temperature measurement
	do
	{
		result = FS_Sensor_Read(HUM_ADDR, HUM_REG_HUMIDITY_OUT_L, dataBuf, 4);
	}
	while (result != HAL_OK);
}

void FS_Hum_Start(void)
{
	uint8_t buf[1];

	// Enable EXTI pin
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_4);

	// Configure data ready on DRDY pin
	buf[0] = 0x04;
	if (FS_Sensor_Write(HUM_ADDR, HUM_REG_CTRL_REG3, buf, 1) != HAL_OK)
		Error_Handler();

	// Active mode; Output data rate 12.5 Hz; block data update
	buf[0] = 0x84 | humODR;
	if (FS_Sensor_Write(HUM_ADDR, HUM_REG_CTRL_REG1, buf, 1) != HAL_OK)
		Error_Handler();
}

void FS_Hum_Stop(void)
{
	uint8_t buf[1];

	// Disable data ready on DRDY pin
	buf[0] = 0x00;
	if (FS_Sensor_Write(HUM_ADDR, HUM_REG_CTRL_REG3, buf, 1) != HAL_OK)
		Error_Handler();

	// Power down mode
	buf[0] = 0x00;
	if (FS_Sensor_Write(HUM_ADDR, HUM_REG_CTRL_REG1, buf, 1) != HAL_OK)
		Error_Handler();
}

static void FS_Hum_Read_Callback(HAL_StatusTypeDef result)
{
	int16_t H_T_out, T_out;
	int32_t n, d;

	// Read raw measurements
	if (result != HAL_OK)
		Error_Handler();

	H_T_out = (int16_t) ((dataBuf[1] << 8) | dataBuf[0]);
	T_out = (int16_t) ((dataBuf[3] << 8) | dataBuf[2]);

	// Compute humidity
	n = (H1_rH_x2 - H0_rH_x2) * (H_T_out - H0_T0_out) * 10;
	d = H1_T0_out - H0_T0_out;

	humData.humidity = (n / d + H0_rH_x2 * 10) / 2;

	// Compute temperature
	n = (T1_degC_x8_u16 - T0_degC_x8_u16) * (T_out - T0_out) * 10;
	d = T1_out - T0_out;

	humData.temperature = (n / d + T0_degC_x8_u16 * 10) / 8;

	FS_Hum_DataReady_Callback();
}

void FS_Hum_Read(void)
{
	humData.time = HAL_GetTick();

	FS_Sensor_ReadAsync(HUM_ADDR, HUM_REG_HUMIDITY_OUT_L, dataBuf, 4, FS_Hum_Read_Callback);
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
