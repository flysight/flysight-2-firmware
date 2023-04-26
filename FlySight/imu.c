/*
 * imu.c
 *
 *  Created on: Jun. 9, 2020
 *      Author: Michael
 */

#include <stdbool.h>

#include "main.h"
#include "app_common.h"
#include "imu.h"
#include "stm32_seq.h"

#define TIMEOUT 100

#define CS_HIGH()	{ HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET); }
#define CS_LOW()	{ HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET); }

#define LSM6DSO_REG_FUNC_CFG_ACCESS 0x01
#define LSM6DSO_REG_INT1_CTRL       0x0d
#define LSM6DSO_REG_WHO_AM_I        0x0f
#define LSM6DSO_REG_CTRL1_XL        0x10
#define LSM6DSO_REG_CTRL2_G         0x11
#define LSM6DSO_REG_CTRL3_C         0x12
#define LSM6DSO_REG_CTRL4_C         0x13
#define LSM6DSO_REG_CTRL9_XL        0x18
#define LSM6DSO_REG_CTRL10_C        0x19
#define LSM6DSO_OUT_TEMP_L_REG      0x20

static enum {
	ACCEL_ODR_PD   = 0,
	ACCEL_ODR_12_5 = 0x1,
	ACCEL_ODR_26   = 0x2,
	ACCEL_ODR_52   = 0x3,
	ACCEL_ODR_104  = 0x4,
	ACCEL_ODR_208  = 0x5,
	ACCEL_ODR_416  = 0x6,
	ACCEL_ODR_833  = 0x7,
	ACCEL_ODR_1666 = 0x8,
	ACCEL_ODR_3333 = 0x9,
	ACCEL_ODR_6666 = 0xa,
	ACCEL_ODR_1_6  = 0xb
} accelODR = ACCEL_ODR_104;

static enum {
	ACCEL_FS_2  = 0,
	ACCEL_FS_16 = 1,
	ACCEL_FS_4  = 2,
	ACCEL_FS_8  = 3
} accelFS = ACCEL_FS_8;

static enum {
	GYRO_ODR_PD = 0,
	GYRO_ODR_12_5 = 0x1,
	GYRO_ODR_26   = 0x2,
	GYRO_ODR_52   = 0x3,
	GYRO_ODR_104  = 0x4,
	GYRO_ODR_208  = 0x5,
	GYRO_ODR_416  = 0x6,
	GYRO_ODR_833  = 0x7,
	GYRO_ODR_1666 = 0x8,
	GYRO_ODR_3333 = 0x9,
	GYRO_ODR_6666 = 0xa
} gyroODR = GYRO_ODR_104;

static enum {
	GYRO_FS_250  = 0,
	GYRO_FS_500  = 1,
	GYRO_FS_1000 = 2,
	GYRO_FS_2000 = 3
} gyroFS = GYRO_FS_2000;

static int32_t accelFactor;
static int32_t gyroFactor;

static uint8_t dataBuf[15];
static FS_IMU_Data_t imuData;

static volatile bool handleRead  = false;
static volatile bool dataWaiting = false;
static volatile bool busy = false;

extern SPI_HandleTypeDef hspi1;

static void FS_IMU_BeginRead(void);
static void FS_IMU_Read_Callback(HAL_StatusTypeDef result);

void FS_IMU_TransferComplete(void)
{
	FS_IMU_Read_Callback(HAL_OK);
}

void FS_IMU_TransferError(void)
{
	FS_IMU_Read_Callback(HAL_ERROR);
}

static HAL_StatusTypeDef FS_IMU_ReadRegister(uint8_t addr, uint8_t *data, uint16_t size)
{
	HAL_StatusTypeDef result;
	uint32_t ms;

	/* Set read flag */
	addr |= 0x80;

	ms = HAL_GetTick();
	do
	{
		if (HAL_GetTick() - ms > TIMEOUT)
		{
			Error_Handler();
		}

		CS_LOW();
		result = HAL_SPI_Transmit(&hspi1, &addr, 1, TIMEOUT);
		if (result == HAL_OK)
		{
			result = HAL_SPI_Receive(&hspi1, data, size, TIMEOUT);
		}
		CS_HIGH();
	}
	while (result != HAL_OK);

    return HAL_OK;
}

static HAL_StatusTypeDef FS_IMU_WriteRegister(uint8_t addr, uint8_t *data, uint16_t size)
{
	HAL_StatusTypeDef result;
	uint32_t ms;

	ms = HAL_GetTick();
	do
	{
		if (HAL_GetTick() - ms > TIMEOUT)
		{
			Error_Handler();
		}

		CS_LOW();
		result = HAL_SPI_Transmit(&hspi1, &addr, 1, TIMEOUT);
		if (result == HAL_OK)
		{
			result = HAL_SPI_Transmit(&hspi1, data, size, TIMEOUT);
		}
		CS_HIGH();
	}
	while (result != HAL_OK);

	return HAL_OK;
}

void FS_IMU_Init(void)
{
	uint8_t buf[1];
	HAL_StatusTypeDef result;

	// Software reset
	do
	{
		buf[0] = 0x01;
		result = FS_IMU_WriteRegister(LSM6DSO_REG_CTRL3_C, buf, 1);
	}
	while (result != HAL_OK);

	// Wait for reset
	do
	{
		result = FS_IMU_ReadRegister(LSM6DSO_REG_CTRL3_C, buf, 1);
	}
	while ((result != HAL_OK) || (buf[0] & 0x01));

	// Check WHO_AM_I register value
	do
	{
		result = FS_IMU_ReadRegister(LSM6DSO_REG_WHO_AM_I, buf, 1);
	}
	while (result != HAL_OK);
	if (buf[0] != 0x6c)
		Error_Handler();
}

void FS_IMU_Start(void)
{
	uint8_t buf[1];
	uint32_t primask_bit;

	// Enable EXTI pin
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_9);

	// Accelerometer Data Ready interrupt on INT1
	buf[0] = 0x01;
	FS_IMU_WriteRegister(LSM6DSO_REG_INT1_CTRL, buf, 1);

	// Set accelerometer ODR and FS
	buf[0] = (accelODR << 4) | (accelFS << 2);
	FS_IMU_WriteRegister(LSM6DSO_REG_CTRL1_XL, buf, 1);

	// Set gyro ODR and FS
	buf[0] = (gyroODR << 4) | (gyroFS << 2);
	FS_IMU_WriteRegister(LSM6DSO_REG_CTRL2_G, buf, 1);

	// Set BDU and push-pull on INT1
	buf[0] = 0x44;
	FS_IMU_WriteRegister(LSM6DSO_REG_CTRL3_C, buf, 1);

	// Disable I2C
	buf[0] = 0x04;
	FS_IMU_WriteRegister(LSM6DSO_REG_CTRL4_C, buf, 1);

	switch (accelFS)
	{
	case ACCEL_FS_2:  accelFactor = 2 * 100000; break;
	case ACCEL_FS_4:  accelFactor = 4 * 100000; break;
	case ACCEL_FS_8:  accelFactor = 8 * 100000; break;
	case ACCEL_FS_16: accelFactor = 16 * 100000; break;
	}

	switch (gyroFS)
	{
	case GYRO_FS_250:  gyroFactor = 250 * 1000; break;
	case GYRO_FS_500:  gyroFactor = 500 * 1000; break;
	case GYRO_FS_1000: gyroFactor = 1000 * 1000; break;
	case GYRO_FS_2000: gyroFactor = 2000 * 1000; break;
	}

	// Enable asyncrhonous reads
	primask_bit = __get_PRIMASK();
	__disable_irq();

	handleRead = true;
	bool read = dataWaiting;

	__set_PRIMASK(primask_bit);

	if (read)
	{
		FS_IMU_BeginRead();
	}
}

void FS_IMU_Stop(void)
{
	uint8_t buf[1];
	uint32_t primask_bit;

	// Disable asynchronous reads
	primask_bit = __get_PRIMASK();
	__disable_irq();

	handleRead = false;
	dataWaiting = false;

	__set_PRIMASK(primask_bit);

	while (busy);

	// Software reset
	buf[0] = 0x01;
	if (FS_IMU_WriteRegister(LSM6DSO_REG_CTRL3_C, buf, 1) != HAL_OK)
		Error_Handler();
}

void FS_IMU_Read(void)
{
	imuData.time = HAL_GetTick();

	if (handleRead)
	{
		FS_IMU_BeginRead();
	}
	else
	{
		dataWaiting = true;
	}
}

static void FS_IMU_BeginRead(void)
{
	HAL_StatusTypeDef res;

	/* Address with read flag */
	dataBuf[0] = LSM6DSO_OUT_TEMP_L_REG | 0x80;

	if (busy)
		Error_Handler();

	busy = true;

	CS_LOW();
	res = HAL_SPI_TransmitReceive_DMA(&hspi1, dataBuf, dataBuf, 15);

	if (res != HAL_OK)
		FS_IMU_Read_Callback(res);
}

static void FS_IMU_Read_Callback(HAL_StatusTypeDef result)
{
	CS_HIGH();

	busy = false;

	if (result == HAL_OK)
	{
		imuData.temperature = (((int16_t) ((dataBuf[2] << 8) | dataBuf[1])) * 100) / 256 + 2500;

		imuData.wy = (((int16_t) ((dataBuf[4] << 8) | dataBuf[3])) * (gyroFactor / 8)) / (32768 / 8);
		imuData.wx = -(((int16_t) ((dataBuf[6] << 8) | dataBuf[5])) * (gyroFactor / 8)) / (32768 / 8);
		imuData.wz = (((int16_t) ((dataBuf[8] << 8) | dataBuf[7])) * (gyroFactor / 8)) / (32768 / 8);

		imuData.ay = (((int16_t) ((dataBuf[10] << 8) | dataBuf[9])) * (accelFactor / 8)) / (32768 / 8);
		imuData.ax = -(((int16_t) ((dataBuf[12] << 8) | dataBuf[11])) * (accelFactor / 8)) / (32768 / 8);
		imuData.az = (((int16_t) ((dataBuf[14] << 8) | dataBuf[13])) * (accelFactor / 8)) / (32768 / 8);

		FS_IMU_DataReady_Callback();
	}
}

const FS_IMU_Data_t *FS_IMU_GetData(void)
{
	return &imuData;
}

__weak void FS_IMU_DataReady_Callback(void)
{
	  /* NOTE: This function should not be modified, when the callback is needed,
	           the FS_IMU_DataReady_Callback could be implemented in the user file
	   */
}
