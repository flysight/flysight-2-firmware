/*
 * active_mode.c
 *
 *  Created on: Sep. 8, 2020
 *      Author: Michael
 */

#include "main.h"
#include "app_common.h"
#include "app_fatfs.h"
#include "baro.h"
#include "config.h"
#include "control.h"
#include "gnss.h"
#include "hum.h"
#include "imu.h"
#include "mag.h"
#include "sensor.h"
#include "usb_device.h"

static FATFS fs;

extern USBD_HandleTypeDef hUsbDeviceFS;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

void FS_ActiveMode_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Enable charging
	HAL_GPIO_WritePin(CHG_EN_LO_GPIO_Port, CHG_EN_LO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CHG_EN_HI_GPIO_Port, CHG_EN_HI_Pin, GPIO_PIN_SET);

	/* Set GNSS_SAFEBOOT_N */
	HAL_GPIO_WritePin(GNSS_SAFEBOOT_N_GPIO_Port, GNSS_SAFEBOOT_N_Pin, GPIO_PIN_SET);

	/* Enable VCC */
	HAL_GPIO_WritePin(VCC_EN_GPIO_Port, VCC_EN_Pin, GPIO_PIN_SET);

	/* Configure MMC_NCS pin */
	HAL_GPIO_WritePin(MMC_NCS_GPIO_Port, MMC_NCS_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = MMC_NCS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(MMC_NCS_GPIO_Port, &GPIO_InitStruct);

	/* Initialize FatFS */
	if (MX_FATFS_Init() != APP_OK)
	{
		Error_Handler();
	}

	/* Enable microSD card */
	if (f_mount(&fs, "0:/", 1) != FR_OK)
	{
		Error_Handler();
	}

	/* Initialize configuration */
	FS_Config_Init();
	if (FS_Config_Read("/config.txt") != FS_CONFIG_OK)
	{
		Error_Handler();
	}

	/* Initialize controller */
	FS_Control_Init();

	if (FS_Config_Get()->enable_vbat)
	{
		// Enable battery measurement
		HAL_GPIO_WritePin(VBAT_EN_GPIO_Port, VBAT_EN_Pin, GPIO_PIN_SET);
	}

	if (FS_Config_Get()->enable_vbat || FS_Config_Get()->enable_mic)
	{
		// Enable ADC
		MX_ADC1_Init();
	}

	if (FS_Config_Get()->enable_imu)
	{
		/* Start IMU */
		FS_IMU_Start();
	}

	/* Enable USART */
	MX_USART1_UART_Init();

	/* Initialize GNSS */
	FS_GNSS_Init(FS_GNSS_MODE_USB);

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

	/* Algorithm to use USB on CPU1 comes from AN5289 Figure 9 */

	/* Configure peripheral clocks */
	PeriphClock_Config();

	/* Enable USB interface */
	MX_USB_Device_Init();
}

void FS_ActiveMode_DeInit(void)
{
	/* Algorithm to use USB on CPU1 comes from AN5289 Figure 9 */

	/* Disable USB interface */
	if (USBD_DeInit(&hUsbDeviceFS) != USBD_OK)
	{
		Error_Handler();
	}

	/* Disable USB power */
	HAL_PWREx_DisableVddUSB();

	/* Get Sem0 */
	LL_HSEM_1StepLock(HSEM, CFG_HW_RNG_SEMID);

	/* Disable HSI48 */
	LL_RCC_HSI48_Disable();

	/* Release Sem0 */
	LL_HSEM_ReleaseLock(HSEM, CFG_HW_RNG_SEMID, 0);

	/* Release HSI48 semaphore */
	LL_HSEM_ReleaseLock(HSEM, CFG_HW_CLK48_CONFIG_SEMID, 0);

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

	if (FS_Config_Get()->enable_vbat || FS_Config_Get()->enable_mic)
	{
		// Disable ADC
		if (HAL_ADC_DeInit(&hadc1) != HAL_OK)
		{
			Error_Handler();
		}
	}

	if (FS_Config_Get()->enable_vbat)
	{
		// Disable battery measurement
		HAL_GPIO_WritePin(VBAT_EN_GPIO_Port, VBAT_EN_Pin, GPIO_PIN_RESET);
	}

	/* Disable controller */
	FS_Control_DeInit();

	/* Disable microSD card */
	if (f_mount(0, "0:/", 0) != FR_OK)
	{
		Error_Handler();
	}

	/* Disable FatFS */
	if (MX_FATFS_DeInit() != APP_OK)
	{
		Error_Handler();
	}

	/* Disable SPI */
	HAL_SPI_DeInit(&hspi2);

	/* Disable MMC_NCS pin */
	HAL_GPIO_DeInit(MMC_NCS_GPIO_Port, MMC_NCS_Pin);

	/* Disable VCC */
	HAL_GPIO_WritePin(VCC_EN_GPIO_Port, VCC_EN_Pin, GPIO_PIN_RESET);

	/* Reset GNSS_SAFEBOOT_N */
	HAL_GPIO_WritePin(GNSS_SAFEBOOT_N_GPIO_Port, GNSS_SAFEBOOT_N_Pin, GPIO_PIN_RESET);

	// Disable charging
	HAL_GPIO_WritePin(CHG_EN_LO_GPIO_Port, CHG_EN_LO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CHG_EN_HI_GPIO_Port, CHG_EN_HI_Pin, GPIO_PIN_SET);
}
