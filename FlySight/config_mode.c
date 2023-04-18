/*
 * config_mode.c
 *
 *  Created on: Apr 17, 2023
 *      Author: Michael
 */

#include "main.h"
#include "app_fatfs.h"
#include "audio.h"
#include "led.h"

static FATFS fs;

extern SPI_HandleTypeDef hspi2;

void FS_ConfigMode_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Turn on green LED
	FS_LED_SetColour(FS_LED_GREEN);
	FS_LED_On();

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

	/* Initialize audio */
	FS_Audio_Init();

	/* Test audio */
	FS_Audio_Beep(1000, 1000, 1000, 0);
}

void FS_ConfigMode_DeInit(void)
{
	// Turn off LEDs
	FS_LED_Off();

	// Disable audio
	FS_Audio_DeInit();

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
}
