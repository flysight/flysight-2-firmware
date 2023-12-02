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
#include "resource_manager.h"

typedef struct {
	void (*Init)(void);
	void (*DeInit)(void);
} FS_Resource_Operations;

static void VCC_Init(void);
static void VCC_DeInit(void);

static void MicroSD_Init(void);
static void MicroSD_DeInit(void);

static void FatFS_Init(void);
static void FatFS_DeInit(void);

FS_Resource_Operations resource_operations[FS_RESOURCE_COUNT] =
{
	{VCC_Init, VCC_DeInit},
	{MicroSD_Init, MicroSD_DeInit},
	{FatFS_Init, FatFS_DeInit}
};

static uint8_t resource_counts[FS_RESOURCE_COUNT];

static FATFS fs;

extern SPI_HandleTypeDef hspi2;

static void VCC_Init(void)
{
	/* Set GNSS_SAFEBOOT_N */
	HAL_GPIO_WritePin(GNSS_SAFEBOOT_N_GPIO_Port, GNSS_SAFEBOOT_N_Pin, GPIO_PIN_SET);

	/* Enable VCC */
	HAL_GPIO_WritePin(VCC_EN_GPIO_Port, VCC_EN_Pin, GPIO_PIN_SET);
}

static void VCC_DeInit(void)
{
	/* Disable VCC */
	HAL_GPIO_WritePin(VCC_EN_GPIO_Port, VCC_EN_Pin, GPIO_PIN_RESET);

	/* Reset GNSS_SAFEBOOT_N */
	HAL_GPIO_WritePin(GNSS_SAFEBOOT_N_GPIO_Port, GNSS_SAFEBOOT_N_Pin, GPIO_PIN_RESET);
}

static void MicroSD_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Initialize VCC */
	FS_ResourceManager_RequestResource(FS_RESOURCE_VCC);

	/* Configure MMC_NCS pin */
	HAL_GPIO_WritePin(MMC_NCS_GPIO_Port, MMC_NCS_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = MMC_NCS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(MMC_NCS_GPIO_Port, &GPIO_InitStruct);
}

static void MicroSD_DeInit(void)
{
	/* Disable SPI */
	HAL_SPI_DeInit(&hspi2);

	/* Disable MMC_NCS pin */
	HAL_GPIO_DeInit(MMC_NCS_GPIO_Port, MMC_NCS_Pin);

	/* De-initialize VCC */
	FS_ResourceManager_ReleaseResource(FS_RESOURCE_VCC);
}

static void FatFS_Init(void)
{
	/* Initialize microSD */
	FS_ResourceManager_RequestResource(FS_RESOURCE_MICROSD);

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
}

static void FatFS_DeInit(void)
{
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

	/* De-initialize microSD */
	FS_ResourceManager_ReleaseResource(FS_RESOURCE_MICROSD);
}

void FS_ResourceManager_Init(void)
{
	uint8_t i;

	for (i = 0; i < FS_RESOURCE_COUNT; ++i)
	{
		resource_counts[i] = 0;
	}
}

void FS_ResourceManager_RequestResource(FS_Resource_t resource)
{
	if (resource < FS_RESOURCE_COUNT)
	{
		if (resource_counts[resource] == 0)
		{
			resource_operations[resource].Init();
		}

		++resource_counts[resource];
	}
}

void FS_ResourceManager_ReleaseResource(FS_Resource_t resource)
{
	if ((resource < FS_RESOURCE_COUNT) && (resource_counts[resource] > 0))
	{
		--resource_counts[resource];

		if (resource_counts[resource] == 0)
		{
			resource_operations[resource].DeInit();
		}
	}
}
