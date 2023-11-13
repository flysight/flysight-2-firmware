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
#include "charge.h"
#include "led.h"
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_storage_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;

static void FS_USBMode_BeginActivity(void)
{
	FS_LED_Off();
}


static void FS_USBMode_EndActivity(void)
{
	FS_LED_On();
}

void FS_USBMode_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Enable charge status */
	FS_Charge_Init();

	/* Turn on LEDs */
	FS_LED_On();

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

	/* Algorithm to use USB on CPU1 comes from AN5289 Figure 9 */

	/* Configure peripheral clocks */
	PeriphClock_Config();

	/* Enable USB interface */
	MX_USB_Device_Init();

	/* Set disk activity callbacks */
	USBD_SetActivityCallbacks(FS_USBMode_BeginActivity, FS_USBMode_EndActivity);
}

void FS_USBMode_DeInit(void)
{
	/* Clear disk activity callbacks */
	USBD_SetActivityCallbacks(0, 0);

	/* Turn off LEDs */
	FS_LED_Off();

	/* Disable charge status */
	FS_Charge_DeInit();

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

	/* Disable SPI */
	HAL_SPI_DeInit(&hspi2);

	/* Disable MMC_NCS pin */
	HAL_GPIO_DeInit(MMC_NCS_GPIO_Port, MMC_NCS_Pin);

	/* Disable VCC */
	HAL_GPIO_WritePin(VCC_EN_GPIO_Port, VCC_EN_Pin, GPIO_PIN_RESET);

	/* Reset GNSS_SAFEBOOT_N */
	HAL_GPIO_WritePin(GNSS_SAFEBOOT_N_GPIO_Port, GNSS_SAFEBOOT_N_Pin, GPIO_PIN_RESET);
}
