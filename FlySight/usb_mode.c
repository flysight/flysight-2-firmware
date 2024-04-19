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
#include "resource_manager.h"
#include "state.h"
#include "usb_control.h"
#include "usb_device.h"
#include "usbd_core.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
extern UART_HandleTypeDef huart1;

void FS_USBMode_Init(void)
{
	/* Initialize microSD */
	FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS);

	// Read persistent state
	FS_State_Init();

	/* Initialize controller */
	FS_USBControl_Init();

	/* Algorithm to use USB on CPU1 comes from AN5289 Figure 9 */

	/* Configure peripheral clocks */
	PeriphClock_Config();

	/* Enable USB interface */
	MX_USB_Device_Init();
}

void FS_USBMode_DeInit(void)
{
	/* Disable controller */
	FS_USBControl_DeInit();

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

	/* De-initialize microSD */
	FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
}
