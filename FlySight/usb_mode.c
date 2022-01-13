/*
 * usb_mode.c
 *
 *  Created on: Sep. 8, 2020
 *      Author: Michael
 */

#include "main.h"
#include "app_common.h"
#include "charge.h"
#include "gnss.h"
#include "usbd_core.h"
#include "usb_device.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;

void FS_USBMode_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Enable charge status */
	FS_Charge_Init();

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

	/* Enable USART */
	MX_USART1_UART_Init();

	/* Initialize GNSS */
	FS_GNSS_Init();

	/* Stop GNSS */
	FS_GNSS_Stop();

	/* Algorithm to use USB on CPU1 comes from AN5289 Figure 9 */

	/* Configure peripheral clocks */
	PeriphClock_Config();

	/* Enable USB interface */
	MX_USB_Device_Init();
}

void FS_USBMode_DeInit(void)
{
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

	/* Disable GNSS */
	FS_GNSS_DeInit();

	/* Disable USART */
	HAL_UART_DeInit(&huart1);

	/* Disable SPI */
	HAL_SPI_DeInit(&hspi2);

	/* Disable MMC_NCS pin */
	HAL_GPIO_DeInit(MMC_NCS_GPIO_Port, MMC_NCS_Pin);

	/* Disable VCC */
	HAL_GPIO_WritePin(VCC_EN_GPIO_Port, VCC_EN_Pin, GPIO_PIN_RESET);

	/* Reset GNSS_SAFEBOOT_N */
	HAL_GPIO_WritePin(GNSS_SAFEBOOT_N_GPIO_Port, GNSS_SAFEBOOT_N_Pin, GPIO_PIN_RESET);
}
