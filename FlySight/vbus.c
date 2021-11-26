/*
 * vbus.c
 *
 *  Created on: Apr 18, 2020
 *      Author: Michael
 */

#include "main.h"
#include "app_common.h"
#include "mode.h"

void FS_VBUS_Triggered(void)
{
	// Check VBUS_DIV state
	if (HAL_GPIO_ReadPin(VBUS_DIV_GPIO_Port, VBUS_DIV_Pin))
	{
		// Update mode
		FS_Mode_PushQueue(FS_MODE_EVENT_VBUS_HIGH);
	}
	else
	{
		// Update mode
		FS_Mode_PushQueue(FS_MODE_EVENT_VBUS_LOW);
	}
}

void FS_VBUS_Init(void)
{
	// Enable EXTI pin
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_2);
}
