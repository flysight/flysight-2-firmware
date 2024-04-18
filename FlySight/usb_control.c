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

#include <stdbool.h>

#include "main.h"
#include "app_common.h"
#include "charge.h"
#include "led.h"
#include "state.h"
#include "usbd_storage_if.h"

#define UPDATE_MSEC    1000
#define UPDATE_TIMEOUT (UPDATE_MSEC*1000/CFG_TS_TICK_VAL)

static uint8_t timer_id;

static void FS_USBControl_Timer(void)
{
	if (FS_Charge_GetState() == FS_CHARGE_ACTIVE)
	{
		// Turn on red LED
		FS_LED_SetColour(FS_LED_RED);
	}
	else
	{
		// Turn on green LED
		FS_LED_SetColour(FS_LED_GREEN);
	}
}

static void FS_USBControl_BeginActivity(void)
{
	FS_LED_Off();
}

static void FS_USBControl_EndActivity(void)
{
	FS_LED_On();
}

void FS_USBControl_Init(void)
{
	// Initialize LEDs
	FS_USBControl_Timer();
	FS_LED_On();

	// Enable charging
	FS_Charge_Set(FS_State_Get()->charge_current);

	// Start update timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_Repeated, FS_USBControl_Timer);
	HW_TS_Start(timer_id, UPDATE_TIMEOUT);

	// Set disk activity callbacks
	USBD_SetActivityCallbacks(FS_USBControl_BeginActivity, FS_USBControl_EndActivity);
}

void FS_USBControl_DeInit(void)
{
	// Clear disk activity callbacks
	USBD_SetActivityCallbacks(0, 0);

	// Delete timer
	HW_TS_Delete(timer_id);

	// Disable charging
	FS_Charge_Set(FS_CHARGE_DISABLE);

	// Turn off LEDs
	FS_LED_Off();
}
