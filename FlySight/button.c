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
#include "app_ble.h"
#include "app_common.h"
#include "stm32_seq.h"
#include "button.h"
#include "mode.h"

#define CHECK_MSEC       5
#define PRESS_MSEC       10
#define RELEASE_MSEC     100
#define DEBOUNCE_TIMEOUT (CHECK_MSEC*1000/CFG_TS_TICK_VAL)

typedef enum
{
	FS_BUTTON_RELEASED = 0,
	FS_BUTTON_PRESSED
} FS_Button_State_t;

static FS_Button_State_t state;
static volatile uint8_t count;
static uint8_t timer_id;
static volatile bool busy;

static FS_Button_State_t FS_Button_RawState(void)
{
	if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin))
	{
		return FS_BUTTON_RELEASED;
	}
	else
	{
		return FS_BUTTON_PRESSED;
	}
}

static void FS_Button_Timer(void)
{
	// Get button state
	FS_Button_State_t raw_state = FS_Button_RawState();

	if (raw_state == state)
	{
		// Stop debounce timer
		HW_TS_Stop(timer_id);

		// Enable external interrupt
		busy = false;
	}
	else if (--count == 0)
	{
		state = raw_state;

		if (state == FS_BUTTON_RELEASED)
		{
			// Update mode
			FS_Mode_PushQueue(FS_MODE_EVENT_BUTTON_RELEASED);
		}
		else
		{
			// Update mode
			FS_Mode_PushQueue(FS_MODE_EVENT_BUTTON_PRESSED);

			// Start fast advertising
			APP_BLE_Adv_Set(APP_BLE_FAST_ADV);
		}

		// Stop debounce timer
		HW_TS_Stop(timer_id);

		// Enable external interrupt
		busy = false;
	}
}

void FS_Button_Triggered(void)
{
	if (busy) return;

	// Disable external interrupt
	busy = true;

	// Initialize debounce counter
	if (state == FS_BUTTON_RELEASED)
	{
		count = PRESS_MSEC / CHECK_MSEC;
	}
	else
	{
		count = RELEASE_MSEC / CHECK_MSEC;
	}

	// Start debounce timer
	HW_TS_Start(timer_id, DEBOUNCE_TIMEOUT);
}

void FS_Button_Init(void)
{
	// Enable EXTI pin
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_12);

	// Initialize button state
	state = FS_Button_RawState();

	// Create debounce timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_Repeated, FS_Button_Timer);
}
