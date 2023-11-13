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
#include "led.h"

#define UPDATE_MSEC    1000
#define UPDATE_TIMEOUT (UPDATE_MSEC*1000/CFG_TS_TICK_VAL)

typedef enum
{
	FS_CHARGE_ACTIVE = 0,
	FS_CHARGE_COMPLETE
} FS_Charge_State_t;

static uint8_t timer_id;

static FS_Charge_State_t FS_Charge_RawState(void)
{
	if (HAL_GPIO_ReadPin(CHG_STAT_GPIO_Port, CHG_STAT_Pin))
	{
		return FS_CHARGE_COMPLETE;
	}
	else
	{
		return FS_CHARGE_ACTIVE;
	}
}

static void FS_Charge_Timer(void)
{
	// Get charge state
	FS_Charge_State_t raw_state = FS_Charge_RawState();

	if (raw_state == FS_CHARGE_ACTIVE)
	{
		/* Turn on red LED */
		FS_LED_SetColour(FS_LED_RED);
	}
	else
	{
		/* Turn on green LED */
		FS_LED_SetColour(FS_LED_GREEN);
	}
}

void FS_Charge_Init(void)
{
	// Enable charging
	HAL_GPIO_WritePin(CHG_EN_LO_GPIO_Port, CHG_EN_LO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CHG_EN_HI_GPIO_Port, CHG_EN_HI_Pin, GPIO_PIN_SET);

	// Initialize LEDs
	FS_Charge_Timer();

	// Start update timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_Repeated, FS_Charge_Timer);
	HW_TS_Start(timer_id, UPDATE_TIMEOUT);
}

void FS_Charge_DeInit(void)
{
	// Stop update timer
	HW_TS_Stop(timer_id);
	HW_TS_Delete(timer_id);

	// Disable charging
	HAL_GPIO_WritePin(CHG_EN_LO_GPIO_Port, CHG_EN_LO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CHG_EN_HI_GPIO_Port, CHG_EN_HI_Pin, GPIO_PIN_SET);
}
