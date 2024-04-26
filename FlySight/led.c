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

typedef enum
{
	LED_OFF,
	LED_ON
} LED_State_t;

static FS_LED_Colour_t savedColour = FS_LED_RED;
static LED_State_t savedState = LED_OFF;

extern TIM_HandleTypeDef htim1;

static void update(void)
{
	if (savedState == LED_ON)
	{
		if (savedColour == FS_LED_RED)
		{
			TIM1->CCR1 = 100;
			TIM1->CCR2 = 0;
		}
		else
		{
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 100;
		}
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	}
	else
	{
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	}
}

void FS_LED_On(void)
{
	savedState = LED_ON;
	update();
}

void FS_LED_Off(void)
{
	savedState = LED_OFF;
	update();
}

void FS_LED_SetColour(FS_LED_Colour_t colour)
{
	savedColour = colour;
	update();
}
