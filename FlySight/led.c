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

#include <math.h>

#include "main.h"
#include "app_common.h"
#include "led.h"

#define LED_BRIGHTNESS_MAX 100
#define LED_GAMMA          2.2

#define LED_UPDATE_MSEC    10
#define LED_UPDATE_RATE    (LED_UPDATE_MSEC*1000/CFG_TS_TICK_VAL)

typedef enum
{
	LED_OFF,
	LED_ON
} LED_State_t;

static FS_LED_Colour_t colour = FS_LED_RED;
static LED_State_t state = LED_OFF;

static volatile uint16_t brightness;
static int16_t brightness_step = 1;

static uint8_t timer_id;

extern TIM_HandleTypeDef htim1;

static void update(void)
{
	float norm_brightness;
	uint16_t adj_brightness;

	if (state == LED_ON)
	{
		norm_brightness = (float) brightness / LED_BRIGHTNESS_MAX;
		norm_brightness = pow(norm_brightness, LED_GAMMA);
		adj_brightness = norm_brightness * LED_BRIGHTNESS_MAX;

		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;

		if (colour & FS_LED_RED)
		{
			TIM1->CCR1 = adj_brightness;
		}
		if (colour & FS_LED_GREEN)
		{
			TIM1->CCR2 = adj_brightness;
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

static void FS_LED_Timer(void)
{
	if ((brightness + brightness_step > LED_BRIGHTNESS_MAX) ||
			(brightness + brightness_step < 0))
	{
		brightness_step = -brightness_step;
	}
	brightness += brightness_step;
	update();
}

void FS_LED_Init(void)
{
	// Initialize LED update timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_Repeated, FS_LED_Timer);
}

void FS_LED_DeInit(void)
{
	// Delete LED update timer
	HW_TS_Delete(timer_id);
}

void FS_LED_On(void)
{
	brightness = LED_BRIGHTNESS_MAX;
	state = LED_ON;
	update();
}

void FS_LED_Off(void)
{
	HW_TS_Stop(timer_id);

	brightness = 0;
	state = LED_OFF;
	update();
}

void FS_LED_Pulse(void)
{
	brightness = 0;
	state = LED_ON;
	update();

	HW_TS_Start(timer_id, LED_UPDATE_RATE);
}

void FS_LED_SetColour(FS_LED_Colour_t newColour)
{
	colour = newColour;
	update();
}
