/*
 * led.c
 *
 *  Created on: Aug. 8, 2020
 *      Author: Michael
 */

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

static void update(void)
{
	if (savedState == LED_ON)
	{
		if (savedColour == FS_LED_RED)
		{
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
		}
	}
	else
	{
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
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
