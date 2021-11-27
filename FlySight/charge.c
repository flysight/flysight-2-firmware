/*
 * charge.c
 *
 *  Created on: Jun 7, 2020
 *      Author: Michael
 */

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
	FS_LED_On();
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

	/* Turn off both LEDs */
	FS_LED_Off();

	// Disable charging
	HAL_GPIO_WritePin(CHG_EN_LO_GPIO_Port, CHG_EN_LO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CHG_EN_HI_GPIO_Port, CHG_EN_HI_Pin, GPIO_PIN_SET);
}
