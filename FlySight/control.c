/*
 * control.c
 *
 *  Created on: Jul 27, 2020
 *      Author: Michael
 */

#include <stdbool.h>

#include "main.h"
#include "app_common.h"
#include "config.h"
#include "gnss.h"
#include "imu.h"
#include "led.h"
#include "log.h"

#define LED_BLINK_MSEC      900
#define LED_BLINK_TICKS     (LED_BLINK_MSEC*1000/CFG_TS_TICK_VAL)

static uint32_t sessionId = 0;

static uint8_t led_timer_id;

static void FS_Control_LED_Timer(void)
{
	// Turn on LED
	FS_LED_On();
}

void FS_Control_Init(void)
{
	// Turn on green LED
	FS_LED_SetColour(FS_LED_GREEN);
	FS_LED_On();

	// Initialize LED timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &led_timer_id, hw_ts_SingleShot, FS_Control_LED_Timer);

	if (FS_Config_Get()->enable_logging)
	{
		// Enable logging
		FS_Log_Init(sessionId);
	}
}

void FS_Control_DeInit(void)
{
	// Delete timer
	HW_TS_Delete(led_timer_id);

	// Turn off LEDs
	FS_LED_Off();

	if (FS_Config_Get()->enable_logging)
	{
		// Disable logging
		FS_Log_DeInit(sessionId);
	}

	// Increment session counter
	++sessionId;
}

void FS_GNSS_DataReady_Callback(void)
{
	const FS_GNSS_Data_t *data = FS_GNSS_GetData();

	if (FS_Config_Get()->enable_logging)
	{
		// Save to log file
		FS_Log_WriteGNSSData(data);
	}
}

void FS_GNSS_TimeReady_Callback(void)
{
	// Turn off LED
	FS_LED_Off();
	HW_TS_Start(led_timer_id, LED_BLINK_TICKS);

	if (FS_Config_Get()->enable_logging)
	{
		// Save to log file
		FS_Log_WriteGNSSTime(FS_GNSS_GetTime());
	}
}

void FS_IMU_DataReady_Callback(void)
{
	if (FS_Config_Get()->enable_logging)
	{
		// Save to log file
		FS_Log_WriteIMUData(FS_IMU_GetData());
	}
}
