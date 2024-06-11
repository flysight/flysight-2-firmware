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
#include "audio.h"
#include "charge.h"
#include "config.h"
#include "custom_app.h"
#include "gnss.h"
#include "led.h"
#include "log.h"
#include "state.h"
#include "stm32_seq.h"

#define LED_BLINK_MSEC      900
#define LED_BLINK_TICKS     (LED_BLINK_MSEC*1000/CFG_TS_TICK_VAL)

#define COUNT_MSEC          1000
#define COUNT_TICKS         (COUNT_MSEC*1000/CFG_TS_TICK_VAL)

#define TONE_MAX_PITCH 1760

typedef enum
{
	FS_START_COMMAND_START  = 0x00,
	FS_START_COMMAND_CANCEL = 0x01
} FS_Start_Command_t;

static uint8_t led_timer_id;
static uint8_t count_timer_id;

static volatile bool hasFix = false;

static volatile enum {
	FS_CONTROL_INACTIVE = 0,
	FS_CONTROL_IDLE,
	FS_CONTROL_COUNTING,
	FS_CONTROL_UPDATE
} state = FS_CONTROL_INACTIVE;

static uint8_t countdown;
static FS_GNSS_Int_t gnss_int;

void FS_StartControl_DataReady_Callback(void);
void FS_StartControl_TimeReady_Callback(bool validTime);
void FS_StartControl_IntReady_Callback(void);

void FS_StartControl_Update(void);
static void FS_StartControl_Count_Timer(void);

static void FS_StartControl_LED_Timer(void)
{
	// Turn on LED
	FS_LED_On();
}

void FS_StartControl_RegisterTasks(void)
{
	// Register update task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_START_UPDATE_ID, UTIL_SEQ_RFU, FS_StartControl_Update);
}

void FS_StartControl_Init(void)
{
	// Set callback functions
	FS_GNSS_DataReady_SetCallback(FS_StartControl_DataReady_Callback);
	FS_GNSS_TimeReady_SetCallback(FS_StartControl_TimeReady_Callback);
	FS_GNSS_IntReady_SetCallback(FS_StartControl_IntReady_Callback);

	// Initialize LEDs
	FS_LED_SetColour(FS_LED_ORANGE);
	FS_LED_On();

	// Enable charging
	FS_Charge_SetCurrent(FS_State_Get()->charge_current);

	// Initialize timers
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &led_timer_id, hw_ts_SingleShot, FS_StartControl_LED_Timer);
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &count_timer_id, hw_ts_Repeated, FS_StartControl_Count_Timer);

	// Initialize state
	hasFix = false;
	state = FS_CONTROL_IDLE;
}

void FS_StartControl_DeInit(void)
{
	// Update state
	state = FS_CONTROL_INACTIVE;

	// Delete timer
	HW_TS_Delete(led_timer_id);
	HW_TS_Delete(count_timer_id);

	// Disable charging
	FS_Charge_SetCurrent(FS_CHARGE_DISABLE);

	// Turn off LEDs
	FS_LED_Off();

	// Clear callback functions
	FS_GNSS_DataReady_SetCallback(NULL);
	FS_GNSS_TimeReady_SetCallback(NULL);
	FS_GNSS_IntReady_SetCallback(NULL);

	// Clear EXTINT
	HAL_GPIO_WritePin(GNSS_EXTINT_GPIO_Port, GNSS_EXTINT_Pin, GPIO_PIN_RESET);
}

void FS_StartControl_DataReady_Callback(void)
{
	const FS_GNSS_Data_t *data = FS_GNSS_GetData();

	if (state == FS_CONTROL_INACTIVE) return;

	if (FS_Config_Get()->enable_logging)
	{
		// Update log path
		FS_Log_UpdatePath(data);
	}

	// Update BLE characteristic
	Custom_GNSS_Update(data);

	hasFix = (data->gpsFix == 3);
}

void FS_StartControl_TimeReady_Callback(bool validTime)
{
	if (state == FS_CONTROL_INACTIVE) return;

	if (hasFix)
	{
		// Turn off LED
		FS_LED_Off();
		HW_TS_Start(led_timer_id, LED_BLINK_TICKS);
	}
}

void FS_StartControl_IntReady_Callback(void)
{
	// Clear EXTINT
	HAL_GPIO_WritePin(GNSS_EXTINT_GPIO_Port, GNSS_EXTINT_Pin, GPIO_PIN_RESET);

	if (state == FS_CONTROL_COUNTING)
	{
		// Copy interrupt data locally
		memcpy(&gnss_int, FS_GNSS_GetInt(), sizeof(FS_GNSS_Int_t));

		// Update state
		state = FS_CONTROL_UPDATE;
		UTIL_SEQ_SetTask(1<<CFG_TASK_FS_START_UPDATE_ID, CFG_SCH_PRIO_1);
	}
}

static void FS_StartControl_Count_Timer(void)
{
	const FS_Config_Data_t *config = FS_Config_Get();
	char filename[13];

	if (state != FS_CONTROL_COUNTING) return;

	if (--countdown == 0)
	{
		// Set EXTINT
		HAL_GPIO_WritePin(GNSS_EXTINT_GPIO_Port, GNSS_EXTINT_Pin, GPIO_PIN_SET);

		// Play tone
		FS_Audio_Beep(TONE_MAX_PITCH, TONE_MAX_PITCH, 500, config->volume * 5);

		// Stop countdown timer
		HW_TS_Stop(count_timer_id);
	}
	else
	{
		// Play countdown value
		filename[0] = '0' + countdown;
		filename[1] = '.';
		filename[2] = 'w';
		filename[3] = 'a';
		filename[4] = 'v';
		filename[5] = '\0';

		FS_Audio_Play(filename, config->sp_volume * 5);
	}
}

void FS_StartControl_Update(void)
{
	Custom_Start_Packet_t *packet;

	if (state == FS_CONTROL_INACTIVE) return;

	if (state == FS_CONTROL_UPDATE)
	{
		Custom_Start_Update(&gnss_int);
		state = FS_CONTROL_IDLE;
	}

	while ((packet = Custom_Start_GetNextControlPacket()))
	{
		if (packet->length > 0)
		{
			// Handle commands
			switch (packet->data[0])
			{
			case FS_START_COMMAND_START:
				if (state == FS_CONTROL_IDLE)
				{
					countdown = 6;
					state = FS_CONTROL_COUNTING;
					HW_TS_Start(count_timer_id, COUNT_TICKS);
				}
				break;
			case FS_START_COMMAND_CANCEL:
				if (state == FS_CONTROL_COUNTING)
				{
					HW_TS_Stop(count_timer_id);
					state = FS_CONTROL_IDLE;
				}
				break;
			}
		}
	}
}
