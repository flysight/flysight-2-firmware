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
#include "active_mode.h"
#include "app_ble.h"
#include "app_common.h"
#include "button.h"
#include "config_mode.h"
#include "custom_app.h"
#include "mode.h"
#include "pairing_mode.h"
#include "start_mode.h"
#include "state.h"
#include "stm32_seq.h"
#include "usb_mode.h"

#define QUEUE_LENGTH 4
#define HOLD_MSEC    1000
#define HOLD_TIMEOUT (HOLD_MSEC*1000/CFG_TS_TICK_VAL)

typedef FS_Mode_State_t FS_Mode_StateFunc_t(FS_Mode_Event_t event);

static FS_Mode_State_t FS_Mode_State_Sleep(FS_Mode_Event_t event);
static FS_Mode_State_t FS_Mode_State_Active(FS_Mode_Event_t event);
static FS_Mode_State_t FS_Mode_State_Config(FS_Mode_Event_t event);
static FS_Mode_State_t FS_Mode_State_USB(FS_Mode_Event_t event);
static FS_Mode_State_t FS_Mode_State_Pairing(FS_Mode_Event_t event);
static FS_Mode_State_t FS_Mode_State_Start(FS_Mode_Event_t event);

static FS_Mode_StateFunc_t *const mode_state_table[FS_MODE_STATE_COUNT] =
{
	FS_Mode_State_Sleep,
	FS_Mode_State_Active,
	FS_Mode_State_Config,
	FS_Mode_State_USB,
	FS_Mode_State_Pairing,
	FS_Mode_State_Start
};

static FS_Mode_State_t mode_state = FS_MODE_STATE_SLEEP;

static FS_Mode_Event_t event_queue[QUEUE_LENGTH];
static uint8_t queue_read = 0;
static uint8_t queue_write = 0;

static uint8_t timer_id;

typedef enum
{
	BUTTON_IDLE,
	BUTTON_FIRST_PRESS,
	BUTTON_RELEASED,
	BUTTON_SECOND_PRESS
} Button_State_t;

Button_State_t button_state;

void FS_Mode_PushQueue(FS_Mode_Event_t event)
{
	// TODO: Log if this queue overflows

	event_queue[queue_write] = event;
	queue_write = (queue_write + 1) % QUEUE_LENGTH;

	// Call update task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_MODE_UPDATE_ID, CFG_SCH_PRIO_1);
}

FS_Mode_Event_t FS_Mode_PopQueue(void)
{
	// TODO: Log if this queue underflows

	const FS_Mode_Event_t event = event_queue[queue_read];
	queue_read = (queue_read + 1) % QUEUE_LENGTH;
	return event;
}

bool FS_Mode_QueueEmpty(void)
{
	return queue_read == queue_write;
}

static FS_Mode_State_t FS_Mode_State_Sleep(FS_Mode_Event_t event)
{
	FS_Mode_State_t next_mode = FS_MODE_STATE_SLEEP;
	Button_State_t prev_state;

	if (event == FS_MODE_EVENT_BUTTON_PRESSED)
	{
		// Update button state
		if (button_state == BUTTON_IDLE)
		{
			button_state = BUTTON_FIRST_PRESS;
		}
		else if (button_state == BUTTON_RELEASED)
		{
			button_state = BUTTON_SECOND_PRESS;
		}

		// Start a timer
		HW_TS_Start(timer_id, HOLD_TIMEOUT);
	}
	else if (event == FS_MODE_EVENT_BUTTON_RELEASED)
	{
		// Save button state
		prev_state = button_state;

		// Update button state
		button_state = BUTTON_IDLE;

		// Update button state
		if (prev_state == BUTTON_FIRST_PRESS)
		{
			button_state = BUTTON_RELEASED;
		}
		else if (prev_state == BUTTON_SECOND_PRESS)
		{
			if (FS_State_Get()->enable_ble)
			{
				FS_PairingMode_Init();
				next_mode = FS_MODE_STATE_PAIRING;
			}
		}
	}
	else if (event == FS_MODE_EVENT_TIMER)
	{
		// Save button state
		prev_state = button_state;

		// Update button state
		button_state = BUTTON_IDLE;

		if (prev_state == BUTTON_FIRST_PRESS)
		{
			if (FS_State_Get()->active_mode == FS_ACTIVE_MODE_DEFAULT)
			{
				FS_ActiveMode_Init();
				next_mode = FS_MODE_STATE_ACTIVE;
			}
			else if (FS_State_Get()->active_mode == FS_ACTIVE_MODE_START)
			{
				FS_StartMode_Init();
				next_mode = FS_MODE_STATE_START;
			}
			else
			{
				Error_Handler();
			}
		}
		else if (prev_state == BUTTON_SECOND_PRESS)
		{
			FS_ConfigMode_Init();
			next_mode = FS_MODE_STATE_CONFIG;
		}
	}
	else if (event == FS_MODE_EVENT_VBUS_HIGH)
	{
		FS_USBMode_Init();
		next_mode = FS_MODE_STATE_USB;
	}

	return next_mode;
}

static FS_Mode_State_t FS_Mode_State_Active(FS_Mode_Event_t event)
{
	FS_Mode_State_t next_mode = FS_MODE_STATE_ACTIVE;

	if (event == FS_MODE_EVENT_BUTTON_PRESSED)
	{
		HW_TS_Start(timer_id, HOLD_TIMEOUT);
	}
	else if (event == FS_MODE_EVENT_BUTTON_RELEASED)
	{
		HW_TS_Stop(timer_id);
	}
	else if (event == FS_MODE_EVENT_TIMER)
	{
		FS_ActiveMode_DeInit();
		next_mode = FS_MODE_STATE_SLEEP;
	}

	return next_mode;
}

static FS_Mode_State_t FS_Mode_State_Config(FS_Mode_Event_t event)
{
	FS_Mode_State_t next_mode = FS_MODE_STATE_CONFIG;

	if (event == FS_MODE_EVENT_BUTTON_PRESSED)
	{
		FS_ConfigMode_DeInit();
		next_mode = FS_MODE_STATE_SLEEP;
	}
	else if (event == FS_MODE_EVENT_FORCE_UPDATE)
	{
		FS_ConfigMode_DeInit();
		next_mode = FS_MODE_STATE_SLEEP;
	}

	return next_mode;
}

static FS_Mode_State_t FS_Mode_State_USB(FS_Mode_Event_t event)
{
	FS_Mode_State_t next_mode = FS_MODE_STATE_USB;

	if (event == FS_MODE_EVENT_VBUS_LOW)
	{
		FS_USBMode_DeInit();
		next_mode = FS_MODE_STATE_SLEEP;
	}

	return next_mode;
}

static FS_Mode_State_t FS_Mode_State_Pairing(FS_Mode_Event_t event)
{
	FS_Mode_State_t next_mode = FS_MODE_STATE_PAIRING;

	if (event == FS_MODE_EVENT_BUTTON_PRESSED)
	{
		FS_PairingMode_DeInit();
		next_mode = FS_MODE_STATE_SLEEP;
	}
	else if (event == FS_MODE_EVENT_FORCE_UPDATE)
	{
		FS_PairingMode_DeInit();
		next_mode = FS_MODE_STATE_SLEEP;
	}

	return next_mode;
}

static FS_Mode_State_t FS_Mode_State_Start(FS_Mode_Event_t event)
{
	FS_Mode_State_t next_mode = FS_MODE_STATE_START;

	if (event == FS_MODE_EVENT_BUTTON_PRESSED)
	{
		HW_TS_Start(timer_id, HOLD_TIMEOUT);
	}
	else if (event == FS_MODE_EVENT_BUTTON_RELEASED)
	{
		HW_TS_Stop(timer_id);
	}
	else if (event == FS_MODE_EVENT_TIMER)
	{
		FS_StartMode_DeInit();
		next_mode = FS_MODE_STATE_SLEEP;
	}

	return next_mode;
}

static void FS_Mode_Update(void)
{
    while (!FS_Mode_QueueEmpty())
    {
        FS_Mode_State_t oldMode = mode_state;
        FS_Mode_Event_t event   = FS_Mode_PopQueue();

        /* The table call: next_mode = mode_state_table[current_mode](event) */
        FS_Mode_State_t nextMode = mode_state_table[oldMode](event);

        if (nextMode != oldMode)
        {
            mode_state = nextMode;

            /* Notify BLE about the updated mode (cast enum -> 1-byte) */
            Custom_Mode_Update((uint8_t) nextMode);
        }
    }
}

static void FS_Mode_Timer(void)
{
	FS_Mode_PushQueue(FS_MODE_EVENT_TIMER);
}

void FS_Mode_Init(void)
{
	if (HAL_GPIO_ReadPin(VBUS_DIV_GPIO_Port, VBUS_DIV_Pin))
	{
		// Update mode
		FS_Mode_PushQueue(FS_MODE_EVENT_VBUS_HIGH);
	}

	// Initialize button state
	button_state = BUTTON_IDLE;

	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_MODE_UPDATE_ID, UTIL_SEQ_RFU, FS_Mode_Update);
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_SingleShot, FS_Mode_Timer);
}

FS_Mode_State_t FS_Mode_State(void)
{
	return mode_state;
}
