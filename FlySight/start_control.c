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
#include <string.h>

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
#include "time.h"
#include "custom_stm.h"
#include "ble_tx_queue.h"
#include "control_point_protocol.h"

#define LED_BLINK_MSEC      900
#define LED_BLINK_TICKS     (LED_BLINK_MSEC*1000/CFG_TS_TICK_VAL)

#define COUNT_MSEC          1000
#define COUNT_TICKS         (COUNT_MSEC*1000/CFG_TS_TICK_VAL)

#define TONE_MAX_PITCH 1760

// Starter Pistol (SP) Control Point command opcodes
#define SP_CMD_START_COUNTDOWN  0x01 // Payload: (none)
#define SP_CMD_CANCEL_COUNTDOWN 0x02 // Payload: (none)

static uint8_t led_timer_id;
static uint8_t count_timer_id;

static volatile bool hasFix;

static volatile enum {
	FS_CONTROL_INACTIVE = 0,
	FS_CONTROL_IDLE,
	FS_CONTROL_COUNTING,
	FS_CONTROL_UPDATE
} state = FS_CONTROL_INACTIVE;

static uint8_t countdown;
static FS_GNSS_Int_t gnss_int;

extern uint8_t SizeSp_Control_Point;

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

	// Delete timers
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
	uint32_t epoch, timestamp;

	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint16_t ms;

	if (state == FS_CONTROL_INACTIVE) return;

	if (state == FS_CONTROL_UPDATE)
	{
		// Start of the year 2000 (gmtime epoch)
		epoch = 1042 * 7 * 24 * 3600 + 518400;

		// Calculate timestamp at start_time
		timestamp = gnss_int.week * 7 * 24 * 3600 - epoch;
		timestamp += gnss_int.towMS / 1000;

		// Calculate millisecond part of date/time
		ms = gnss_int.towMS % 1000;

		// Convert back to date/time
		gmtime_r(timestamp, &year, &month, &day, &hour, &min, &sec);

		// Update BLE characteristics
		Custom_Start_Update(year, month, day, hour, min, sec, ms);

		// Update event log
		FS_Log_WriteEvent("Start tone at %04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
				year, month, day, hour, min, sec, ms);

		state = FS_CONTROL_IDLE;
	}
}

static uint8_t FS_StartControl_InitiateCommand(uint8_t cmd_opcode)
{
	// Handle commands
	switch (cmd_opcode)
	{
	case SP_CMD_START_COUNTDOWN:
		if (state == FS_CONTROL_IDLE)
		{
			countdown = 6;
			state = FS_CONTROL_COUNTING;
			HW_TS_Start(count_timer_id, COUNT_TICKS);
			return CP_STATUS_SUCCESS;
		}
        else if (state == FS_CONTROL_COUNTING)
        {
            return CP_STATUS_BUSY;
        }
        else
        {
            return CP_STATUS_OPERATION_NOT_PERMITTED;
        }
	case SP_CMD_CANCEL_COUNTDOWN:
		if (state == FS_CONTROL_COUNTING)
		{
			HW_TS_Stop(count_timer_id);
			state = FS_CONTROL_IDLE;
			return CP_STATUS_SUCCESS;
		}
        else if (state == FS_CONTROL_IDLE)
        {
            return CP_STATUS_SUCCESS; // No active countdown to cancel
        }
        else
        {
            return CP_STATUS_OPERATION_NOT_PERMITTED;
        }
    default:
        return CP_STATUS_CMD_NOT_SUPPORTED;
	}
}

void FS_StartControl_Handle_SP_ControlPointWrite(
		const uint8_t *payload, uint8_t length,
		uint16_t conn_handle, uint8_t indication_enabled_flag)
{
    (void)conn_handle; // Mark as unused

    uint8_t received_cmd_opcode = 0xFF;
    uint8_t status = CP_STATUS_ERROR_UNKNOWN;
    // SP commands do not return optional data in the ACK, so no response_data_buf needed here.

    if (length < 1)
    {
        status = CP_STATUS_INVALID_PARAMETER;
    }
    else
    {
        received_cmd_opcode = payload[0]; // This is an SP_CMD_...
        // const uint8_t *params = &payload[1]; // No params for current SP commands
        uint8_t params_len = length - 1;

        if (params_len != 0) // Current SP commands don't take parameters
        {
            status = CP_STATUS_INVALID_PARAMETER;
        }
        else
        {
            status = FS_StartControl_InitiateCommand(received_cmd_opcode);
        }
    }

    if (indication_enabled_flag)
    {
        uint8_t final_response_packet[3]; // Response is always 3 bytes for SP acks
        final_response_packet[0] = CP_RESPONSE_ID;
        final_response_packet[1] = received_cmd_opcode;
        final_response_packet[2] = status;

        SizeSp_Control_Point = 3;
        BLE_TX_Queue_SendTxPacket(CUSTOM_STM_SP_CONTROL_POINT,
                                  final_response_packet,
                                  3,
                                  &SizeSp_Control_Point,
                                  0);
    }
}
