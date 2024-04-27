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
#include "app_fatfs.h"
#include "audio.h"
#include "config.h"
#include "led.h"
#include "mode.h"
#include "resource_manager.h"
#include "state.h"
#include "stm32_seq.h"

#define PLAY_TIMER_MSEC    10
#define PLAY_TIMER_TICKS   (PLAY_TIMER_MSEC*1000/CFG_TS_TICK_VAL)

#define WAIT_TIMER_MSEC    500
#define WAIT_TIMER_TICKS   (WAIT_TIMER_MSEC*1000/CFG_TS_TICK_VAL)

static DIR dir;
static FIL file;

static uint8_t timer_id;

typedef enum
{
	STATE_IDLE,
	STATE_PLAY,
	STATE_WAIT,
	STATE_DONE
} State_t;

static State_t state = STATE_IDLE;

static char config_filename[13];

extern SPI_HandleTypeDef hspi2;

static void updateTimer(void)
{
	// Call update task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_CONFIG_UPDATE_ID, CFG_SCH_PRIO_1);
}

static void readSingleConfigName(
	char *fname)
{
	char    buffer[100];
	size_t  len;

	char    *name;
	char    *result;
	char    filename[13];

	if (f_chdir("/config") != FR_OK)
		return;

	if (f_open(&file, fname, FA_READ) != FR_OK)
		return;

	while (!f_eof(&file))
	{
		f_gets(buffer, sizeof(buffer), &file);

		len = strcspn(buffer, ";");
		buffer[len] = '\0';

		name = strtok(buffer, " \r\n\t:");
		if (name == 0) continue ;

		result = strtok(0, " \r\n\t:");
		if (result == 0) continue ;

		if (!strcmp(name, "Init_File"))
		{
			filename[0] = '\0';

			strncat(filename, result, sizeof(filename) - 1);
			strncat(filename, ".wav", sizeof(filename) - 1);

			FS_Audio_Play(filename, FS_Config_Get()->sp_volume * 5);
		}
	}

	f_close(&file);
}

static void updateTask(void)
{
	FILINFO fno;

	if (state == STATE_PLAY)
	{
		if (FS_Audio_IsIdle())
		{
			state = STATE_WAIT;
			HW_TS_Start(timer_id, WAIT_TIMER_TICKS);
		}
		else
		{
			HW_TS_Start(timer_id, PLAY_TIMER_TICKS);
		}
	}
	else
	{
		State_t next_state = STATE_DONE;

		if (f_readdir(&dir, &fno) == FR_OK)
		{
			if ((fno.fname[0] != 0) &&
					(fno.fname[0] != '.') &&
					!(fno.fattrib & AM_DIR))
			{
				// Update current config file
				strncpy(config_filename, fno.fname, sizeof(config_filename));

				// Play init file
				readSingleConfigName(fno.fname);

				// Update state
				next_state = STATE_PLAY;
				HW_TS_Start(timer_id, PLAY_TIMER_TICKS);
			}
		}

		state = next_state;
	}

	if (state == STATE_DONE)
	{
		// Update current config file
		config_filename[0] = 0;

		// Exit config mode
		FS_Mode_PushQueue(FS_MODE_EVENT_FORCE_UPDATE);
	}
}

void FS_ConfigMode_Init(void)
{
	/* Initialize FatFS */
	FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS);

	// Turn on green LED
	FS_LED_SetColour(FS_LED_GREEN);
	FS_LED_On();

	// Initialize configuration
	FS_Config_Init();
	FS_Config_Read("/config.txt");

	// Initialize audio
	FS_Audio_Init();

	// Initialize update task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_CONFIG_UPDATE_ID, UTIL_SEQ_RFU, updateTask);

	// Initialize update timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_SingleShot, updateTimer);

	if (f_opendir(&dir, "/config") == FR_OK)
	{
		// Start the state machine
		state = STATE_WAIT;
		updateTask();
	}
	else
	{
		// Update current config file
		config_filename[0] = 0;

		// Exit config mode
		FS_Mode_PushQueue(FS_MODE_EVENT_FORCE_UPDATE);
	}
}

void FS_ConfigMode_DeInit(void)
{
	// Turn off LEDs
	FS_LED_Off();

	// Close directory
	f_closedir(&dir);

	// Delete update timer
	HW_TS_Delete(timer_id);

	// Disable audio
	FS_Audio_DeInit();

	// Update configuration filename
	FS_State_SetConfigFilename(config_filename);

	/* De-initialize FatFS */
	FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);

	// Reset state
	state = STATE_IDLE;
}
