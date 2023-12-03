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
#include "crs.h"
#include "custom_app.h"
#include "ff.h"
#include "resource_manager.h"
#include "stm32_seq.h"

#define QUEUE_LENGTH 4

typedef enum
{
	FS_CRS_COMMAND_DIR = 0x00,
	FS_CRS_RESP_FILE_INFO = 0x01,
	FS_CRS_COMMAND_CANCEL = 0xff
} FS_CRS_Command_t;

typedef enum
{
	FS_CRS_STATE_IDLE,
	FS_CRS_STATE_DIR,

	// Number of states
	FS_CRS_STATE_COUNT
} FS_CRS_State_t;

typedef FS_CRS_State_t FS_CRS_StateFunc_t(FS_CRS_Event_t event);

static FS_CRS_State_t FS_CRS_State_Idle(FS_CRS_Event_t event);
static FS_CRS_State_t FS_CRS_State_Dir(FS_CRS_Event_t event);

static FS_CRS_StateFunc_t *const state_table[FS_CRS_STATE_COUNT] =
{
	FS_CRS_State_Idle,
	FS_CRS_State_Dir
};

static FS_CRS_State_t state = FS_CRS_STATE_IDLE;

static DIR dir;

static FS_CRS_Event_t event_queue[QUEUE_LENGTH];
static uint8_t queue_read = 0;
static uint8_t queue_write = 0;

void FS_CRS_PushQueue(FS_CRS_Event_t event)
{
	// TODO: Log if this queue overflows

	event_queue[queue_write] = event;
	queue_write = (queue_write + 1) % QUEUE_LENGTH;

	// Call update task
    UTIL_SEQ_SetTask(1<<CFG_TASK_FS_CRS_UPDATE_ID, CFG_SCH_PRIO_1);
}

FS_CRS_Event_t FS_CRS_PopQueue(void)
{
	// TODO: Log if this queue underflows

	const FS_CRS_Event_t event = event_queue[queue_read];
	queue_read = (queue_read + 1) % QUEUE_LENGTH;
	return event;
}

static void FS_CRS_SendPacket(uint8_t command, uint8_t *payload, uint8_t length)
{
	Custom_CRS_Packet_t *tx_packet;

	if ((tx_packet = Custom_CRS_GetNextTxPacket()))
	{
		tx_packet->length = length + 1;
		*(tx_packet->data) = command;
		memcpy(tx_packet->data + 1, payload, length);
		Custom_CRS_SendNextTxPacket();
	}
}

static FS_CRS_State_t FS_CRS_State_Idle(FS_CRS_Event_t event)
{
	FS_CRS_State_t next_state = FS_CRS_STATE_IDLE;
	Custom_CRS_Packet_t *packet;

	if (event == FS_CRS_EVENT_RX_WRITE)
	{
		if ((packet = Custom_CRS_GetNextRxPacket()))
		{
			if (packet->length > 0)
			{
				// Handle commands
				switch (packet->data[0])
				{
				case FS_CRS_COMMAND_DIR:
					// Initialize disk
					FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS);

					// Open directory
					if (f_opendir(&dir, (TCHAR *) &(packet->data[1])) == FR_OK)
					{
					    // Call update task
					    FS_CRS_PushQueue(FS_CRS_EVENT_TX_READ);

						next_state = FS_CRS_STATE_DIR;
					}
					break;
				}
			}
		}
	}

	return next_state;
}

static FS_CRS_State_t FS_CRS_State_Dir(FS_CRS_Event_t event)
{
	FS_CRS_State_t next_state = FS_CRS_STATE_DIR;
	FILINFO fno;
	Custom_CRS_Packet_t *packet;

	if (event == FS_CRS_EVENT_RX_WRITE)
	{
		if ((packet = Custom_CRS_GetNextRxPacket()))
		{
			if (packet->length > 0)
			{
				// Handle commands
				switch (packet->data[0])
				{
				case FS_CRS_COMMAND_CANCEL:
					f_closedir(&dir);
					next_state = FS_CRS_STATE_IDLE;
					break;
				}
			}
		}
	}
	else if (event == FS_CRS_EVENT_TX_READ)
	{
		// Read a directory item
		if (f_readdir(&dir, &fno) == FR_OK)
		{
			FS_CRS_SendPacket(FS_CRS_RESP_FILE_INFO, (uint8_t *) &fno, sizeof(fno));

			if (fno.fname[0] == 0)
			{
				f_closedir(&dir);
				next_state = FS_CRS_STATE_IDLE;
			}
		}
		else
		{
			f_closedir(&dir);
			next_state = FS_CRS_STATE_IDLE;
		}
	}

	if (next_state == FS_CRS_STATE_IDLE)
	{
		// De-initialize disk
		FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
	}

	return next_state;
}

static void FS_CRS_Update(void)
{
	while (queue_read != queue_write)
	{
		state = state_table[state](FS_CRS_PopQueue());
	}
}

void FS_CRS_Init(void)
{
	// Initialize CRS task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_CRS_UPDATE_ID, UTIL_SEQ_RFU, FS_CRS_Update);
}
