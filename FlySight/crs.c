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
#define FILE_READ_LENGTH 243

typedef enum
{
	FS_CRS_COMMAND_CREATE    = 0x00,
	FS_CRS_COMMAND_DELETE    = 0x01,
	FS_CRS_COMMAND_READ      = 0x02,
	FS_CRS_COMMAND_WRITE     = 0x03,
	FS_CRS_COMMAND_MK_DIR    = 0x04,
	FS_CRS_COMMAND_READ_DIR  = 0x05,
	FS_CRS_COMMAND_FILE_DATA = 0x10,
	FS_CRS_COMMAND_FILE_INFO = 0x11,
	FS_CRS_COMMAND_CANCEL    = 0xff
} FS_CRS_Command_t;

typedef enum
{
	FS_CRS_STATE_IDLE,
	FS_CRS_STATE_READ,
	FS_CRS_STATE_WRITE,
	FS_CRS_STATE_DIR,

	// Number of states
	FS_CRS_STATE_COUNT
} FS_CRS_State_t;

typedef FS_CRS_State_t FS_CRS_StateFunc_t(FS_CRS_Event_t event);

static FS_CRS_State_t FS_CRS_State_Idle(FS_CRS_Event_t event);
static FS_CRS_State_t FS_CRS_State_Read(FS_CRS_Event_t event);
static FS_CRS_State_t FS_CRS_State_Write(FS_CRS_Event_t event);
static FS_CRS_State_t FS_CRS_State_Dir(FS_CRS_Event_t event);

static FS_CRS_StateFunc_t *const state_table[FS_CRS_STATE_COUNT] =
{
	FS_CRS_State_Idle,
	FS_CRS_State_Read,
	FS_CRS_State_Write,
	FS_CRS_State_Dir
};

static FS_CRS_State_t state = FS_CRS_STATE_IDLE;

static FIL file;
static uint8_t buffer[FILE_READ_LENGTH];
static DIR dir;

static FSIZE_t read_stride;
static FSIZE_t read_pos;

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
				case FS_CRS_COMMAND_CREATE:
					// Initialize disk
					FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS);

					// Create file
					if (f_open(&file, (TCHAR *) &(packet->data[1]),
							FA_WRITE|FA_CREATE_NEW) == FR_OK)
					{
						f_close(&file);
					}

					// De-initialize disk
					FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
					break;
				case FS_CRS_COMMAND_DELETE:
					// Initialize disk
					FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS);

					// Delete file
					f_unlink((TCHAR *) &(packet->data[1]));

					// De-initialize disk
					FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
					break;
				case FS_CRS_COMMAND_READ:
					// Initialize disk
					FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS);

					// Terminate file name
					packet->data[packet->length] = 0;

					// Open file
					if (f_open(&file, (TCHAR *) &(packet->data[5]), FA_READ) == FR_OK)
					{
						// Initialize read stride and position
						read_stride = ((FSIZE_t) *((uint16_t *) &(packet->data[3])) + 1) * FILE_READ_LENGTH;
						read_pos = (FSIZE_t) *((uint16_t *) &(packet->data[1])) * FILE_READ_LENGTH;

						if (f_lseek(&file, read_pos) == FR_OK)
						{
							// Call update task
							FS_CRS_PushQueue(FS_CRS_EVENT_TX_READ);

							next_state = FS_CRS_STATE_READ;
						}
					}

					if (next_state == FS_CRS_STATE_IDLE)
					{
						// De-initialize disk
						FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
					}
					break;
				case FS_CRS_COMMAND_WRITE:
					// Initialize disk
					FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS);

					// Terminate file name
					packet->data[packet->length] = 0;

					// Open file
					if (f_open(&file, (TCHAR *) &(packet->data[1]), FA_WRITE) == FR_OK)
					{
						next_state = FS_CRS_STATE_WRITE;
					}

					if (next_state == FS_CRS_STATE_IDLE)
					{
						// De-initialize disk
						FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
					}
					break;
				case FS_CRS_COMMAND_MK_DIR:
					// Initialize disk
					FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS);

					// Create directory
					f_mkdir((TCHAR *) &(packet->data[1]));

					// De-initialize disk
					FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
					break;
				case FS_CRS_COMMAND_READ_DIR:
					// Initialize disk
					FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS);

					// Terminate file name
					packet->data[packet->length] = 0;

					// Open directory
					if (f_opendir(&dir, (TCHAR *) &(packet->data[1])) == FR_OK)
					{
					    // Call update task
					    FS_CRS_PushQueue(FS_CRS_EVENT_TX_READ);

						next_state = FS_CRS_STATE_DIR;
					}
					else
					{
						// De-initialize disk
						FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
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
			memcpy(buffer, &fno.fsize, sizeof(fno.fsize));
			memcpy(buffer + 4, &fno.fdate, sizeof(fno.fdate));
			memcpy(buffer + 6, &fno.ftime, sizeof(fno.ftime));
			buffer[8] = fno.fattrib;
			memcpy(buffer + 9, fno.fname, sizeof(fno.fname));

			FS_CRS_SendPacket(FS_CRS_COMMAND_FILE_INFO, buffer, 22);

			if (fno.fname[0] == 0)
			{
				next_state = FS_CRS_STATE_IDLE;
			}
		}
		else
		{
			next_state = FS_CRS_STATE_IDLE;
		}
	}
	else if (event == FS_CRS_EVENT_DISCONNECT)
	{
		next_state = FS_CRS_STATE_IDLE;
	}

	if (next_state == FS_CRS_STATE_IDLE)
	{
		// Close directory
		f_closedir(&dir);

		// De-initialize disk
		FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
	}

	return next_state;
}

static FS_CRS_State_t FS_CRS_State_Read(FS_CRS_Event_t event)
{
	FS_CRS_State_t next_state = FS_CRS_STATE_READ;
	UINT br;
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
					next_state = FS_CRS_STATE_IDLE;
					break;
				}
			}
		}
	}
	else if (event == FS_CRS_EVENT_TX_READ)
	{
		if (f_eof(&file))
		{
			// Send empty buffer to signal end of file
			FS_CRS_SendPacket(FS_CRS_COMMAND_FILE_DATA, buffer, 0);

			next_state = FS_CRS_STATE_IDLE;
		}
		else if (f_read(&file, buffer, FILE_READ_LENGTH, &br) == FR_OK)
		{
			if (read_stride > FILE_READ_LENGTH)
			{
				read_pos += read_stride;
				f_lseek(&file, read_pos);
			}

			// Send data
			FS_CRS_SendPacket(FS_CRS_COMMAND_FILE_DATA, buffer, br);
		}
		else
		{
			next_state = FS_CRS_STATE_IDLE;
		}
	}
	else if (event == FS_CRS_EVENT_DISCONNECT)
	{
		next_state = FS_CRS_STATE_IDLE;
	}

	if (next_state == FS_CRS_STATE_IDLE)
	{
		// Close file
		f_close(&file);

		// De-initialize disk
		FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
	}

	return next_state;
}

static FS_CRS_State_t FS_CRS_State_Write(FS_CRS_Event_t event)
{
	FS_CRS_State_t next_state = FS_CRS_STATE_WRITE;
	UINT bw;
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
				case FS_CRS_COMMAND_FILE_DATA:
					if (packet->length > 1)
					{
						f_write(&file, &packet->data[1], packet->length - 1, &bw);
					}
					else
					{
						next_state = FS_CRS_STATE_IDLE;
					}
					break;
				case FS_CRS_COMMAND_CANCEL:
					next_state = FS_CRS_STATE_IDLE;
					break;
				}
			}
		}
	}
	else if (event == FS_CRS_EVENT_DISCONNECT)
	{
		next_state = FS_CRS_STATE_IDLE;
	}

	if (next_state == FS_CRS_STATE_IDLE)
	{
		// Close file
		f_close(&file);

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
