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

#include <ble_tx_queue.h>
#include "main.h"
#include "app_common.h"
#include "crs.h"
#include "custom_app.h"
#include "dbg_trace.h"
#include "ff.h"
#include "resource_manager.h"
#include "stm32_seq.h"

#include <stdlib.h>

/* Uncomment the following to inject errors into the transmission */
/* #define TEST_MODE */

#define FRAME_LENGTH 242

#define TX_TIMEOUT_MSEC  200
#define TX_TIMEOUT_TICKS (TX_TIMEOUT_MSEC*1000/CFG_TS_TICK_VAL)

#define RX_TIMEOUT_MSEC  10000
#define RX_TIMEOUT_TICKS (RX_TIMEOUT_MSEC*1000/CFG_TS_TICK_VAL)

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
	FS_CRS_COMMAND_FILE_ACK  = 0x12,
	FS_CRS_COMMAND_NAK       = 0xf0,
	FS_CRS_COMMAND_ACK       = 0xf1,
	FS_CRS_COMMAND_PING      = 0xfe,
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

typedef FS_CRS_State_t FS_CRS_StateFunc_t(void);

static FS_CRS_State_t FS_CRS_State_Idle(void);
static FS_CRS_State_t FS_CRS_State_Read(void);
static FS_CRS_State_t FS_CRS_State_Write(void);
static FS_CRS_State_t FS_CRS_State_Dir(void);

static FS_CRS_StateFunc_t *const state_table[FS_CRS_STATE_COUNT] =
{
	FS_CRS_State_Idle,
	FS_CRS_State_Read,
	FS_CRS_State_Write,
	FS_CRS_State_Dir
};

static FS_CRS_State_t state = FS_CRS_STATE_IDLE;

static FIL file;
static DIR dir;

static uint32_t read_offset;
static uint32_t read_stride;
static uint32_t read_pos;

static uint32_t next_packet;
static uint32_t next_ack;
static uint32_t last_packet;

static uint8_t ack_timer_id;
static volatile uint8_t timeout_flag;

static void FS_CRS_TxCallback()
{
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_CRS_UPDATE_ID, CFG_SCH_PRIO_1);
}

static void FS_CRS_SendPacket(uint8_t command, uint8_t *payload, uint8_t length)
{
	uint8_t *tx_buffer;

	if ((tx_buffer = BLE_TX_Queue_GetNextTxPacket()))
	{
		tx_buffer[0] = command;
		memcpy(&tx_buffer[1], payload, length);
		BLE_TX_Queue_SendNextTxPacket(CUSTOM_STM_FT_PACKET_OUT, length + 1,
				&SizeFt_Packet_Out, FS_CRS_TxCallback);
	}
}

static void FS_CRS_SendNak(uint8_t command)
{
	FS_CRS_SendPacket(FS_CRS_COMMAND_NAK, &command, sizeof(command));
}

static void FS_CRS_SendAck(uint8_t command)
{
	FS_CRS_SendPacket(FS_CRS_COMMAND_ACK, &command, sizeof(command));
}

static FS_CRS_State_t FS_CRS_State_Idle(void)
{
	FS_CRS_State_t next_state = FS_CRS_STATE_IDLE;
	Custom_CRS_Packet_t *packet;

	while ((next_state == FS_CRS_STATE_IDLE) && (packet = Custom_CRS_GetNextRxPacket()))
	{
		if (packet->length > 0)
		{
			// Handle commands
			switch (packet->data[0])
			{
			case FS_CRS_COMMAND_CREATE:
				// Initialize disk
				if (FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS)
						== FS_RESOURCE_MANAGER_SUCCESS)
				{
					// Create file
					if (f_open(&file, (TCHAR *) &(packet->data[1]),
							FA_WRITE|FA_CREATE_NEW) == FR_OK)
					{
						FS_CRS_SendAck(FS_CRS_COMMAND_CREATE);
						f_close(&file);
					}
					else
					{
						FS_CRS_SendNak(FS_CRS_COMMAND_CREATE);
					}

					// De-initialize disk
					FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
				}
				else
				{
					FS_CRS_SendNak(FS_CRS_COMMAND_CREATE);
				}
				break;
			case FS_CRS_COMMAND_DELETE:
				// Initialize disk
				if (FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS)
						== FS_RESOURCE_MANAGER_SUCCESS)
				{
					// Delete file
					if (f_unlink((TCHAR *) &(packet->data[1])) == FR_OK)
					{
						FS_CRS_SendAck(FS_CRS_COMMAND_DELETE);
					}
					else
					{
						FS_CRS_SendNak(FS_CRS_COMMAND_DELETE);
					}

					// De-initialize disk
					FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
				}
				else
				{
					FS_CRS_SendNak(FS_CRS_COMMAND_DELETE);
				}
				break;
			case FS_CRS_COMMAND_READ:
				// Initialize disk
				if (FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS)
						== FS_RESOURCE_MANAGER_SUCCESS)
				{
					// Terminate file name
					packet->data[packet->length] = 0;

					// Open file
					if (f_open(&file, (TCHAR *) &(packet->data[9]), FA_READ) == FR_OK)
					{
						// Initialize read stride and position
						read_offset = *((uint32_t *) &(packet->data[1])) * FRAME_LENGTH;
						read_stride = (*((uint32_t *) &(packet->data[5])) + 1) * FRAME_LENGTH;
						read_pos = read_offset;

						// Initialize flow control
						next_packet = 0;
						next_ack = 0;
						last_packet = -1;

						// Start timeout timer
						HW_TS_Start(ack_timer_id, TX_TIMEOUT_TICKS);
						timeout_flag = 0;

						if (f_lseek(&file, read_pos) == FR_OK)
						{
							FS_CRS_SendAck(FS_CRS_COMMAND_READ);

							// Call update task
							UTIL_SEQ_SetTask(1<<CFG_TASK_FS_CRS_UPDATE_ID, CFG_SCH_PRIO_1);

							next_state = FS_CRS_STATE_READ;
						}
					}

					if (next_state == FS_CRS_STATE_IDLE)
					{
						FS_CRS_SendNak(FS_CRS_COMMAND_READ);

						// De-initialize disk
						FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
					}
				}
				else
				{
					FS_CRS_SendNak(FS_CRS_COMMAND_READ);
				}
				break;
			case FS_CRS_COMMAND_WRITE:
				// Initialize disk
				if (FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS)
						== FS_RESOURCE_MANAGER_SUCCESS)
				{
					// Terminate file name
					packet->data[packet->length] = 0;

					// Open file
					if (f_open(&file, (TCHAR *) &(packet->data[1]), FA_WRITE|FA_CREATE_ALWAYS) == FR_OK)
					{
						FS_CRS_SendAck(FS_CRS_COMMAND_WRITE);

						// Initialize flow control
						next_packet = 0;

						// Start timeout timer
						HW_TS_Start(ack_timer_id, RX_TIMEOUT_TICKS);
						timeout_flag = 0;

						next_state = FS_CRS_STATE_WRITE;
					}

					if (next_state == FS_CRS_STATE_IDLE)
					{
						FS_CRS_SendNak(FS_CRS_COMMAND_WRITE);

						// De-initialize disk
						FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
					}
				}
				else
				{
					FS_CRS_SendNak(FS_CRS_COMMAND_WRITE);
				}
				break;
			case FS_CRS_COMMAND_MK_DIR:
				// Initialize disk
				if (FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS)
						== FS_RESOURCE_MANAGER_SUCCESS)
				{
					// Create directory
					if (f_mkdir((TCHAR *) &(packet->data[1])) == FR_OK)
					{
						FS_CRS_SendAck(FS_CRS_COMMAND_MK_DIR);
					}
					else
					{
						FS_CRS_SendNak(FS_CRS_COMMAND_MK_DIR);
					}

					// De-initialize disk
					FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
				}
				else
				{
					FS_CRS_SendNak(FS_CRS_COMMAND_MK_DIR);
				}
				break;
			case FS_CRS_COMMAND_READ_DIR:
				// Initialize disk
				if (FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS)
						== FS_RESOURCE_MANAGER_SUCCESS)
				{
					// Terminate file name
					packet->data[packet->length] = 0;

					// Open directory
					if (f_opendir(&dir, (TCHAR *) &(packet->data[1])) == FR_OK)
					{
						FS_CRS_SendAck(FS_CRS_COMMAND_READ_DIR);

						// Initialize flow control
						next_packet = 0;

						// Call update task
						UTIL_SEQ_SetTask(1<<CFG_TASK_FS_CRS_UPDATE_ID, CFG_SCH_PRIO_1);

						next_state = FS_CRS_STATE_DIR;
					}
					else
					{
						FS_CRS_SendNak(FS_CRS_COMMAND_READ_DIR);

						// De-initialize disk
						FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
					}
				}
				else
				{
					FS_CRS_SendNak(FS_CRS_COMMAND_READ_DIR);
				}
				break;
			case FS_CRS_COMMAND_PING:
				FS_CRS_SendAck(FS_CRS_COMMAND_PING);
				break;
			default:
				FS_CRS_SendNak(packet->data[0]);
			}
		}
	}

	return next_state;
}

static FS_CRS_State_t FS_CRS_State_Dir(void)
{
	FS_CRS_State_t next_state = FS_CRS_STATE_DIR;
	Custom_CRS_Packet_t *packet;
	uint8_t *tx_buffer;
	FILINFO fno;

	if (!Custom_APP_IsConnected())
	{
		next_state = FS_CRS_STATE_IDLE;
	}

	while ((next_state == FS_CRS_STATE_DIR) && (packet = Custom_CRS_GetNextRxPacket()))
	{
		if (packet->length >= 1)
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

	while ((next_state == FS_CRS_STATE_DIR) &&
			(tx_buffer = BLE_TX_Queue_GetNextTxPacket()))
	{
		// Read a directory item
		if (f_readdir(&dir, &fno) == FR_OK)
		{
			tx_buffer[0] = FS_CRS_COMMAND_FILE_INFO;
			tx_buffer[1] = ((next_packet++) & 0xff);
			memcpy(&tx_buffer[2], &fno.fsize, sizeof(fno.fsize));
			memcpy(&tx_buffer[6], &fno.fdate, sizeof(fno.fdate));
			memcpy(&tx_buffer[8], &fno.ftime, sizeof(fno.ftime));
			tx_buffer[10] = fno.fattrib;
			memcpy(&tx_buffer[11], fno.fname, sizeof(fno.fname));

			BLE_TX_Queue_SendNextTxPacket(CUSTOM_STM_FT_PACKET_OUT, 24,
					&SizeFt_Packet_Out, FS_CRS_TxCallback);

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

	if (next_state == FS_CRS_STATE_IDLE)
	{
		// Close directory
		f_closedir(&dir);

		// De-initialize disk
		FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
	}

	return next_state;
}

static FS_CRS_State_t FS_CRS_State_Read(void)
{
	FS_CRS_State_t next_state = FS_CRS_STATE_READ;
	Custom_CRS_Packet_t *packet;
	uint8_t *tx_buffer;
	UINT br;

	if (!Custom_APP_IsConnected())
	{
		next_state = FS_CRS_STATE_IDLE;
	}

	if (timeout_flag)
	{
		next_packet = next_ack;
		timeout_flag = 0;

		read_pos = read_offset + next_packet * read_stride;
		f_lseek(&file, read_pos);

		// Reset timeout timer
		HW_TS_Start(ack_timer_id, TX_TIMEOUT_TICKS);
	}

	while ((next_state == FS_CRS_STATE_READ) && (packet = Custom_CRS_GetNextRxPacket()))
	{
		if (packet->length >= 1)
		{
			// Handle commands
			switch (packet->data[0])
			{
			case FS_CRS_COMMAND_CANCEL:
				next_state = FS_CRS_STATE_IDLE;
				break;
			case FS_CRS_COMMAND_FILE_ACK:
				if (packet->length >= 2)
				{
					if (packet->data[1] == (next_ack & 0xff))
					{
						++next_ack;

						// Reset timeout timer
						HW_TS_Start(ack_timer_id, TX_TIMEOUT_TICKS);
					}
				}
				break;
			}
		}
	}

	while ((next_state == FS_CRS_STATE_READ) &&
			(tx_buffer = BLE_TX_Queue_GetNextTxPacket()) &&
			(next_packet < next_ack + FS_CRS_WINDOW_LENGTH) &&
			(next_packet < last_packet))
	{
		tx_buffer[0] = FS_CRS_COMMAND_FILE_DATA;
		tx_buffer[1] = (next_packet & 0xff);

		if (f_eof(&file))
		{
			// Send empty buffer to signal end of file
			BLE_TX_Queue_SendNextTxPacket(CUSTOM_STM_FT_PACKET_OUT, 2,
					&SizeFt_Packet_Out, FS_CRS_TxCallback);
			last_packet = ++next_packet;
		}
		else if (f_read(&file, &tx_buffer[2], FRAME_LENGTH, &br) == FR_OK)
		{
			if (read_stride > FRAME_LENGTH)
			{
				read_pos += read_stride;
				f_lseek(&file, read_pos);
			}

			// Send data
			BLE_TX_Queue_SendNextTxPacket(CUSTOM_STM_FT_PACKET_OUT, br + 2,
					&SizeFt_Packet_Out, FS_CRS_TxCallback);
			++next_packet;
		}
		else
		{
			next_state = FS_CRS_STATE_IDLE;
		}
	}

	if (next_ack == last_packet)
	{
		next_state = FS_CRS_STATE_IDLE;
	}

	if (next_state == FS_CRS_STATE_IDLE)
	{
		// Stop timeout timer
		HW_TS_Stop(ack_timer_id);

		// Close file
		f_close(&file);

		// De-initialize disk
		FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
	}

	return next_state;
}

static FS_CRS_State_t FS_CRS_State_Write(void)
{
	FS_CRS_State_t next_state = FS_CRS_STATE_WRITE;
	UINT bw;
	Custom_CRS_Packet_t *packet;

	if (!Custom_APP_IsConnected())
	{
		next_state = FS_CRS_STATE_IDLE;
	}

	if (timeout_flag)
	{
		next_state = FS_CRS_STATE_IDLE;
	}

	while ((next_state == FS_CRS_STATE_WRITE) && (packet = Custom_CRS_GetNextRxPacket()))
	{
		if (packet->length >= 1)
		{
			// Handle commands
			switch (packet->data[0])
			{
			case FS_CRS_COMMAND_FILE_DATA:
				if (packet->length >= 2)
				{
#ifdef TEST_MODE
					if ((packet->data[1] == (next_packet & 0xff)) && (rand() % 100 >= 30))
#else
					if (packet->data[1] == (next_packet & 0xff))
#endif
					{
						f_write(&file, &packet->data[2], packet->length - 2, &bw);
						FS_CRS_SendPacket(FS_CRS_COMMAND_FILE_ACK, &packet->data[1], 1);
						++next_packet;

						// Reset timeout timer
						HW_TS_Start(ack_timer_id, RX_TIMEOUT_TICKS);
					}

					if (packet->length == 2)
					{
						next_state = FS_CRS_STATE_IDLE;
					}
				}
				break;
			case FS_CRS_COMMAND_CANCEL:
				next_state = FS_CRS_STATE_IDLE;
				break;
			}
		}
	}

	if (next_state == FS_CRS_STATE_IDLE)
	{
		// Stop timeout timer
		HW_TS_Stop(ack_timer_id);

		// Close file
		f_close(&file);

		// De-initialize disk
		FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
	}

	return next_state;
}

static void FS_CRS_Update(void)
{
	state = state_table[state]();
}

static void FS_CRS_Timeout(void)
{
	timeout_flag = 1;

	// Call update task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_CRS_UPDATE_ID, CFG_SCH_PRIO_1);
}

void FS_CRS_Init(void)
{
	// Initialize CRS task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_CRS_UPDATE_ID, UTIL_SEQ_RFU, FS_CRS_Update);

	// Initialize timeout timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &ack_timer_id, hw_ts_SingleShot, FS_CRS_Timeout);
}
