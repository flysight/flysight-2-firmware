/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2025 Bionic Avionics Inc.                                   **
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
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "crs.h"
#include "stm32_seq.h"
#include "custom_stm.h"

typedef struct
{
	Custom_STM_Char_Opcode_t opcode;
	uint8_t data[244];
	uint8_t length;
	BLE_TX_Queue_callback_t callback;
} BLE_TX_Queue_Packet_t;

static BLE_TX_Queue_Packet_t tx_buffer[FS_CRS_WINDOW_LENGTH+1];
static uint32_t tx_read_index, tx_write_index;
static uint8_t tx_flow_status;

extern uint8_t SizeCrs_Tx;

static void BLE_TX_Queue_Transmit(void);

void BLE_TX_Queue_Init(void)
{
	tx_read_index = 0;
	tx_write_index = 0;
	tx_flow_status = 1;

	UTIL_SEQ_RegTask(1<<CFG_TASK_BLE_TX_QUEUE_TRANSMIT_ID, UTIL_SEQ_RFU,
			BLE_TX_Queue_Transmit);
}

void BLE_TX_Queue_TxPoolAvailableNotification(void)
{
	tx_flow_status = 1;
	UTIL_SEQ_SetTask(1<<CFG_TASK_BLE_TX_QUEUE_TRANSMIT_ID, CFG_SCH_PRIO_1);
}

uint8_t *BLE_TX_Queue_GetNextTxPacket(void)
{
	if (tx_write_index < tx_read_index + FS_CRS_WINDOW_LENGTH)
	{
		return tx_buffer[tx_write_index % FS_CRS_WINDOW_LENGTH].data;
	}
	else
	{
		return 0;
	}
}

void BLE_TX_Queue_SendNextTxPacket(Custom_STM_Char_Opcode_t opcode,
		uint8_t length, BLE_TX_Queue_callback_t callback)
{
	if (tx_write_index < tx_read_index + FS_CRS_WINDOW_LENGTH)
	{
		BLE_TX_Queue_Packet_t *packet =
				&tx_buffer[tx_write_index % FS_CRS_WINDOW_LENGTH];

		packet->opcode = opcode;
		packet->length = length;
		packet->callback = callback;

		++tx_write_index;
		UTIL_SEQ_SetTask(1<<CFG_TASK_BLE_TX_QUEUE_TRANSMIT_ID, CFG_SCH_PRIO_1);
	}
	else
	{
		APP_DBG_MSG("Custom_CRS_SendNextTxPacket: buffer overflow\n");
	}
}

void BLE_TX_Queue_SendTxPacket(Custom_STM_Char_Opcode_t opcode,
		uint8_t *data, uint8_t length, BLE_TX_Queue_callback_t callback)
{
	uint8_t *tx_buffer;
	if ((tx_buffer = BLE_TX_Queue_GetNextTxPacket()))
	{
		memcpy(tx_buffer, data, length);
		BLE_TX_Queue_SendNextTxPacket(opcode, length, callback);
	}
}

static void BLE_TX_Queue_Transmit(void)
{
	static uint8_t tx_busy = 0;
	BLE_TX_Queue_Packet_t *packet;
	tBleStatus status;

	if (!tx_busy &&
			(tx_read_index < tx_write_index) &&
			tx_flow_status)
	{
		tx_busy = 1;

		packet = &tx_buffer[tx_read_index % FS_CRS_WINDOW_LENGTH];
		SizeCrs_Tx = packet->length;

		status = Custom_STM_App_Update_Char(packet->opcode, packet->data);
		if (status == BLE_STATUS_INSUFFICIENT_RESOURCES)
		{
			tx_flow_status = 0;
		}
		else
		{
			++tx_read_index;

			// Call callback and transmit next packet
			if (packet->callback)
			{
				packet->callback();
			}
			UTIL_SEQ_SetTask(1<<CFG_TASK_BLE_TX_QUEUE_TRANSMIT_ID, CFG_SCH_PRIO_1);
		}

		tx_busy = 0;
	}
}
