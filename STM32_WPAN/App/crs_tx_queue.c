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

#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "crs_tx_queue.h"
#include "crs.h"
#include "stm32_seq.h"
#include "custom_stm.h"

static CRS_TX_Queue_Packet_t tx_buffer[FS_CRS_WINDOW_LENGTH+1];
static uint32_t tx_read_index, tx_write_index;
static uint8_t tx_flow_status;

extern uint8_t SizeCrs_Tx;

static void CRS_TX_Queue_Transmit(void);

void CRS_TX_Queue_Init(void)
{
	tx_read_index = 0;
	tx_write_index = 0;
	tx_flow_status = 1;

	UTIL_SEQ_RegTask(1<<CFG_TASK_CRS_TX_QUEUE_TRANSMIT_ID, UTIL_SEQ_RFU,
			CRS_TX_Queue_Transmit);
}

void CRS_TX_Queue_TxPoolAvailableNotification(void)
{
	tx_flow_status = 1;
	UTIL_SEQ_SetTask(1<<CFG_TASK_CRS_TX_QUEUE_TRANSMIT_ID, CFG_SCH_PRIO_1);
}

CRS_TX_Queue_Packet_t *CRS_TX_Queue_GetNextTxPacket(void)
{
	CRS_TX_Queue_Packet_t *ret = 0;

	if (tx_write_index < tx_read_index + FS_CRS_WINDOW_LENGTH)
	{
		ret = &tx_buffer[tx_write_index % FS_CRS_WINDOW_LENGTH];
	}

	return ret;
}

void CRS_TX_Queue_SendNextTxPacket(void)
{
	if (tx_write_index < tx_read_index + FS_CRS_WINDOW_LENGTH)
	{
		++tx_write_index;
		UTIL_SEQ_SetTask(1<<CFG_TASK_CRS_TX_QUEUE_TRANSMIT_ID, CFG_SCH_PRIO_1);
	}
	else
	{
		APP_DBG_MSG("Custom_CRS_SendNextTxPacket: buffer overflow\n");
	}
}

static void CRS_TX_Queue_Transmit(void)
{
	static uint8_t tx_busy = 0;
	CRS_TX_Queue_Packet_t *packet;
	tBleStatus status;

	if (!tx_busy &&
			(tx_read_index < tx_write_index) &&
			tx_flow_status)
	{
		tx_busy = 1;

		packet = &tx_buffer[tx_read_index % FS_CRS_WINDOW_LENGTH];
		SizeCrs_Tx = packet->length;

		status = Custom_STM_App_Update_Char(CUSTOM_STM_CRS_TX, packet->data);
		if (status == BLE_STATUS_INSUFFICIENT_RESOURCES)
		{
			tx_flow_status = 0;
		}
		else
		{
			++tx_read_index;

			// Call update task and transmit next packet
			UTIL_SEQ_SetTask(1<<CFG_TASK_FS_CRS_UPDATE_ID, CFG_SCH_PRIO_1);
			UTIL_SEQ_SetTask(1<<CFG_TASK_CRS_TX_QUEUE_TRANSMIT_ID, CFG_SCH_PRIO_1);
		}

		tx_busy = 0;
	}
}
