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
#include "stm32_seq.h"

#define CRS_UPDATE_MSEC    100
#define CRS_UPDATE_RATE    (CRS_UPDATE_MSEC*1000/CFG_TS_TICK_VAL)

static uint8_t timer_id;

static void FS_CRS_Timer(void);
static void FS_CRS_Update(void);

void FS_CRS_Init(void)
{
	// Initialize CRS task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_CRS_UPDATE_ID, UTIL_SEQ_RFU, FS_CRS_Update);

	// Initialize CRS update timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_Repeated, FS_CRS_Timer);
	HW_TS_Start(timer_id, CRS_UPDATE_RATE);
}

static void FS_CRS_Timer(void)
{
	// Call update task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_CRS_UPDATE_ID, CFG_SCH_PRIO_1);
}

static void FS_CRS_Update(void)
{
	Custom_CRS_Packet_t *rx_packet, *tx_packet;

	while ((rx_packet = Custom_CRS_GetNextRxPacket()))
	{
		if ((tx_packet = Custom_CRS_GetNextTxPacket()))
		{
			tx_packet->length = rx_packet->length;
			memcpy(tx_packet->data, rx_packet->data, tx_packet->length);
			Custom_CRS_SendNextTxPacket();
		}
	}
}
