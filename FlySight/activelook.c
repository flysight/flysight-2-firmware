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

#include "main.h"
#include "activelook.h"
#include "activelook_client.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "stm32_seq.h"
#include <string.h>

/* State */
static uint8_t clear_display = 0;

/* Forward declaration */
static void OnActiveLookDiscoveryComplete(void);

/* We'll define the callback struct */
static const FS_ActiveLook_ClientCb_t s_alk_cb =
{
	.OnDiscoveryComplete = OnActiveLookDiscoveryComplete
};

void FS_ActiveLook_Init(void)
{
    /* Register the callback before scanning/connecting. */
	FS_ActiveLook_Client_RegisterCb(&s_alk_cb);

    /* Start scanning for BLE peripherals,
       leading eventually to connect to the ActiveLook device. */
	UTIL_SEQ_SetTask(1 << CFG_TASK_START_SCAN_ID, CFG_SCH_PRIO_0);
}

void FS_ActiveLook_DeInit(void)
{
	/* Disconnect from BLE device #1 */
	UTIL_SEQ_SetTask(1 << CFG_TASK_DISCONN_DEV_1_ID, CFG_SCH_PRIO_0);
}

/* This function is called once the client discovered the Rx characteristic. */
static void OnActiveLookDiscoveryComplete(void)
{
	APP_DBG_MSG("ActiveLook: Discovery complete\n");
	clear_display = 1;
}

void FS_ActiveLook_GNSS_Update(const FS_GNSS_Data_t *current)
{
	if (!FS_ActiveLook_Client_IsReady())
		return;

	uint8_t packet[26];
	uint8_t index, length_index;

	if (clear_display)
	{
		clear_display = 0;

		index = 0;
		packet[index++] = 0xFF;
		packet[index++] = 0x01;     // 'clear'
		packet[index++] = 0x00;
		packet[index++] = 5;        // total length
		packet[index++] = 0xAA;

		APP_DBG_MSG("ActiveLook: Clearing screen\n");
		FS_ActiveLook_Client_WriteWithoutResp(packet, index);
	}
	else
	{
		index = 0;
		packet[index++] = 0xFF;
		packet[index++] = 0x37;     // 'txt'
		packet[index++] = 0x00;
		length_index = index++;

		// s16 x=255, y=128
		packet[index++] = 0; packet[index++] = 255;
		packet[index++] = 0; packet[index++] = 128;

		// rotation=4, font=2, color=15
		packet[index++] = 4;
		packet[index++] = 2;
		packet[index++] = 15;

		char text[14];
		snprintf(text, 14, "t: %3lu", (current->iTOW / 1000) % 1000);
		size_t text_len = strlen(text) + 1;
		memcpy(&packet[index], text, text_len);
		index += text_len;

		packet[index++] = 0xAA;
		packet[length_index] = index; // total length

		APP_DBG_MSG("ActiveLook: Updating GNSS\n");
		FS_ActiveLook_Client_WriteWithoutResp(packet, index);
	}
}
