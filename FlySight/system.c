/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2024 Bionic Avionics Inc.                                   **
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
#include "common.h"
#include "custom_app.h"
#include "mode.h"
#include "stm32_seq.h"

typedef enum
{
	FS_SYSTEM_COMMAND_CREATE_TOKEN = 0x00,
	FS_SYSTEM_COMMAND_CHECK_TOKEN  = 0x01,
	FS_SYSTEM_COMMAND_NAK          = 0xf0,
	FS_SYSTEM_COMMAND_ACK          = 0xf1
} FS_System_Command_t;

typedef enum
{
	FS_SYSTEM_STATE_IDLE,

	// Number of states
	FS_SYSTEM_STATE_COUNT
} FS_System_State_t;

typedef FS_System_State_t FS_System_StateFunc_t(void);

static FS_System_State_t FS_System_State_Idle(void);

static FS_System_StateFunc_t *const state_table[FS_SYSTEM_STATE_COUNT] =
{
	FS_System_State_Idle
};

static FS_System_State_t state = FS_SYSTEM_STATE_IDLE;

static FS_System_State_t FS_System_State_Idle(void)
{
	FS_System_State_t next_state = FS_SYSTEM_STATE_IDLE;
	Custom_System_Packet_t *packet = Custom_System_GetNextPacket();

	if (packet)
	{
		// Handle commands
		switch (packet->command)
		{
		case FS_SYSTEM_COMMAND_CREATE_TOKEN:
			// Initialize response packet
			memset(packet, 0, sizeof(*packet));

			if (Custom_System_GetConnectMode() == FS_MODE_STATE_PAIRING)
			{
				// Get random token
				FS_Common_GetRandomBytes((uint32_t *)packet->token,
						sizeof(packet->token) / 4);
				packet->command = FS_SYSTEM_COMMAND_ACK;

				// TODO: Save token and description

				// TODO: Check token against accepted list
			}
			else
			{
				// Not in pairing mode
				packet->command = FS_SYSTEM_COMMAND_NAK;
			}

			// Call update task
			UTIL_SEQ_SetTask(1<<CFG_TASK_CUSTOM_SYSTEM_UPDATE_ID, CFG_SCH_PRIO_1);
			break;
		case FS_SYSTEM_COMMAND_CHECK_TOKEN:
			// TODO: Check token against accepted list
			break;
		}
	}

	return next_state;
}

static void FS_System_Update(void)
{
	state = state_table[state]();
}

void FS_System_Init(void)
{
	// Initialize system update task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_SYSTEM_UPDATE_ID, UTIL_SEQ_RFU, FS_System_Update);
}
