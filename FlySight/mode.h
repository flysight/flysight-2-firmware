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

#ifndef MODE_H_
#define MODE_H_

typedef enum
{
	FS_MODE_EVENT_BUTTON_PRESSED,
	FS_MODE_EVENT_BUTTON_RELEASED,
	FS_MODE_EVENT_TIMER,
	FS_MODE_EVENT_VBUS_HIGH,
	FS_MODE_EVENT_VBUS_LOW,
	FS_MODE_EVENT_FORCE_UPDATE
} FS_Mode_Event_t;

typedef enum
{
	FS_MODE_STATE_SLEEP,
	FS_MODE_STATE_ACTIVE,
	FS_MODE_STATE_CONFIG,
	FS_MODE_STATE_USB,

	// Number of modes
	FS_MODE_STATE_COUNT
} FS_Mode_State_t;

void FS_Mode_Init(void);
void FS_Mode_PushQueue(FS_Mode_Event_t event);
FS_Mode_State_t FS_Mode_State(void);

#endif /* MODE_H_ */
