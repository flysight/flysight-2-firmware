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

#ifndef START_CONTROL_H_
#define START_CONTROL_H_

#include <stdint.h>

void FS_StartControl_RegisterTasks(void);
void FS_StartControl_Init(void);
void FS_StartControl_DeInit(void);

// Function to handle writes to the SP Control Point
void FS_StartControl_Handle_SP_ControlPointWrite(
		const uint8_t *payload, uint8_t length,
		uint16_t conn_handle, uint8_t indication_enabled_flag);

#endif /* START_CONTROL_H_ */
