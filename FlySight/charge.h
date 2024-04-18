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

#ifndef CHARGE_H_
#define CHARGE_H_

#define FS_CHARGE_DISABLE 0
#define FS_CHARGE_100MA   1
#define FS_CHARGE_200MA   2
#define FS_CHARGE_300MA   3

typedef uint8_t FS_Charge_Current_t;

typedef enum
{
	FS_CHARGE_ACTIVE = 0,
	FS_CHARGE_COMPLETE
} FS_Charge_State_t;

void FS_Charge_SetCurrent(FS_Charge_Current_t charge_current);
FS_Charge_State_t FS_Charge_GetState(void);

#endif /* CHARGE_H_ */
