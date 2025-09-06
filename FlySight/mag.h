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

#ifndef MAG_H_
#define MAG_H_

typedef struct
{
	uint32_t time;			// ms
	int16_t x;				// gauss * 1000
	int16_t y;				// gauss * 1000
	int16_t z;				// gauss * 1000
	int16_t temperature;	// degrees C * 10
} FS_Mag_Data_t;

void FS_Mag_Init(void);
HAL_StatusTypeDef FS_Mag_Start(void);
void FS_Mag_Stop(void);
void FS_Mag_Read(void);
const FS_Mag_Data_t *FS_Mag_GetData(void);
void FS_Mag_DataReady_Callback(void);

#endif /* MAG_H_ */
