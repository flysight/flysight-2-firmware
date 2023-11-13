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

#ifndef SENSOR_H_
#define SENSOR_H_

void FS_Sensor_Start(void);
void FS_Sensor_Stop(void);

void FS_Sensor_WriteAsync(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size, void (*Callback)(HAL_StatusTypeDef));
void FS_Sensor_ReadAsync(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size, void (*Callback)(HAL_StatusTypeDef));

void FS_Sensor_TransferComplete(void);
void FS_Sensor_TransferError(void);

HAL_StatusTypeDef FS_Sensor_Write(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size);
HAL_StatusTypeDef FS_Sensor_Read(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size);

#endif /* SENSOR_H_ */
