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

#ifndef LOG_H_
#define LOG_H_

#include "baro.h"
#include "gnss.h"
#include "hum.h"
#include "imu.h"
#include "led.h"
#include "mag.h"
#include "vbat.h"

#define FS_LOG_ENABLE_GNSS   0x01
#define FS_LOG_ENABLE_SENSOR 0x02
#define FS_LOG_ENABLE_RAW    0x04
#define FS_LOG_ENABLE_EVENT  0x08
#define FS_LOG_ENABLE_ALL    0xff

HAL_StatusTypeDef FS_Log_Init(uint32_t sessionId, uint8_t flags);
void FS_Log_DeInit(uint32_t sessionId);

void FS_Log_WriteBaroData(const FS_Baro_Data_t *current);
void FS_Log_WriteHumData(const FS_Hum_Data_t *current);
void FS_Log_WriteMagData(const FS_Mag_Data_t *current);
void FS_Log_WriteGNSSData(const FS_GNSS_Data_t *current);
void FS_Log_WriteGNSSTime(const FS_GNSS_Time_t *current);
void FS_Log_WriteGNSSRaw(const FS_GNSS_Raw_t *current);
void FS_Log_WriteIMUData(const FS_IMU_Data_t *current);
void FS_Log_WriteVBATData(const FS_VBAT_Data_t *current);
void FS_Log_WriteEvent(const char *format, ...);
void FS_Log_WriteEventAsync(const char *format, ...);

void FS_Log_UpdatePath(const FS_GNSS_Data_t *current);

#endif /* LOG_H_ */
