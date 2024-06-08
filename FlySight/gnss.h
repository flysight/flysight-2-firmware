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

#ifndef GNSS_H_
#define GNSS_H_

#include <stdbool.h>

#define GNSS_RX_BUF_LEN   2048  // Circular buffer for UART
#define GNSS_RAW_BUF_LEN  512   // Circular buffer for raw output

typedef struct
{
	uint32_t iTOW;     // GPS time of week             (ms)

	uint16_t year;     // Year                         (1999..2099)
	uint8_t  month;    // Month                        (1..12)
	uint8_t  day;      // Day of month                 (1..31)
	uint8_t  hour;     // Hour of day                  (0..23)
	uint8_t  min;      // Minute of hour               (0..59)
	uint8_t  sec;      // Second of minute             (0..59)
	int32_t  nano;     // Nanoseconds of second        (ns)

	int32_t  lon;      // Longitude                    (deg)
	int32_t  lat;      // Latitude                     (deg)
	int32_t  hMSL;     // Height above mean sea level  (mm)

	int32_t  velN;     // North velocity               (mm/s)
	int32_t  velE;     // East velocity                (mm/s)
	int32_t  velD;     // Down velocity                (mm/s)

	int32_t  speed;    // 3D speed                     (cm/s)
	int32_t  gSpeed;   // Ground speed                 (cm/s)
	int32_t  heading;  // 2D heading                   (deg)

	uint32_t tAcc;     // Time accuracy estimate       (ns)
	uint32_t hAcc;     // Horizontal accuracy estimate (mm)
	uint32_t vAcc;     // Vertical accuracy estimate   (mm)
	uint32_t sAcc;     // Speed accuracy estimate      (mm/s)

	uint8_t  gpsFix;   // GPS fix type
	uint8_t  numSV;    // Number of SVs in solution
} FS_GNSS_Data_t;

typedef struct
{
	uint32_t time;		// ms
	uint32_t towMS;     // Time pulse time of week     (ms)
	uint16_t week;      // Time pulse week number
} FS_GNSS_Time_t;

typedef struct
{
	uint32_t towMS;     // Time pulse time of week     (ms)
	uint16_t week;      // Time pulse week number
} FS_GNSS_Int_t;

typedef struct
{
	unsigned char buf[GNSS_RAW_BUF_LEN];
} FS_GNSS_Raw_t;

void FS_GNSS_Init(void);
void FS_GNSS_DeInit(void);

void FS_GNSS_Start(void);
void FS_GNSS_Stop(void);

const FS_GNSS_Data_t *FS_GNSS_GetData(void);
void FS_GNSS_DataReady_SetCallback(void (*callback)(void));

void FS_GNSS_Timepulse(void);
const FS_GNSS_Time_t *FS_GNSS_GetTime(void);
void FS_GNSS_TimeReady_SetCallback(void (*callback)(bool));

const FS_GNSS_Raw_t *FS_GNSS_GetRaw(void);
void FS_GNSS_RawReady_SetCallback(void (*callback)(void));

const FS_GNSS_Int_t *FS_GNSS_GetInt(void);
void FS_GNSS_IntReady_SetCallback(void (*callback)(void));

#endif /* GNSS_H_ */
