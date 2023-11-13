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

#include <stdbool.h>

#include "main.h"
#include "app_common.h"
#include "common.h"
#include "config.h"
#include "ff.h"
#include "log.h"
#include "state.h"
#include "stm32_seq.h"
#include "time.h"
#include "version.h"

#define LOG_TIMEOUT     5		// Write timeout

#define LOG_UPDATE_MSEC 10
#define LOG_UPDATE_RATE (LOG_UPDATE_MSEC*1000/CFG_TS_TICK_VAL)

#define BARO_COUNT	4
#define HUM_COUNT	2
#define MAG_COUNT	2
#define GNSS_COUNT	2
#define TIME_COUNT	2
#define RAW_COUNT	2
#define IMU_COUNT   133
#define VBAT_COUNT  2

static          FS_Baro_Data_t baroBuf[BARO_COUNT];	// data buffer
static          uint32_t       baroRdI;				// read index
static volatile uint32_t       baroWrI;				// write index

static          FS_Hum_Data_t humBuf[HUM_COUNT];	// data buffer
static          uint32_t      humRdI;				// read index
static volatile uint32_t      humWrI;				// write index

static          FS_Mag_Data_t magBuf[MAG_COUNT];	// data buffer
static          uint32_t      magRdI;				// read index
static volatile uint32_t      magWrI;				// write index

static          FS_GNSS_Data_t gnssBuf[GNSS_COUNT];	// data buffer
static          uint32_t       gnssRdI;				// read index
static volatile uint32_t       gnssWrI;				// write index

static          FS_GNSS_Time_t timeBuf[TIME_COUNT];	// data buffer
static          uint32_t       timeRdI;				// read index
static volatile uint32_t       timeWrI;				// write index

static          FS_GNSS_Raw_t  rawBuf[RAW_COUNT];	// data buffer
static          uint32_t       rawRdI;				// read index
static volatile uint32_t       rawWrI;				// write index

static          FS_IMU_Data_t  imuBuf[IMU_COUNT];	// data buffer
static          uint32_t       imuRdI;				// read index
static volatile uint32_t       imuWrI;				// write index

static          FS_VBAT_Data_t vbatBuf[VBAT_COUNT];	// data buffer
static          uint32_t       vbatRdI;				// read index
static volatile uint32_t       vbatWrI;				// write index

static FIL gnssFile;
static FIL sensorFile;
static FIL rawFile;

static uint8_t timer_id;

static bool validDateTime;
static FS_GNSS_Data_t saved_data;

typedef enum
{
	FS_LOG_SENSOR_NONE,
	FS_LOG_SENSOR_BARO,
	FS_LOG_SENSOR_HUM,
	FS_LOG_SENSOR_MAG,
	FS_LOG_SENSOR_TIME,
	FS_LOG_SENSOR_IMU,
	FS_LOG_SENSOR_VBAT
} FS_Log_SensorType_t ;

static void FS_Log_Timer(void)
{
	// Call update task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_LOG_UPDATE_ID, CFG_SCH_PRIO_1);
}

static FS_Log_SensorType_t FS_Log_GetNextSensor(void)
{
	FS_Log_SensorType_t nextType = FS_LOG_SENSOR_NONE;
	uint32_t nextTime = (uint32_t) (-1);

	#define HANDLE_SENSOR(rdI, wrI, buf, len, type)		\
		if (rdI != wrI) {								\
			const uint32_t time = buf[rdI % len].time;	\
			if (time < nextTime) {						\
				nextType = type;						\
				nextTime = time;						\
			}											\
		}

	HANDLE_SENSOR(baroRdI, baroWrI, baroBuf, BARO_COUNT, FS_LOG_SENSOR_BARO);
	HANDLE_SENSOR(humRdI,  humWrI,  humBuf,  HUM_COUNT,  FS_LOG_SENSOR_HUM);
	HANDLE_SENSOR(magRdI,  magWrI,  magBuf,  MAG_COUNT,  FS_LOG_SENSOR_MAG);
	HANDLE_SENSOR(timeRdI, timeWrI, timeBuf, TIME_COUNT, FS_LOG_SENSOR_TIME);
	HANDLE_SENSOR(imuRdI,  imuWrI,  imuBuf,  IMU_COUNT,  FS_LOG_SENSOR_IMU);
	HANDLE_SENSOR(vbatRdI, vbatWrI, vbatBuf, VBAT_COUNT, FS_LOG_SENSOR_VBAT);

	return nextType;
}

void FS_Log_UpdateHum(void)
{
	char row[150];

	// Get current data point
	FS_Hum_Data_t *data = &humBuf[humRdI % HUM_COUNT];

	// Write to disk
	char *ptr = row + sizeof(row);
	*(--ptr) = 0;

	ptr = writeInt32ToBuf(ptr, data->temperature, 1, 1, '\n');
	ptr = writeInt32ToBuf(ptr, data->humidity,    1, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->time,        3, 1, ',');
	*(--ptr) = ',';
	*(--ptr) = 'M';
	*(--ptr) = 'U';
	*(--ptr) = 'H';
	*(--ptr) = '$';

	f_puts(ptr, &sensorFile);

	// Increment read index
	++humRdI;
}

void FS_Log_UpdateBaro(void)
{
	char row[150];

	// Get current data point
	FS_Baro_Data_t *data = &baroBuf[baroRdI % BARO_COUNT];

	// Write to disk
	char *ptr = row + sizeof(row);
	*(--ptr) = 0;

	ptr = writeInt32ToBuf(ptr, data->temperature, 2, 1, '\n');
	ptr = writeInt32ToBuf(ptr, data->pressure,    2, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->time,        3, 1, ',');
	*(--ptr) = ',';
	*(--ptr) = 'O';
	*(--ptr) = 'R';
	*(--ptr) = 'A';
	*(--ptr) = 'B';
	*(--ptr) = '$';

	f_puts(ptr, &sensorFile);

	// Increment read index
	++baroRdI;
}

void FS_Log_UpdateMag(void)
{
	char row[150];

	// Get current data point
	FS_Mag_Data_t *data = &magBuf[magRdI % MAG_COUNT];

	// Write to disk
	char *ptr = row + sizeof(row);
	*(--ptr) = 0;

	ptr = writeInt32ToBuf(ptr, data->temperature, 1, 1, '\n');
	ptr = writeInt32ToBuf(ptr, data->z,           3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->y,           3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->x,           3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->time,        3, 1, ',');
	*(--ptr) = ',';
	*(--ptr) = 'G';
	*(--ptr) = 'A';
	*(--ptr) = 'M';
	*(--ptr) = '$';

	f_puts(ptr, &sensorFile);

	// Increment read index
	++magRdI;
}

void FS_Log_UpdateGNSS(void)
{
	char row[150];

	// Get current data point
	FS_GNSS_Data_t *data = &gnssBuf[gnssRdI % GNSS_COUNT];

	if ((data->gpsFix >= 3) && (!validDateTime))
	{
		// Remember date and time
		memcpy(&saved_data, data, sizeof(FS_GNSS_Data_t));
		validDateTime = true;
	}

	// Write to disk
	char *ptr = row + sizeof(row);
	*(--ptr) = 0;

	ptr = writeInt32ToBuf(ptr, data->numSV,   0, 0, '\n');
	ptr = writeInt32ToBuf(ptr, data->sAcc,    2, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->vAcc,    3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->hAcc,    3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->velD,    2, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->velE,    2, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->velN,    2, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->hMSL,    3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->lon,     7, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->lat,     7, 1, ',');
	*(--ptr) = ',';
	ptr = writeInt32ToBuf(ptr, (data->nano + 500000) / 1000000, 3, 0, 'Z');
	ptr = writeInt32ToBuf(ptr, data->sec,     2, 0, '.');
	ptr = writeInt32ToBuf(ptr, data->min,     2, 0, ':');
	ptr = writeInt32ToBuf(ptr, data->hour,    2, 0, ':');
	ptr = writeInt32ToBuf(ptr, data->day,     2, 0, 'T');
	ptr = writeInt32ToBuf(ptr, data->month,   2, 0, '-');
	ptr = writeInt32ToBuf(ptr, data->year,    4, 0, '-');
	*(--ptr) = ',';
	*(--ptr) = 'S';
	*(--ptr) = 'S';
	*(--ptr) = 'N';
	*(--ptr) = 'G';
	*(--ptr) = '$';

	f_puts(ptr, &gnssFile);

	// Increment read index
	++gnssRdI;
}

void FS_Log_UpdateTime(void)
{
	char row[150];

	// Get current data point
	FS_GNSS_Time_t *time = &timeBuf[timeRdI % TIME_COUNT];

	// Write to disk
	char *ptr = row + sizeof(row);
	*(--ptr) = 0;

	ptr = writeInt32ToBuf(ptr, time->week,        0, 0, '\n');
	ptr = writeInt32ToBuf(ptr, time->towMS,       3, 1, ',');
	ptr = writeInt32ToBuf(ptr, time->time,        3, 1, ',');
	*(--ptr) = ',';
	*(--ptr) = 'E';
	*(--ptr) = 'M';
	*(--ptr) = 'I';
	*(--ptr) = 'T';
	*(--ptr) = '$';

	f_puts(ptr, &sensorFile);

	// Increment read index
	++timeRdI;
}

void FS_Log_UpdateRaw(void)
{
	UINT bw;

	// Get current data point
	FS_GNSS_Raw_t *data = &rawBuf[rawRdI % RAW_COUNT];

	// Write to disk
	f_write(&rawFile, data, sizeof(FS_GNSS_Raw_t), &bw);

	// Increment read index
	++rawRdI;
}

void FS_Log_UpdateIMU(void)
{
	char row[150];

	// Get current data point
	FS_IMU_Data_t *data = &imuBuf[imuRdI % IMU_COUNT];

	// Write to disk
	char *ptr = row + sizeof(row);
	*(--ptr) = 0;

	ptr = writeInt32ToBuf(ptr, data->temperature, 2, 1, '\n');
	ptr = writeInt32ToBuf(ptr, data->az,          5, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->ay,          5, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->ax,          5, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->wz,          3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->wy,          3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->wx,          3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->time,        3, 1, ',');
	*(--ptr) = ',';
	*(--ptr) = 'U';
	*(--ptr) = 'M';
	*(--ptr) = 'I';
	*(--ptr) = '$';

	f_puts(ptr, &sensorFile);

	// Increment read index
	++imuRdI;
}

void FS_Log_UpdateVBAT(void)
{
	char row[150];

	// Get current data point
	FS_VBAT_Data_t *data = &vbatBuf[vbatRdI % VBAT_COUNT];

	// Write to disk
	char *ptr = row + sizeof(row);
	*(--ptr) = 0;

	ptr = writeInt32ToBuf(ptr, data->voltage, 3, 1, '\n');
	ptr = writeInt32ToBuf(ptr, data->time,    3, 1, ',');
	*(--ptr) = ',';
	*(--ptr) = 'T';
	*(--ptr) = 'A';
	*(--ptr) = 'B';
	*(--ptr) = 'V';
	*(--ptr) = '$';

	f_puts(ptr, &sensorFile);

	// Increment read index
	++vbatRdI;
}

static void FS_Log_Update(void)
{
	const uint32_t ms = HAL_GetTick();
	const uint32_t gnssIndex = gnssWrI;
	const uint32_t rawIndex = rawWrI;
	FS_Log_SensorType_t next;

	// Write all raw GNSS output
	while (FS_Config_Get()->enable_raw &&
			(rawRdI != rawIndex))
	{
		FS_Log_UpdateRaw();
	}

	// Write all GNSS log entries
	while (gnssRdI != gnssIndex)
	{
		FS_Log_UpdateGNSS();
	}

	// Write as many sensor log entries as we can
	while ((next = FS_Log_GetNextSensor()) != FS_LOG_SENSOR_NONE)
	{
		if (HAL_GetTick() >= ms + LOG_TIMEOUT)
			break;

		switch (next)
		{
		case FS_LOG_SENSOR_BARO:
			FS_Log_UpdateBaro();
			break;
		case FS_LOG_SENSOR_HUM:
			FS_Log_UpdateHum();
			break;
		case FS_LOG_SENSOR_MAG:
			FS_Log_UpdateMag();
			break;
		case FS_LOG_SENSOR_TIME:
			FS_Log_UpdateTime();
			break;
		case FS_LOG_SENSOR_IMU:
			FS_Log_UpdateIMU();
			break;
		case FS_LOG_SENSOR_VBAT:
			FS_Log_UpdateVBAT();
			break;
		case FS_LOG_SENSOR_NONE:
			break;		// should never be called
		}
	}
}

static void FS_Log_WriteHex(FIL *file, const uint32_t *data, uint32_t count)
{
	uint32_t i;

	for (i = 0; i < count; ++i)
	{
		f_printf(file, "%08x", data[i]);
	}
}

static void FS_Log_WriteCommonHeader(FIL *file)
{
	// Write file format
	f_printf(file, "$FLYS,1\n");

	// Write firmware version
    f_printf(file, "$VAR,FIRMWARE_VER,%s\n", GIT_TAG);

	// Write device ID
	f_printf(file, "$VAR,DEVICE_ID,");
	FS_Log_WriteHex(file, FS_State_Get()->device_id, 3);
	f_printf(file, "\n");

	// Write session ID
	f_printf(file, "$VAR,SESSION_ID,");
	FS_Log_WriteHex(file, FS_State_Get()->session_id, 3);
	f_printf(file, "\n");
}

void FS_Log_Init(uint32_t temp_folder)
{
	char filename[50];

	// Reset state
	baroRdI = 0;
	baroWrI = 0;
	humRdI = 0;
	humWrI = 0;
	magRdI = 0;
	magWrI = 0;
	gnssRdI = 0;
	gnssWrI = 0;
	timeRdI = 0;
	timeWrI = 0;
	rawRdI = 0;
	rawWrI = 0;
	imuRdI = 0;
	imuWrI = 0;
	vbatRdI = 0;
	vbatWrI = 0;
	validDateTime = false;

	// Create temporary folder
	f_mkdir("/temp");
	sprintf(filename, "/temp/%04lu", temp_folder);
	f_mkdir(filename);

	// Open GNSS log file
	sprintf(filename, "/temp/%04lu/track.csv", temp_folder);
	if (f_open(&gnssFile, filename, FA_WRITE|FA_CREATE_ALWAYS) != FR_OK)
	{
		Error_Handler();
	}

	FS_Log_WriteCommonHeader(&gnssFile);
	f_printf(&gnssFile, "$COL,GNSS,time,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,numSV\n");
	f_printf(&gnssFile, "$UNIT,GNSS,,deg,deg,m,m/s,m/s,m/s,m,m,m/s,\n");
	f_printf(&gnssFile, "$DATA\n");

	if (FS_Config_Get()->enable_raw)
	{
		// Open raw GNSS file
		sprintf(filename, "/temp/%04lu/raw.ubx", temp_folder);
		if (f_open(&rawFile, filename, FA_WRITE|FA_CREATE_ALWAYS) != FR_OK)
		{
			Error_Handler();
		}
	}

	// Open sensor log file
	sprintf(filename, "/temp/%04lu/sensor.csv", temp_folder);
	if (f_open(&sensorFile, filename, FA_WRITE|FA_CREATE_ALWAYS) != FR_OK)
	{
		Error_Handler();
	}

	FS_Log_WriteCommonHeader(&sensorFile);
	f_printf(&sensorFile, "$COL,BARO,time,pressure,temperature\n");
	f_printf(&sensorFile, "$UNIT,BARO,s,Pa,deg C\n");
	f_printf(&sensorFile, "$COL,HUM,time,humidity,temperature\n");
	f_printf(&sensorFile, "$UNIT,HUM,s,percent,deg C\n");
	f_printf(&sensorFile, "$COL,MAG,time,x,y,z,temperature\n");
	f_printf(&sensorFile, "$UNIT,MAG,s,gauss,gauss,gauss,deg C\n");
	f_printf(&sensorFile, "$COL,IMU,time,wx,wy,wz,ax,ay,az,temperature\n");
	f_printf(&sensorFile, "$UNIT,IMU,s,deg/s,deg/s,deg/s,g,g,g,deg C\n");
	f_printf(&sensorFile, "$COL,TIME,time,tow,week\n");
	f_printf(&sensorFile, "$UNIT,TIME,s,s,\n");
	f_printf(&sensorFile, "$COL,VBAT,time,voltage\n");
	f_printf(&sensorFile, "$UNIT,VBAT,s,volt\n");
	f_printf(&sensorFile, "$DATA\n");

	// Initialize update task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_LOG_UPDATE_ID, UTIL_SEQ_RFU, FS_Log_Update);

	// Initialize update timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_Repeated, FS_Log_Timer);
	HW_TS_Start(timer_id, LOG_UPDATE_RATE);
}

static void FS_Log_AdjustDateTime(uint16_t *year, uint8_t *month, uint8_t *day,
		uint8_t *hour, uint8_t *min, uint8_t *sec)
{
	uint32_t timestamp;

	// Convert UTC date/time to a single value, offset it, and convert back
	timestamp = mk_gmtime(*year, *month, *day, *hour, *min, *sec);
	timestamp += FS_Config_Get()->tz_offset;
	gmtime_r(timestamp, year, month, day, hour, min, sec);
}

void FS_Log_DeInit(uint32_t temp_folder)
{
	uint16_t year;
	uint8_t  month;
	uint8_t  day;
	uint8_t  hour;
	uint8_t  min;
	uint8_t  sec;

	char date[15], time[15];
	char oldPath[50], newPath[50];
    FILINFO fno;

	// Delete timer
	HW_TS_Delete(timer_id);

	// Close files
	if (FS_Config_Get()->enable_raw)
	{
		f_close(&rawFile);
	}
	f_close(&gnssFile);
	f_close(&sensorFile);

	if (validDateTime)
	{
		// Get date/time
		year = saved_data.year;
		month = saved_data.month;
		day = saved_data.day;
		hour = saved_data.hour;
		min = saved_data.min;
		sec = saved_data.sec;

		// Adjust using timezone
		FS_Log_AdjustDateTime(&year, &month, &day, &hour, &min, &sec);

		// Set timestamp in FILINFO structure
		fno.fdate = (WORD)(((year - 1980) * 512U) | month * 32U | day);
		fno.ftime = (WORD)(hour * 2048U | min * 32U | sec / 2U);

		// Update timestamps
		sprintf(oldPath, "/temp/%04lu", temp_folder);
		f_utime(oldPath, &fno);

		sprintf(oldPath, "/temp/%04lu/track.csv", temp_folder);
		f_utime(oldPath, &fno);

		if (FS_Config_Get()->enable_raw)
		{
			sprintf(oldPath, "/temp/%04lu/raw.ubx", temp_folder);
			f_utime(oldPath, &fno);
		}

		sprintf(oldPath, "/temp/%04lu/sensor.csv", temp_folder);
		f_utime(oldPath, &fno);

		// Format date and time
		sprintf(date, "%02d-%02d-%02d", year % 100, month, day);
		sprintf(time, "%02d-%02d-%02d", hour, min, sec);

		sprintf(newPath, "/%s", date);
		if (f_stat(newPath, 0) != FR_OK)
		{
			// Create new folder
			f_mkdir(newPath);

			// Update timestamp
			f_utime(newPath, &fno);
		}

		// Move temporary folder
		sprintf(oldPath, "/temp/%04lu", temp_folder);
		sprintf(newPath, "/%s/%s", date, time);
		f_rename(oldPath, newPath);
	}
}

void FS_Log_WriteBaroData(const FS_Baro_Data_t *current)
{
	// Copy to circular buffer
	FS_Baro_Data_t *saved = &baroBuf[baroWrI % BARO_COUNT];
	memcpy(saved, current, sizeof(FS_Baro_Data_t));

	// Increment write index
	++baroWrI;
}

void FS_Log_WriteHumData(const FS_Hum_Data_t *current)
{
	// Copy to circular buffer
	FS_Hum_Data_t *saved = &humBuf[humWrI % HUM_COUNT];
	memcpy(saved, current, sizeof(FS_Hum_Data_t));

	// Increment write index
	++humWrI;
}

void FS_Log_WriteMagData(const FS_Mag_Data_t *current)
{
	// Copy to circular buffer
	FS_Mag_Data_t *saved = &magBuf[magWrI % MAG_COUNT];
	memcpy(saved, current, sizeof(FS_Mag_Data_t));

	// Increment write index
	++magWrI;
}

void FS_Log_WriteGNSSData(const FS_GNSS_Data_t *current)
{
	if (current->gpsFix == 3)
	{
		// Copy to circular buffer
		FS_GNSS_Data_t *saved = &gnssBuf[gnssWrI % GNSS_COUNT];
		memcpy(saved, current, sizeof(FS_GNSS_Data_t));

		// Increment write index
		++gnssWrI;
	}
}

void FS_Log_WriteGNSSTime(const FS_GNSS_Time_t *current)
{
	// Copy to circular buffer
	FS_GNSS_Time_t *saved = &timeBuf[timeWrI % TIME_COUNT];
	memcpy(saved, current, sizeof(FS_GNSS_Time_t));

	// Increment write index
	++timeWrI;
}

void FS_Log_WriteGNSSRaw(const FS_GNSS_Raw_t *current)
{
	if (FS_Config_Get()->enable_raw)
	{
		// Copy to circular buffer
		FS_GNSS_Raw_t *saved = &rawBuf[rawWrI % RAW_COUNT];
		memcpy(saved, current, sizeof(FS_GNSS_Raw_t));

		// Increment write index
		++rawWrI;
	}
}

void FS_Log_WriteIMUData(const FS_IMU_Data_t *current)
{
	// Copy to circular buffer
	FS_IMU_Data_t *saved = &imuBuf[imuWrI % IMU_COUNT];
	memcpy(saved, current, sizeof(FS_IMU_Data_t));

	// Increment write index
	++imuWrI;
}

void FS_Log_WriteVBATData(const FS_VBAT_Data_t *current)
{
	// Copy to circular buffer
	FS_VBAT_Data_t *saved = &vbatBuf[vbatWrI % VBAT_COUNT];
	memcpy(saved, current, sizeof(FS_VBAT_Data_t));

	// Increment write index
	++vbatWrI;
}
