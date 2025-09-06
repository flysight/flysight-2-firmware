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
#include <stdarg.h>

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

#define LOG_TIMEOUT     50  // Write timeout

#define LOG_UPDATE_MSEC 50
#define LOG_UPDATE_RATE (LOG_UPDATE_MSEC*1000/CFG_TS_TICK_VAL)

#define BARO_COUNT  30
#define HUM_COUNT   3
#define MAG_COUNT   15
#define GNSS_COUNT  6
#define TIME_COUNT  2
#define RAW_COUNT   5
#define IMU_COUNT   667
#define VBAT_COUNT  2

static          FS_Baro_Data_t baroBuf[BARO_COUNT]; // data buffer
static          uint32_t       baroRdI;             // read index
static volatile uint32_t       baroWrI;             // write index
static          uint32_t       baroUsed;            // buffer used

static          FS_Hum_Data_t  humBuf[HUM_COUNT];   // data buffer
static          uint32_t       humRdI;              // read index
static volatile uint32_t       humWrI;              // write index
static          uint32_t       humUsed;             // buffer used

static          FS_Mag_Data_t  magBuf[MAG_COUNT];   // data buffer
static          uint32_t       magRdI;              // read index
static volatile uint32_t       magWrI;              // write index
static          uint32_t       magUsed;             // buffer used

static          FS_GNSS_Data_t gnssBuf[GNSS_COUNT]; // data buffer
static          uint32_t       gnssRdI;             // read index
static volatile uint32_t       gnssWrI;             // write index
static          uint32_t       gnssUsed;            // buffer used

static          FS_GNSS_Time_t timeBuf[TIME_COUNT]; // data buffer
static          uint32_t       timeRdI;             // read index
static volatile uint32_t       timeWrI;             // write index
static          uint32_t       timeUsed;            // buffer used

static          FS_GNSS_Raw_t  rawBuf[RAW_COUNT];   // data buffer
static          uint32_t       rawRdI;              // read index
static volatile uint32_t       rawWrI;              // write index
static          uint32_t       rawUsed;             // buffer used

static          FS_IMU_Data_t  imuBuf[IMU_COUNT];   // data buffer
static          uint32_t       imuRdI;              // read index
static volatile uint32_t       imuWrI;              // write index
static          uint32_t       imuUsed;             // buffer used

static          FS_VBAT_Data_t vbatBuf[VBAT_COUNT]; // data buffer
static          uint32_t       vbatRdI;             // read index
static volatile uint32_t       vbatWrI;             // write index
static          uint32_t       vbatUsed;            // buffer used

static FIL gnssFile;
static FIL sensorFile;
static FIL rawFile;
static FIL eventFile;

static uint8_t timer_id;

static bool validDateTime;
static FS_GNSS_Data_t saved_data;

static uint32_t updateCount;
static uint32_t updateTotalTime;
static uint32_t updateMaxTime;
static uint32_t updateLastCall;
static uint32_t updateMaxInterval;

static uint32_t syncCount;
static uint32_t syncTotalTime;
static uint32_t syncMaxTime;
static uint32_t syncLastCall;
static uint32_t syncMaxInterval;

static TCHAR path[256];

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

static uint8_t enable_flags;

typedef enum
{
    LOG_STATE_UNINITIALIZED,
    LOG_STATE_ACTIVE,
    LOG_STATE_FAILED
} FS_Log_State_t;

static FS_Log_State_t logState = LOG_STATE_UNINITIALIZED;

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
	UINT bw;

	if (!(enable_flags & FS_LOG_ENABLE_SENSOR))
	{
		Error_Handler();
	}

	// Get current data point
	FS_Hum_Data_t *data = &humBuf[humRdI % HUM_COUNT];

	// Write to disk
	char *ptr = row + sizeof(row);

	*(--ptr) = '\n';
	ptr = writeInt32ToBuf(ptr, data->temperature, 1, 1, '\r');
	ptr = writeInt32ToBuf(ptr, data->humidity,    1, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->time,        3, 1, ',');
	*(--ptr) = ',';
	*(--ptr) = 'M';
	*(--ptr) = 'U';
	*(--ptr) = 'H';
	*(--ptr) = '$';

	f_write(&sensorFile, ptr, row + sizeof(row) - ptr, &bw);

	// Increment read index
	++humRdI;
}

void FS_Log_UpdateBaro(void)
{
	char row[150];
	UINT bw;

	if (!(enable_flags & FS_LOG_ENABLE_SENSOR))
	{
		Error_Handler();
	}

	// Get current data point
	FS_Baro_Data_t *data = &baroBuf[baroRdI % BARO_COUNT];

	// Write to disk
	char *ptr = row + sizeof(row);

	*(--ptr) = '\n';
	ptr = writeInt32ToBuf(ptr, data->temperature, 2, 1, '\r');
	ptr = writeInt32ToBuf(ptr, data->pressure,    2, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->time,        3, 1, ',');
	*(--ptr) = ',';
	*(--ptr) = 'O';
	*(--ptr) = 'R';
	*(--ptr) = 'A';
	*(--ptr) = 'B';
	*(--ptr) = '$';

	f_write(&sensorFile, ptr, row + sizeof(row) - ptr, &bw);

	// Increment read index
	++baroRdI;
}

void FS_Log_UpdateMag(void)
{
	char row[150];
	UINT bw;

	if (!(enable_flags & FS_LOG_ENABLE_SENSOR))
	{
		Error_Handler();
	}

	// Get current data point
	FS_Mag_Data_t *data = &magBuf[magRdI % MAG_COUNT];

	// Write to disk
	char *ptr = row + sizeof(row);

	*(--ptr) = '\n';
	ptr = writeInt32ToBuf(ptr, data->temperature, 1, 1, '\r');
	ptr = writeInt32ToBuf(ptr, data->z,           3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->y,           3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->x,           3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->time,        3, 1, ',');
	*(--ptr) = ',';
	*(--ptr) = 'G';
	*(--ptr) = 'A';
	*(--ptr) = 'M';
	*(--ptr) = '$';

	f_write(&sensorFile, ptr, row + sizeof(row) - ptr, &bw);

	// Increment read index
	++magRdI;
}

// Helper function to normalize nanoseconds and adjust time accordingly
void normalizeGNSSTime(const FS_GNSS_Data_t *data,
                       uint16_t *norm_year, uint8_t *norm_month, uint8_t *norm_day,
                       uint8_t *norm_hour, uint8_t *norm_min, uint8_t *norm_sec,
                       int32_t *norm_millis)
{
	// Convert original time to epoch seconds
	uint32_t epoch_seconds = mk_gmtime(data->year, data->month, data->day,
	                                   data->hour, data->min, data->sec);

	// Convert nanoseconds to milliseconds (rounded)
	if (data->nano >= 0) {
		*norm_millis = (data->nano + 500000) / 1000000;
	} else {
		*norm_millis = (data->nano - 499999) / 1000000;
	}

	// Handle negative milliseconds by borrowing from seconds
	if (*norm_millis >= 1000) {
		*norm_millis -= 1000;
		++epoch_seconds;
	}
	else if (*norm_millis < 0) {
		*norm_millis += 1000;
		--epoch_seconds;
	}

	// Convert back to date/time components
	gmtime_r(epoch_seconds, norm_year, norm_month, norm_day,
	         norm_hour, norm_min, norm_sec);
}

// Modified logging function
void FS_Log_UpdateGNSS(void)
{
	char row[150];
	UINT bw;

	if (!(enable_flags & FS_LOG_ENABLE_GNSS))
	{
		Error_Handler();
	}

	// Get current data point
	FS_GNSS_Data_t *data = &gnssBuf[gnssRdI % GNSS_COUNT];

	// Normalize the time components
	uint16_t norm_year;
	uint8_t norm_month, norm_day, norm_hour, norm_min, norm_sec;
	int32_t norm_millis;

	normalizeGNSSTime(data, &norm_year, &norm_month, &norm_day,
	&norm_hour, &norm_min, &norm_sec, &norm_millis);

	// Write to disk using normalized values
	char *ptr = row + sizeof(row);

	*(--ptr) = '\n';
	ptr = writeInt32ToBuf(ptr, data->numSV,   0, 0, '\r');
	ptr = writeInt32ToBuf(ptr, data->sAcc,    3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->vAcc,    3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->hAcc,    3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->velD,    3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->velE,    3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->velN,    3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->hMSL,    3, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->lon,     7, 1, ',');
	ptr = writeInt32ToBuf(ptr, data->lat,     7, 1, ',');
	*(--ptr) = ',';
	ptr = writeInt32ToBuf(ptr, norm_millis,   3, 0, 'Z');
	ptr = writeInt32ToBuf(ptr, norm_sec,      2, 0, '.');
	ptr = writeInt32ToBuf(ptr, norm_min,      2, 0, ':');
	ptr = writeInt32ToBuf(ptr, norm_hour,     2, 0, ':');
	ptr = writeInt32ToBuf(ptr, norm_day,      2, 0, 'T');
	ptr = writeInt32ToBuf(ptr, norm_month,    2, 0, '-');
	ptr = writeInt32ToBuf(ptr, norm_year,     4, 0, '-');
	*(--ptr) = ',';
	*(--ptr) = 'S';
	*(--ptr) = 'S';
	*(--ptr) = 'N';
	*(--ptr) = 'G';
	*(--ptr) = '$';

	f_write(&gnssFile, ptr, row + sizeof(row) - ptr, &bw);

	// Increment read index
	++gnssRdI;
}

void FS_Log_UpdateTime(void)
{
	char row[150];
	UINT bw;

	if (!(enable_flags & FS_LOG_ENABLE_SENSOR))
	{
		Error_Handler();
	}

	// Get current data point
	FS_GNSS_Time_t *time = &timeBuf[timeRdI % TIME_COUNT];

	// Write to disk
	char *ptr = row + sizeof(row);

	*(--ptr) = '\n';
	ptr = writeInt32ToBuf(ptr, time->week,        0, 0, '\r');
	ptr = writeInt32ToBuf(ptr, time->towMS,       3, 1, ',');
	ptr = writeInt32ToBuf(ptr, time->time,        3, 1, ',');
	*(--ptr) = ',';
	*(--ptr) = 'E';
	*(--ptr) = 'M';
	*(--ptr) = 'I';
	*(--ptr) = 'T';
	*(--ptr) = '$';

	f_write(&sensorFile, ptr, row + sizeof(row) - ptr, &bw);

	// Increment read index
	++timeRdI;
}

void FS_Log_UpdateRaw(void)
{
	UINT bw;

	if (!(enable_flags & FS_LOG_ENABLE_RAW))
	{
		Error_Handler();
	}

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
	UINT bw;

	if (!(enable_flags & FS_LOG_ENABLE_SENSOR))
	{
		Error_Handler();
	}

	// Get current data point
	FS_IMU_Data_t *data = &imuBuf[imuRdI % IMU_COUNT];

	// Write to disk
	char *ptr = row + sizeof(row);

	*(--ptr) = '\n';
	ptr = writeInt32ToBuf(ptr, data->temperature, 2, 1, '\r');
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

	f_write(&sensorFile, ptr, row + sizeof(row) - ptr, &bw);

	// Increment read index
	++imuRdI;
}

void FS_Log_UpdateVBAT(void)
{
	char row[150];
	UINT bw;

	if (!(enable_flags & FS_LOG_ENABLE_SENSOR))
	{
		Error_Handler();
	}

	// Get current data point
	FS_VBAT_Data_t *data = &vbatBuf[vbatRdI % VBAT_COUNT];

	// Write to disk
	char *ptr = row + sizeof(row);

	*(--ptr) = '\n';
	ptr = writeInt32ToBuf(ptr, data->voltage, 3, 1, '\r');
	ptr = writeInt32ToBuf(ptr, data->time,    3, 1, ',');
	*(--ptr) = ',';
	*(--ptr) = 'T';
	*(--ptr) = 'A';
	*(--ptr) = 'B';
	*(--ptr) = 'V';
	*(--ptr) = '$';

	f_write(&sensorFile, ptr, row + sizeof(row) - ptr, &bw);

	// Increment read index
	++vbatRdI;
}

static void FS_Log_Update(void)
{
	uint32_t msStart, msEnd;
	const uint32_t gnssIndex = gnssWrI;
	const uint32_t rawIndex = rawWrI;
	FS_Log_SensorType_t next;

	msStart = HAL_GetTick();

	if (updateLastCall != 0)
	{
		updateMaxInterval = MAX(updateMaxInterval, msStart - updateLastCall);
	}
	updateLastCall = msStart;

	// Write raw GNSS output
	while ((HAL_GetTick() < msStart + LOG_TIMEOUT) &&
			(rawRdI != rawIndex))
	{
		FS_Log_UpdateRaw();
	}

	// Write GNSS log entries
	while ((HAL_GetTick() < msStart + LOG_TIMEOUT) &&
			(gnssRdI != gnssIndex))
	{
		FS_Log_UpdateGNSS();
	}

	// Write sensor log entries
	while ((HAL_GetTick() < msStart + LOG_TIMEOUT) &&
			((next = FS_Log_GetNextSensor()) != FS_LOG_SENSOR_NONE))
	{
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

	++updateCount;

	if (updateCount % 6 == 0)
	{
		// Call sync task
		UTIL_SEQ_SetTask(1<<CFG_TASK_FS_LOG_SYNC_ID, CFG_SCH_PRIO_1);
	}

	msEnd = HAL_GetTick();
	updateTotalTime += msEnd - msStart;
	updateMaxTime = MAX(updateMaxTime, msEnd - msStart);
}

static void FS_Log_Sync(void)
{
	uint32_t msStart, msEnd;

	msStart = HAL_GetTick();

	if (syncLastCall != 0)
	{
		syncMaxInterval = MAX(syncMaxInterval, msStart - syncLastCall);
	}
	syncLastCall = msStart;

	switch (syncCount % 3)
	{
	case 0:
		if (enable_flags & FS_LOG_ENABLE_GNSS)
		{
			f_sync(&gnssFile);
		}
		break;
	case 1:
		if (enable_flags & FS_LOG_ENABLE_RAW)
		{
			f_sync(&rawFile);
		}
		break;
	case 2:
		if (enable_flags & FS_LOG_ENABLE_SENSOR)
		{
			f_sync(&sensorFile);
		}
		break;
	}

	++syncCount;

	msEnd = HAL_GetTick();
	syncTotalTime += msEnd - msStart;
	syncMaxTime = MAX(syncMaxTime, msEnd - msStart);
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

static FRESULT delete_node (
		TCHAR* path,    /* Path name buffer with the sub-directory to delete */
		UINT sz_buff,   /* Size of path name buffer (items) */
		FILINFO* fno    /* Name read buffer */
		)
{
	UINT i, j;
	FRESULT fr;
	DIR dir;

	fr = f_opendir(&dir, path); /* Open the sub-directory to make it empty */
	if (fr != FR_OK) return fr;

	for (i = 0; path[i]; i++) ; /* Get current path length */
	path[i++] = _T('/');

	for (;;) {
		fr = f_readdir(&dir, fno);  /* Get a directory item */
		if (fr != FR_OK || !fno->fname[0]) break;   /* End of directory? */
		j = 0;
		do {    /* Make a path name */
			if (i + j >= sz_buff) { /* Buffer over flow? */
				fr = 100; break;    /* Fails with 100 when buffer overflow */
			}
			path[i + j] = fno->fname[j];
		} while (fno->fname[j++]);
		if (fno->fattrib & AM_DIR) {    /* Item is a sub-directory */
			fr = delete_node(path, sz_buff, fno);
		} else {                        /* Item is a file */
			fr = f_unlink(path);
		}
		if (fr != FR_OK) break;
	}

	path[--i] = 0;  /* Restore the path name */
	f_closedir(&dir);

	if (fr == FR_OK) fr = f_unlink(path);  /* Delete the empty sub-directory */
	return fr;
}

HAL_StatusTypeDef FS_Log_Init(uint32_t temp_folder, uint8_t flags)
{
	FILINFO fno;

	// Save enable flags
	enable_flags = flags;

	// Reset state
	baroRdI = 0;
	baroWrI = 0;
	baroUsed = 0;

	humRdI = 0;
	humWrI = 0;
	humUsed = 0;

	magRdI = 0;
	magWrI = 0;
	magUsed = 0;

	gnssRdI = 0;
	gnssWrI = 0;
	gnssUsed = 0;

	timeRdI = 0;
	timeWrI = 0;
	timeUsed = 0;

	rawRdI = 0;
	rawWrI = 0;
	rawUsed = 0;

	imuRdI = 0;
	imuWrI = 0;
	imuUsed = 0;

	vbatRdI = 0;
	vbatWrI = 0;
	vbatUsed = 0;

	validDateTime = false;

	updateCount = 0;
	updateTotalTime = 0;
	updateMaxTime = 0;
	updateLastCall = 0;
	updateMaxInterval = 0;

	syncCount = 0;
	syncTotalTime = 0;
	syncMaxTime = 0;
	syncLastCall = 0;
	syncMaxInterval = 0;

	// Create temporary folder
	f_mkdir("/temp");
	sprintf(path, "/temp/%04lu", temp_folder);

	// Delete temporary folder if it exists
	delete_node(path, sizeof(path) / sizeof(path[0]), &fno);

	// Create temporary folder
	f_mkdir(path);

	if (enable_flags & FS_LOG_ENABLE_GNSS)
	{
		// Open GNSS log file
		sprintf(path, "/temp/%04lu/track.csv", temp_folder);
		if (f_open(&gnssFile, path, FA_WRITE|FA_CREATE_ALWAYS) != FR_OK)
		{
			logState = LOG_STATE_FAILED;
			return HAL_ERROR;
		}

		FS_Log_WriteCommonHeader(&gnssFile);
		f_printf(&gnssFile, "$COL,GNSS,time,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,numSV\n");
		f_printf(&gnssFile, "$UNIT,GNSS,,deg,deg,m,m/s,m/s,m/s,m,m,m/s,\n");
		f_printf(&gnssFile, "$DATA\n");
		f_sync(&gnssFile);
	}

	if (enable_flags & FS_LOG_ENABLE_RAW)
	{
		// Open raw GNSS file
		sprintf(path, "/temp/%04lu/raw.ubx", temp_folder);
		if (f_open(&rawFile, path, FA_WRITE|FA_CREATE_ALWAYS) != FR_OK)
		{
			logState = LOG_STATE_FAILED;
			return HAL_ERROR;
		}
		f_sync(&rawFile);
	}

	if (enable_flags & FS_LOG_ENABLE_SENSOR)
	{
		// Open sensor log file
		sprintf(path, "/temp/%04lu/sensor.csv", temp_folder);
		if (f_open(&sensorFile, path, FA_WRITE|FA_CREATE_ALWAYS) != FR_OK)
		{
			logState = LOG_STATE_FAILED;
			return HAL_ERROR;
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
		f_sync(&sensorFile);
	}

	if (enable_flags & FS_LOG_ENABLE_EVENT)
	{
		// Open event log file
		sprintf(path, "/temp/%04lu/event.csv", temp_folder);
		if (f_open(&eventFile, path, FA_WRITE|FA_CREATE_ALWAYS) != FR_OK)
		{
			logState = LOG_STATE_FAILED;
			return HAL_ERROR;
		}

		FS_Log_WriteCommonHeader(&eventFile);
		f_printf(&eventFile, "$COL,EVNT,time,description\n");
		f_printf(&eventFile, "$UNIT,EVNT,s,\n");
		f_printf(&eventFile, "$DATA\n");
		f_sync(&eventFile);
	}

	// Initialize update task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_LOG_UPDATE_ID, UTIL_SEQ_RFU, FS_Log_Update);
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_LOG_SYNC_ID, UTIL_SEQ_RFU, FS_Log_Sync);

	// Initialize update timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_Repeated, FS_Log_Timer);
	HW_TS_Start(timer_id, LOG_UPDATE_RATE);

	logState = LOG_STATE_ACTIVE;
	return HAL_OK;
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
	char oldPath[50];
    FILINFO fno;

	if (logState == LOG_STATE_ACTIVE)
	{
		// Delete timer
		HW_TS_Delete(timer_id);
	}

	if ((logState == LOG_STATE_ACTIVE) && (enable_flags & FS_LOG_ENABLE_EVENT))
	{
		// Add event log entries for buffer info
		FS_Log_WriteEvent("----------");
		FS_Log_WriteEvent("%lu/%lu slots used in $BARO message buffer", baroUsed, BARO_COUNT);
		FS_Log_WriteEvent("%lu/%lu slots used in $HUM message buffer",  humUsed, HUM_COUNT);
		FS_Log_WriteEvent("%lu/%lu slots used in $MAG message buffer",  magUsed, MAG_COUNT);
		FS_Log_WriteEvent("%lu/%lu slots used in $GNSS message buffer", gnssUsed, GNSS_COUNT);
		FS_Log_WriteEvent("%lu/%lu slots used in $TIME message buffer", timeUsed, TIME_COUNT);
		FS_Log_WriteEvent("%lu/%lu slots used in $RAW message buffer",  rawUsed, RAW_COUNT);
		FS_Log_WriteEvent("%lu/%lu slots used in $IMU message buffer",  imuUsed, IMU_COUNT);
		FS_Log_WriteEvent("%lu/%lu slots used in $VBAT message buffer", vbatUsed, VBAT_COUNT);

		// Add event log entries for timing info
		FS_Log_WriteEvent("----------");
		FS_Log_WriteEvent("%lu ms average time spent in log update task",
				(updateCount > 0) ? (updateTotalTime / updateCount) : 0);
		FS_Log_WriteEvent("%lu ms maximum time spent in log update task", updateMaxTime);
		FS_Log_WriteEvent("%lu ms maximum time between calls to log update task", updateMaxInterval);

		FS_Log_WriteEvent("----------");
		FS_Log_WriteEvent("%lu ms average time spent in log sync task",
				(syncCount > 0) ? (syncTotalTime / syncCount) : 0);
		FS_Log_WriteEvent("%lu ms maximum time spent in log sync task", syncMaxTime);
		FS_Log_WriteEvent("%lu ms maximum time between calls to log sync task", syncMaxInterval);
	}

	// Close files
	if (enable_flags & FS_LOG_ENABLE_RAW)
	{
		f_close(&rawFile);
	}
	if (enable_flags & FS_LOG_ENABLE_GNSS)
	{
		f_close(&gnssFile);
	}
	if (enable_flags & FS_LOG_ENABLE_SENSOR)
	{
		f_close(&sensorFile);
	}
	if (enable_flags & FS_LOG_ENABLE_EVENT)
	{
		f_close(&eventFile);
	}

	if ((logState == LOG_STATE_ACTIVE) && validDateTime)
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

		// Format date and time
		sprintf(date, "%02d-%02d-%02d", year % 100, month, day);
		sprintf(time, "%02d-%02d-%02d", hour, min, sec);

		sprintf(path, "/%s", date);
		if (f_stat(path, 0) != FR_OK)
		{
			// Create new folder
			f_mkdir(path);
		}

		// Move temporary folder
		sprintf(oldPath, "/temp/%04lu", temp_folder);
		sprintf(path, "/%s/%s", date, time);

		// Delete date/time folder if it exists
		delete_node(path, sizeof(path) / sizeof(path[0]), &fno);

		// Rename temporary folder
		f_rename(oldPath, path);
	}

	// Return the logging module to its initial state for the next session.
	logState = LOG_STATE_UNINITIALIZED;
}

void FS_Log_WriteBaroData(const FS_Baro_Data_t *current)
{
	if (logState != LOG_STATE_ACTIVE) return;
	if (!(enable_flags & FS_LOG_ENABLE_SENSOR)) return;

	if (baroWrI < baroRdI + BARO_COUNT)
	{
		// Copy to circular buffer
		FS_Baro_Data_t *saved = &baroBuf[baroWrI % BARO_COUNT];
		memcpy(saved, current, sizeof(FS_Baro_Data_t));

		// Increment write index
		++baroWrI;

		// Update buffer statistics
		baroUsed = MAX(baroUsed, baroWrI - baroRdI);
	}
	else
	{
		// Update buffer statistics
		baroUsed = BARO_COUNT;
	}
}

void FS_Log_WriteHumData(const FS_Hum_Data_t *current)
{
	if (logState != LOG_STATE_ACTIVE) return;
	if (!(enable_flags & FS_LOG_ENABLE_SENSOR)) return;

	if (humWrI < humRdI + HUM_COUNT)
	{
		// Copy to circular buffer
		FS_Hum_Data_t *saved = &humBuf[humWrI % HUM_COUNT];
		memcpy(saved, current, sizeof(FS_Hum_Data_t));

		// Increment write index
		++humWrI;

		// Update buffer statistics
		humUsed = MAX(humUsed, humWrI - humRdI);
	}
	else
	{
		// Update buffer statistics
		humUsed = HUM_COUNT;
	}
}

void FS_Log_WriteMagData(const FS_Mag_Data_t *current)
{
	if (logState != LOG_STATE_ACTIVE) return;
	if (!(enable_flags & FS_LOG_ENABLE_SENSOR)) return;

	if (magWrI < magRdI + MAG_COUNT)
	{
		// Copy to circular buffer
		FS_Mag_Data_t *saved = &magBuf[magWrI % MAG_COUNT];
		memcpy(saved, current, sizeof(FS_Mag_Data_t));

		// Increment write index
		++magWrI;

		// Update buffer statistics
		magUsed = MAX(magUsed, magWrI - magRdI);
	}
	else
	{
		// Update buffer statistics
		magUsed = MAG_COUNT;
	}
}

void FS_Log_WriteGNSSData(const FS_GNSS_Data_t *current)
{
	if (logState != LOG_STATE_ACTIVE) return;
	if (!(enable_flags & FS_LOG_ENABLE_GNSS)) return;

	if (current->gpsFix == 3)
	{
		if (gnssWrI < gnssRdI + GNSS_COUNT)
		{
			// Copy to circular buffer
			FS_GNSS_Data_t *saved = &gnssBuf[gnssWrI % GNSS_COUNT];
			memcpy(saved, current, sizeof(FS_GNSS_Data_t));

			// Increment write index
			++gnssWrI;

			// Update buffer statistics
			gnssUsed = MAX(gnssUsed, gnssWrI - gnssRdI);
		}
		else
		{
			// Update buffer statistics
			gnssUsed = GNSS_COUNT;
		}
	}
}

void FS_Log_UpdatePath(const FS_GNSS_Data_t *current)
{
	if ((current->gpsFix == 3) && (!validDateTime))
	{
		// Remember date and time
		memcpy(&saved_data, current, sizeof(FS_GNSS_Data_t));
		validDateTime = true;
	}
}

void FS_Log_WriteGNSSTime(const FS_GNSS_Time_t *current)
{
	if (logState != LOG_STATE_ACTIVE) return;
	if (!(enable_flags & FS_LOG_ENABLE_SENSOR)) return;

	if (timeWrI < timeRdI + TIME_COUNT)
	{
		// Copy to circular buffer
		FS_GNSS_Time_t *saved = &timeBuf[timeWrI % TIME_COUNT];
		memcpy(saved, current, sizeof(FS_GNSS_Time_t));

		// Increment write index
		++timeWrI;

		// Update buffer statistics
		timeUsed = MAX(timeUsed, timeWrI - timeRdI);
	}
	else
	{
		// Update buffer statistics
		timeUsed = TIME_COUNT;
	}
}

void FS_Log_WriteGNSSRaw(const FS_GNSS_Raw_t *current)
{
	if (logState != LOG_STATE_ACTIVE) return;
	if (!(enable_flags & FS_LOG_ENABLE_RAW)) return;

	if (FS_Config_Get()->enable_raw)
	{
		if (rawWrI < rawRdI + RAW_COUNT)
		{
			// Copy to circular buffer
			FS_GNSS_Raw_t *saved = &rawBuf[rawWrI % RAW_COUNT];
			memcpy(saved, current, sizeof(FS_GNSS_Raw_t));

			// Increment write index
			++rawWrI;

			// Update buffer statistics
			rawUsed = MAX(rawUsed, rawWrI - rawRdI);
		}
		else
		{
			// Update buffer statistics
			rawUsed = RAW_COUNT;
		}
	}
}

void FS_Log_WriteIMUData(const FS_IMU_Data_t *current)
{
	if (logState != LOG_STATE_ACTIVE) return;
	if (!(enable_flags & FS_LOG_ENABLE_SENSOR)) return;

	if (imuWrI < imuRdI + IMU_COUNT)
	{
		// Copy to circular buffer
		FS_IMU_Data_t *saved = &imuBuf[imuWrI % IMU_COUNT];
		memcpy(saved, current, sizeof(FS_IMU_Data_t));

		// Increment write index
		++imuWrI;

		// Update buffer statistics
		imuUsed = MAX(imuUsed, imuWrI - imuRdI);
	}
	else
	{
		// Update buffer statistics
		imuUsed = IMU_COUNT;
	}
}

void FS_Log_WriteVBATData(const FS_VBAT_Data_t *current)
{
	if (logState != LOG_STATE_ACTIVE) return;
	if (!(enable_flags & FS_LOG_ENABLE_SENSOR)) return;

	if (vbatWrI < vbatRdI + VBAT_COUNT)
	{
		// Copy to circular buffer
		FS_VBAT_Data_t *saved = &vbatBuf[vbatWrI % VBAT_COUNT];
		memcpy(saved, current, sizeof(FS_VBAT_Data_t));

		// Increment write index
		++vbatWrI;

		// Update buffer statistics
		vbatUsed = MAX(vbatUsed, vbatWrI - vbatRdI);
	}
	else
	{
		// Update buffer statistics
		vbatUsed = VBAT_COUNT;
	}
}

void FS_Log_WriteEvent(const char *format, ...)
{
	const uint32_t time = HAL_GetTick();
	char row[100];
	char *ptr;
	UINT bw;

	va_list args;

	if (logState != LOG_STATE_ACTIVE) return;
	if (!(enable_flags & FS_LOG_ENABLE_EVENT)) return;

	// Write to disk
	ptr = row + sizeof(row);
	ptr = writeInt32ToBuf(ptr, time, 3, 1, ',');
	*(--ptr) = ',';
	*(--ptr) = 'T';
	*(--ptr) = 'N';
	*(--ptr) = 'V';
	*(--ptr) = 'E';
	*(--ptr) = '$';

	f_write(&eventFile, ptr, row + sizeof(row) - ptr, &bw);

	f_puts("\"", &eventFile);

	va_start(args, format);
	vsprintf(row, format, args);
	f_puts(row, &eventFile);
	va_end(args);

	f_puts("\"\n", &eventFile);

	f_sync(&eventFile);
}
