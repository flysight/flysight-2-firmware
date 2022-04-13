/*
 * log.c
 *
 *  Created on: Sep. 29, 2020
 *      Author: Michael
 */

#include <stdbool.h>

#include "main.h"
#include "app_common.h"
#include "common.h"
#include "config.h"
#include "ff.h"
#include "log.h"
#include "stm32_seq.h"

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

static FIL gnssFile;
static FIL sensorFile;
static FIL rawFile;

static uint8_t timer_id;

static bool validDateTime;
static char date[15], time[15];

typedef enum
{
	FS_LOG_SENSOR_NONE,
	FS_LOG_SENSOR_BARO,
	FS_LOG_SENSOR_HUM,
	FS_LOG_SENSOR_MAG,
	FS_LOG_SENSOR_TIME,
	FS_LOG_SENSOR_IMU
} FS_Log_SensorType_t ;

static void FS_Log_Timer(void)
{
	// Call update task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_LOG_UPDATE_ID, CFG_SCH_PRIO_0);
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
		sprintf(date, "%02d-%02d-%02d", data->year % 100, data->month, data->day);
		sprintf(time, "%02d-%02d-%02d", data->hour, data->min, data->sec);

		validDateTime = true;
	}

	// Write to disk
	char *ptr = row + sizeof(row);
	*(--ptr) = 0;

	ptr = writeInt32ToBuf(ptr, data->numSV,   0, 0, '\n');
	ptr = writeInt32ToBuf(ptr, data->gpsFix,  0, 0, ',');
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
		case FS_LOG_SENSOR_NONE:
			break;		// should never be called
		}
	}
}

void FS_Log_Init(uint32_t sessionId)
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
	validDateTime = false;

	// Create temporary folder
	f_mkdir("/temp");
	sprintf(filename, "/temp/%04lu", sessionId);
	f_mkdir(filename);

	// Open GNSS log file
	sprintf(filename, "/temp/%04lu/gnss.csv", sessionId);
	if (f_open(&gnssFile, filename, FA_WRITE|FA_CREATE_ALWAYS) != FR_OK)
	{
		Error_Handler();
	}

	f_printf(&gnssFile, "$HEAD,time,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,gpsFix,numSV\n");
	f_printf(&gnssFile, "$HEAD,,(deg),(deg),(m),(m/s),(m/s),(m/s),(m),(m),(m/s),,\n");

	if (FS_Config_Get()->enable_raw)
	{
		// Open raw GNSS file
		sprintf(filename, "/temp/%04lu/raw.ubx", sessionId);
		if (f_open(&rawFile, filename, FA_WRITE|FA_CREATE_ALWAYS) != FR_OK)
		{
			Error_Handler();
		}
	}

	// Open sensor log file
	sprintf(filename, "/temp/%04lu/sensor.csv", sessionId);
	if (f_open(&sensorFile, filename, FA_WRITE|FA_CREATE_ALWAYS) != FR_OK)
	{
		Error_Handler();
	}

	f_printf(&sensorFile, "$HEAD,time,pressure,temperature\n");
	f_printf(&sensorFile, "$HEAD,(s),(Pa),(degrees C)\n");
	f_printf(&sensorFile, "$HEAD,time,humidity,temperature\n");
	f_printf(&sensorFile, "$HEAD,(s),(percent),(degrees C)\n");
	f_printf(&sensorFile, "$HEAD,time,x,y,z,temperature\n");
	f_printf(&sensorFile, "$HEAD,(s),(gauss),(gauss),(gauss),(degrees C)\n");
	f_printf(&sensorFile, "$HEAD,time,wx,wy,wz,ax,ay,az,temperature\n");
	f_printf(&sensorFile, "$HEAD,(s),(deg/s),(deg/s),(deg/s),(g),(g),(g),(degrees C)\n");
	f_printf(&sensorFile, "$HEAD,time,towMS,week\n");
	f_printf(&sensorFile, "$HEAD,(ms),(ms),\n");

	// Initialize update task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_LOG_UPDATE_ID, UTIL_SEQ_RFU, FS_Log_Update);

	// Initialize update timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_Repeated, FS_Log_Timer);
	HW_TS_Start(timer_id, LOG_UPDATE_RATE);
}

void FS_Log_DeInit(uint32_t sessionId)
{
	char oldPath[50], newPath[50];

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
		// Create temporary folder
		sprintf(newPath, "/%s", date);
		f_mkdir(newPath);

		sprintf(oldPath, "/temp/%04lu", sessionId);
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
	// Copy to circular buffer
	FS_GNSS_Data_t *saved = &gnssBuf[gnssWrI % GNSS_COUNT];
	memcpy(saved, current, sizeof(FS_GNSS_Data_t));

	// Increment write index
	++gnssWrI;
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
