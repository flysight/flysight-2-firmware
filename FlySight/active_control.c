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
#include "audio_control.h"
#include "baro.h"
#include "charge.h"
#include "config.h"
#include "custom_app.h"
#include "gnss.h"
#include "hum.h"
#include "imu.h"
#include "led.h"
#include "log.h"
#include "mag.h"
#include "state.h"
#include "time.h"
#include "vbat.h"

#define LED_BLINK_MSEC      900
#define LED_BLINK_TICKS     (LED_BLINK_MSEC*1000/CFG_TS_TICK_VAL)

static uint8_t led_timer_id;

static volatile bool hasFix;

static volatile enum {
	FS_CONTROL_INACTIVE = 0,
	FS_CONTROL_ACTIVE
} state = FS_CONTROL_INACTIVE;

static FS_GNSS_Time_t savedTime;

extern RTC_HandleTypeDef hrtc;

void FS_ActiveControl_DataReady_Callback(void);
void FS_ActiveControl_TimeReady_Callback(bool validTime);
void FS_ActiveControl_RawReady_Callback(void);

static void FS_ActiveControl_LED_Timer(void)
{
	// Turn on LED
	FS_LED_On();
}

void FS_ActiveControl_Init(void)
{
	// Set callback functions
	FS_GNSS_DataReady_SetCallback(FS_ActiveControl_DataReady_Callback);
	FS_GNSS_TimeReady_SetCallback(FS_ActiveControl_TimeReady_Callback);
	FS_GNSS_RawReady_SetCallback(FS_ActiveControl_RawReady_Callback);
	FS_GNSS_IntReady_SetCallback(NULL);

	// Initialize LEDs
	FS_LED_SetColour(FS_LED_GREEN);
	FS_LED_On();

	// Enable charging
	FS_Charge_SetCurrent(FS_State_Get()->charge_current);

	// Initialize LED timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &led_timer_id, hw_ts_SingleShot, FS_ActiveControl_LED_Timer);

	// Initialize state
	hasFix = false;
	state = FS_CONTROL_ACTIVE;

	// Initialize saved GNSS time
	memset(&savedTime, 0, sizeof(FS_GNSS_Time_t));
}

void FS_ActiveControl_DeInit(void)
{
	uint32_t epoch;
	uint32_t timestamp;

	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint16_t ms;

	uint32_t offset_ms;
	uint32_t ms_total;

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	// Update state
	state = FS_CONTROL_INACTIVE;

	// Delete timer
	HW_TS_Delete(led_timer_id);

	// Disable charging
	FS_Charge_SetCurrent(FS_CHARGE_DISABLE);

	// Turn off LEDs
	FS_LED_Off();

	// Clear callback functions
	FS_GNSS_DataReady_SetCallback(NULL);
	FS_GNSS_TimeReady_SetCallback(NULL);
	FS_GNSS_RawReady_SetCallback(NULL);
	FS_GNSS_IntReady_SetCallback(NULL);

	// Update RTC
	if (savedTime.week != 0)
	{
		// Start of the year 2000 (gmtime epoch)
		epoch = 1042 * 7 * 24 * 3600 + 518400;

		// Calculate timestamp at start_time
		timestamp = savedTime.week * 7 * 24 * 3600 - epoch;
		timestamp += savedTime.towMS / 1000;

		// Calculate millisecond part of date/time
		ms = savedTime.towMS % 1000;

		// Add the offset to milliseconds
		offset_ms = HAL_GetTick() - savedTime.time;
		ms_total = ms + offset_ms;

		// Calculate new timestamp and milliseconds
		timestamp += ms_total / 1000;
		ms = ms_total % 1000;

		// Convert back to date/time
		gmtime_r(timestamp, &year, &month, &day, &hour, &min, &sec);

		// Update RTC
		sTime.Hours = hour;
		sTime.Minutes = min;
		sTime.Seconds = sec;
		sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		sTime.StoreOperation = RTC_STOREOPERATION_RESET;

		sDate.WeekDay = RTC_WEEKDAY_MONDAY;
		sDate.Month = month;
		sDate.Date = day;
		sDate.Year = year % 100;

		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	}
}

void FS_Baro_DataReady_Callback(void)
{
	if (state != FS_CONTROL_ACTIVE) return;

	if (FS_Config_Get()->enable_logging)
	{
		// Save to log file
		FS_Log_WriteBaroData(FS_Baro_GetData());
	}
}

void FS_Hum_DataReady_Callback(void)
{
	if (state != FS_CONTROL_ACTIVE) return;

	if (FS_Config_Get()->enable_logging)
	{
		// Save to log file
		FS_Log_WriteHumData(FS_Hum_GetData());
	}
}

void FS_Mag_DataReady_Callback(void)
{
	if (state != FS_CONTROL_ACTIVE) return;

	if (FS_Config_Get()->enable_logging)
	{
		// Save to log file
		FS_Log_WriteMagData(FS_Mag_GetData());
	}
}

void FS_ActiveControl_DataReady_Callback(void)
{
	const FS_GNSS_Data_t *data = FS_GNSS_GetData();

	if (state != FS_CONTROL_ACTIVE) return;

	if (FS_Config_Get()->enable_audio)
	{
		// Update audio
		FS_AudioControl_UpdateGNSS(data);
	}

	if (FS_Config_Get()->enable_logging)
	{
		// Save to log file
		FS_Log_WriteGNSSData(data);

		// Update log path
		FS_Log_UpdatePath(data);
	}

	// Update BLE characteristic
	Custom_GNSS_Update(data);

	hasFix = (data->gpsFix == 3);
}

void FS_ActiveControl_TimeReady_Callback(bool validTime)
{
	const FS_GNSS_Time_t *gnssTime;

	if (state != FS_CONTROL_ACTIVE) return;

	if (hasFix)
	{
		// Turn off LED
		FS_LED_Off();
		HW_TS_Start(led_timer_id, LED_BLINK_TICKS);
	}

	if (validTime)
	{
		gnssTime = FS_GNSS_GetTime();

		if (FS_Config_Get()->enable_logging)
		{
			// Save to log file
			FS_Log_WriteGNSSTime(gnssTime);
		}

		// Update saved GNSS time
		memcpy(&savedTime, gnssTime, sizeof(FS_GNSS_Time_t));
	}
}

void FS_ActiveControl_RawReady_Callback(void)
{
	if (state != FS_CONTROL_ACTIVE) return;

	if (FS_Config_Get()->enable_logging)
	{
		// Save to log file
		FS_Log_WriteGNSSRaw(FS_GNSS_GetRaw());
	}
}

void FS_IMU_DataReady_Callback(void)
{
	if (state != FS_CONTROL_ACTIVE) return;

	if (FS_Config_Get()->enable_logging)
	{
		// Save to log file
		FS_Log_WriteIMUData(FS_IMU_GetData());
	}
}

void FS_VBAT_ValueReady_Callback(void)
{
	if (state != FS_CONTROL_ACTIVE) return;

	// Save to log file
	FS_Log_WriteVBATData(FS_VBAT_GetData());
}
