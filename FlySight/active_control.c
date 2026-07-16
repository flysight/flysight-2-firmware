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
#include "activelook.h"
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
#include "rtc_util.h"
#include "time.h"
#include "vbat.h"

#define LED_BLINK_MSEC      900
#define LED_BLINK_TICKS     (LED_BLINK_MSEC*1000/CFG_TS_TICK_VAL)

static uint8_t led_timer_id;

static volatile bool hasFix;
static volatile bool rtcUpdated;

static volatile enum {
	FS_CONTROL_INACTIVE = 0,
	FS_CONTROL_ACTIVE
} state = FS_CONTROL_INACTIVE;

static FS_GNSS_Time_t savedTime;

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
	rtcUpdated = false;
	state = FS_CONTROL_ACTIVE;

	// Initialize saved GNSS time
	memset(&savedTime, 0, sizeof(FS_GNSS_Time_t));
}

void FS_ActiveControl_DeInit(void)
{
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

	if (FS_Config_Get()->al_mode != 0)
	{
		// Update ActiveLook measurement filters
		FS_ActiveLook_UpdateGNSS(data);
	}

	if (FS_Config_Get()->enable_logging)
	{
		// Save to log file
		FS_Log_WriteGNSSData(data);
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

		// Set RTC on first valid time with 3D fix
		if (hasFix && !rtcUpdated)
		{
			FS_RTC_SetFromGNSS(&savedTime);
			rtcUpdated = true;
		}
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

	const FS_VBAT_Data_t *vbat_data = FS_VBAT_GetData();

	// Save to log file
	FS_Log_WriteVBATData(vbat_data);

	// Update BLE characteristic
	Custom_VBAT_Update(vbat_data);
}

void FS_ActiveControl_SetHealthStatus(bool isSystemHealthy)
{
	if (state != FS_CONTROL_ACTIVE) return;

	// Set LED color based on system health from initialization
	FS_LED_SetColour(isSystemHealthy ? FS_LED_GREEN : FS_LED_RED);
}
