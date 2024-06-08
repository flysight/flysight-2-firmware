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
#include "vbat.h"

#define LED_BLINK_MSEC      900
#define LED_BLINK_TICKS     (LED_BLINK_MSEC*1000/CFG_TS_TICK_VAL)

static uint8_t led_timer_id;

static volatile bool hasFix = false;

static volatile enum {
	FS_CONTROL_INACTIVE = 0,
	FS_CONTROL_ACTIVE
} state = FS_CONTROL_INACTIVE;

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

	if (FS_Config_Get()->enable_logging)
	{
		// Save to log file
		FS_Log_WriteGNSSData(data);
	}

	if (Custom_APP_IsConnected())
	{
		// Update BLE characteristic
		Custom_GNSS_Update(data);
	}

	hasFix = (data->gpsFix == 3);
}

void FS_ActiveControl_TimeReady_Callback(bool validTime)
{
	if (state != FS_CONTROL_ACTIVE) return;

	if (hasFix)
	{
		// Turn off LED
		FS_LED_Off();
		HW_TS_Start(led_timer_id, LED_BLINK_TICKS);
	}

	if (validTime && FS_Config_Get()->enable_logging)
	{
		// Save to log file
		FS_Log_WriteGNSSTime(FS_GNSS_GetTime());
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
