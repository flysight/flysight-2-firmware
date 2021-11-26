/*
 * config.c
 *
 *  Created on: Oct. 1, 2020
 *      Author: Michael
 */

#include "main.h"
#include "app_common.h"
#include "config.h"

#define CONFIG_FIRST_ALARM  0x01
#define CONFIG_FIRST_WINDOW 0x02
#define CONFIG_FIRST_SPEECH 0x04

static FS_Config_Data_t config;

void FS_Config_Init(void)
{
	config.model         = 7;
	config.rate          = 200;

	config.mode          = 2;
	config.min           = 0;
	config.max           = 300;
	config.limits        = 1;
	config.volume        = 2;

	config.mode_2        = 9;
	config.min_2         = 300;
	config.max_2         = 1500;
	config.min_rate      = FS_CONFIG_RATE_ONE_HZ;
	config.max_rate      = 5 * FS_CONFIG_RATE_ONE_HZ;
	config.flatline      = 0;

	config.sp_rate       = 0;
	config.sp_volume     = 0;

	config.num_speech    = 0;

	config.threshold     = 1000;
	config.hThreshold    = 0;

	config.use_sas       = 1;
	config.tz_offset     = 0;

	config.init_mode     = 0;

	config.alarm_window_above = 0;
	config.alarm_window_below = 0;
	config.dz_elev       = 0;

	config.num_alarms    = 0;

	config.alt_units     = FS_CONFIG_UNITS_FEET;
	config.alt_step      = 0;

	config.num_windows   = 0;

	config.enable_audio   = 1;
	config.enable_tone    = 0;
	config.enable_logging = 1;
	config.enable_vbat    = 1;
	config.enable_mic     = 1;
	config.enable_imu     = 1;
	config.enable_gnss    = 1;
	config.enable_baro    = 1;
	config.enable_hum     = 1;
	config.enable_mag     = 1;
	config.enable_ble     = 1;
	config.ble_tx_power   = 25;
}

const FS_Config_Data_t *FS_Config_Get(void)
{
	return &config;
}
