/*
 * config.h
 *
 *  Created on: Oct. 1, 2020
 *      Author: Michael
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>

#define FS_CONFIG_MAX_ALARMS    10
#define FS_CONFIG_MAX_WINDOWS   2
#define FS_CONFIG_MAX_SPEECH    3

#define FS_CONFIG_UNITS_KMH     0
#define FS_CONFIG_UNITS_MPH     1

#define FS_CONFIG_UNITS_METERS  0
#define FS_CONFIG_UNITS_FEET    1

#define FS_CONFIG_RATE_ONE_HZ   650
#define FS_CONFIG_RATE_FLATLINE UINT16_MAX

typedef struct
{
	int32_t elev;
	uint8_t type;
	char    filename[9];
} FS_Config_Alarm_t;

typedef struct
{
	int32_t top;
	int32_t bottom;
} FS_Config_Window_t;

typedef struct
{
	uint8_t mode;
	uint8_t units;
	int32_t decimals;
} FS_Config_Speech_t;

typedef struct
{
	uint8_t  model;
	uint16_t rate;

	uint8_t  mode;
	int32_t  min;
	int32_t  max;
	uint8_t  limits;
	uint16_t volume;

	uint8_t  mode_2;
	int32_t  min_2;
	int32_t  max_2;
	int32_t  min_rate;
	int32_t  max_rate;
	uint8_t  flatline;

	uint16_t sp_rate;
	uint16_t sp_volume;

	FS_Config_Speech_t speech[FS_CONFIG_MAX_SPEECH];
	uint8_t  num_speech;

	int32_t  threshold;
	int32_t  hThreshold;

	uint8_t  use_sas;
	int32_t  tz_offset;

	uint8_t  init_mode;
	char     init_filename[9];

	int32_t  alarm_window_above;
	int32_t  alarm_window_below;
	int32_t  dz_elev;

	FS_Config_Alarm_t alarms[FS_CONFIG_MAX_ALARMS];
	uint8_t  num_alarms;

	uint8_t  alt_units;
	int32_t  alt_step;

	FS_Config_Window_t windows[FS_CONFIG_MAX_WINDOWS];
	uint8_t  num_windows;

	uint8_t  enable_audio;
	uint8_t  enable_logging;
	uint8_t  enable_vbat;
	uint8_t  enable_mic;
	uint8_t  enable_imu;
	uint8_t  enable_gnss;
	uint8_t  enable_baro;
	uint8_t  enable_hum;
	uint8_t  enable_mag;
	uint8_t  enable_ble;
	uint8_t  ble_tx_power;
	uint8_t  enable_raw;
	uint8_t  cold_start;

	uint32_t device_id[3];
	uint32_t session_id[3];
} FS_Config_Data_t;

typedef enum {
	FS_CONFIG_OK = 0,
	FS_CONFIG_ERR
} FS_Config_Result_t;

void FS_Config_Init(void);
FS_Config_Result_t FS_Config_Read(const char *filename);
const FS_Config_Data_t *FS_Config_Get(void);

#endif /* CONFIG_H_ */
