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

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>

#define FS_CONFIG_MAX_ALARMS    20
#define FS_CONFIG_MAX_WINDOWS   2
#define FS_CONFIG_MAX_SPEECH    3
#define FS_CONFIG_MAX_AL_LINES  4

#define FS_CONFIG_MODEL_PORTABLE     0
#define FS_CONFIG_MODEL_STATIONARY   2
#define FS_CONFIG_MODEL_PEDESTRIAN   3
#define FS_CONFIG_MODEL_AUTOMOTIVE   4
#define FS_CONFIG_MODEL_SEA          5
#define FS_CONFIG_MODEL_AIRBORNE_1G  6
#define FS_CONFIG_MODEL_AIRBORNE_2G  7
#define FS_CONFIG_MODEL_AIRBORNE_4G  8

#define FS_CONFIG_MODE_HORIZONTAL_SPEED          0
#define FS_CONFIG_MODE_VERTICAL_SPEED            1
#define FS_CONFIG_MODE_GLIDE_RATIO               2
#define FS_CONFIG_MODE_INVERSE_GLIDE_RATIO       3
#define FS_CONFIG_MODE_TOTAL_SPEED               4
#define FS_CONFIG_MODE_DIRECTION_TO_DESTINATION  5
#define FS_CONFIG_MODE_DISTANCE_TO_DESTINATION   6
#define FS_CONFIG_MODE_DIRECTION_TO_BEARING      7
#define FS_CONFIG_MODE_MAGNITUDE_OF_VALUE_1      8
#define FS_CONFIG_MODE_CHANGE_IN_VALUE_1         9
#define FS_CONFIG_MODE_LEFT_RIGHT                10
#define FS_CONFIG_MODE_DIVE_ANGLE                11
#define FS_CONFIG_MODE_ALTITUDE                  12

#define FS_CONFIG_UNITS_KMH     0
#define FS_CONFIG_UNITS_MPH     1
#define FS_CONFIG_UNITS_KNOTS   2

#define FS_CONFIG_UNITS_METERS  0
#define FS_CONFIG_UNITS_FEET    1
#define FS_CONFIG_UNITS_NM      2

typedef enum {
    FS_UNIT_SYSTEM_METRIC = 0,
    FS_UNIT_SYSTEM_IMPERIAL = 1
} FS_Config_UnitSystem_t;

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
	uint8_t mode;
	FS_Config_UnitSystem_t units;
	int32_t decimals;
} FS_Config_AL_Line_t;

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
	uint8_t  ble_tx_power;
	uint8_t  enable_raw;
	uint8_t  cold_start;

	uint8_t  baro_odr;
	uint8_t  hum_odr;
	uint8_t  mag_odr;
	uint8_t  accel_odr;
	uint8_t  accel_fs;
	uint8_t  gyro_odr;
	uint8_t  gyro_fs;

	int32_t  lat;
	int32_t  lon;
	int16_t  bearing;
	uint16_t end_nav;
	uint16_t max_dist;
	uint16_t min_angle;

	char     al_id[6];
	uint8_t  al_mode;
	uint32_t al_rate;

	FS_Config_AL_Line_t al_lines[FS_CONFIG_MAX_AL_LINES];
	uint8_t  num_al_lines;

	// Flag to control whether navigation modes are allowed
	uint8_t  enable_nav;
} FS_Config_Data_t;

typedef enum {
	FS_CONFIG_OK = 0,
	FS_CONFIG_ERR
} FS_Config_Result_t;

void FS_Config_Init(void);
FS_Config_Result_t FS_Config_Read(const char *filename);
FS_Config_Result_t FS_Config_Write(const char *filename);
const FS_Config_Data_t *FS_Config_Get(void);

#endif /* CONFIG_H_ */
