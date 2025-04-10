/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2025 Bionic Avionics Inc.                                   **
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

#include <math.h>

#include "flight_params.h"
#include "config.h"

static const uint16_t sas_table[] =
{
	1024, 1077, 1135, 1197,
	1265, 1338, 1418, 1505,
	1600, 1704, 1818, 1944
};
#define SAS_TABLE_SIZE (sizeof(sas_table)/sizeof(sas_table[0]))

double FS_FlightParams_GetSASCorrectionFactor(int32_t hMSL)
{
	const FS_Config_Data_t *cfg = FS_Config_Get();

	if (!cfg->use_sas) {
		return 1.0; // No correction needed
	}

	uint16_t speed_mul;

	if (hMSL < 0) {
		speed_mul = sas_table[0];
	} else if (hMSL >= ((SAS_TABLE_SIZE - 1) * 1024 * 1024)) {
		speed_mul = sas_table[SAS_TABLE_SIZE - 1];
	} else {
		int32_t h = hMSL / 1024;
		uint16_t i = h / 1024;
		uint16_t j = h % 1024;
		uint16_t y1 = sas_table[i];
		uint16_t y2 = sas_table[i + 1];
		speed_mul = y1 + (((uint32_t)(y2 - y1) * j) / 1024);
	}

	if (speed_mul == 0) {
		return 1.0; // Avoid division by zero, return no correction
	}

	// Return the correction factor
	return 1024.0 / (double)speed_mul;
}
