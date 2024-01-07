/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2024 Bionic Avionics Inc.                                   **
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

#include "main.h"
#include "app_common.h"
#include "hts221.h"
#include "hum.h"
#include "sensor.h"

#define SHT4X_MEAS_MSEC    100
#define SHT4X_MEAS_RATE    (SHT4X_MEAS_MSEC*1000/CFG_TS_TICK_VAL)

#define CRC_POLYNOMIAL 0x31

#define SHT4X_ADDR                   0x88
#define SHT4X_MEASURE_HIGH_PRECISION 0xfd
#define SHT4X_MEASURE_MED_PRECISION  0xf6
#define SHT4X_MEASURE_LOW_PRECISION  0xe0
#define SHT4X_READ_SERIAL_NUMBER     0x89
#define SHT4X_SOFT_RESET             0x94
#define SHT4X_HEATER_200_MW_1000_MS  0x39
#define SHT4X_HEATER_200_MW_100_MS   0x32
#define SHT4X_HEATER_110_MW_1000_MS  0x2f
#define SHT4X_HEATER_110_MW_100_MS   0x24
#define SHT4X_HEATER_20_MW_1000_MS   0x1e
#define SHT4X_HEATER_20_MW_100_MS    0x15

static uint8_t buf[6];

static FS_Hum_Data_t *humData;

static uint8_t timer_id;

static volatile uint8_t measure_busy;

static void FS_SHT4X_Read(void);

static uint8_t CRC8(const uint8_t *data, int length)
{
    unsigned char crc = 0xff;

    while (length--)
    {
        crc ^= *data++; // XOR byte into least sig. byte of crc

        // Loop over each bit
        for (unsigned char i = 8; i > 0; --i)
        {
            if (crc & 0x80)  // If the uppermost bit is a 1...
                crc = (crc << 1) ^ CRC_POLYNOMIAL;
            else             // Else, just shift left
                crc <<= 1;
        }
    }

    return crc; // Final remainder is the CRC result
}

FS_Hum_Result_t FS_SHT4X_Init(FS_Hum_Data_t *data)
{
	HAL_StatusTypeDef result;

	// Keep local pointer to humidity data
	humData = data;

	// Set up serial number read
	buf[0] = SHT4X_READ_SERIAL_NUMBER;
	do
	{
		result = FS_Sensor_Transmit(SHT4X_ADDR, buf, 1);
	}
	while (result != HAL_OK);

	// Read serial number
	do
	{
		result = FS_Sensor_Receive(SHT4X_ADDR, buf, 6);
	}
	while (result != HAL_OK);

	// Check serial number CRC
	if ((CRC8(&buf[0], 2) != buf[2])
			|| (CRC8(&buf[3], 2) != buf[5]))
	{
		return FS_HUM_ERROR;
	}

	// Software reset
	buf[0] = SHT4X_SOFT_RESET;
	do
	{
		result = FS_Sensor_Transmit(SHT4X_ADDR, buf, 1);
	}
	while (result != HAL_OK);

	return FS_HUM_OK;
}

void FS_SHT4X_Start(void)
{
	// Reset measurement flag
	measure_busy = 0;

	// Start measurement timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_Repeated, FS_SHT4X_Read);
	HW_TS_Start(timer_id, SHT4X_MEAS_RATE);
}

void FS_SHT4X_Stop(void)
{
	// Delete measurement timer
	HW_TS_Delete(timer_id);
}

static void FS_SHT4X_Read_Callback(HAL_StatusTypeDef result)
{
	uint16_t val;

	// Reset measurement flag
	measure_busy = 0;

	// Read raw measurements
	if (result == HAL_OK)
	{
		if ((CRC8(&buf[0], 2) == buf[2])
				&& (CRC8(&buf[3], 2) == buf[5]))
		{
			// Compute temperature
			val = (int16_t) ((buf[0] << 8) | buf[1]);
			humData->temperature = -450 + (int32_t) 1750 * val / 0xffff;

			// Compute humidity
			val = (int16_t) ((buf[3] << 8) | buf[4]);
			humData->humidity = -60 + (int32_t) 1250 * val / 0xffff;

			FS_Hum_DataReady_Callback();
		}
	}

	humData->time = HAL_GetTick();
	buf[0] = SHT4X_MEASURE_HIGH_PRECISION;
	FS_Sensor_TransmitAsync(SHT4X_ADDR, buf, 1, 0);
}

static void FS_SHT4X_Read(void)
{
	if (!measure_busy)
	{
		measure_busy = 1;
		FS_Sensor_ReceiveAsync(SHT4X_ADDR, buf, 6, FS_SHT4X_Read_Callback);
	}
}
