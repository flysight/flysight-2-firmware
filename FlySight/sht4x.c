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
#include "config.h"
#include "hum.h"
#include "log.h"
#include "sensor.h"
#include "sht4x.h"

#define READ_TIMER_MSEC    10
#define READ_TIMER_TICKS   (READ_TIMER_MSEC*1000/CFG_TS_TICK_VAL)

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

static uint8_t measure_timer_id;
static uint8_t read_timer_id;

typedef enum {
    SHT4X_STATE_IDLE,
    SHT4X_STATE_WAITING_FOR_MEASUREMENT
} FS_SHT4X_State_t;

static volatile FS_SHT4X_State_t sht4x_state = SHT4X_STATE_IDLE;

static void FS_SHT4X_Measure(void);
static void FS_SHT4X_Measure_Callback(HAL_StatusTypeDef result);
static void FS_SHT4X_Read(void);
static void FS_SHT4X_Read_Callback(HAL_StatusTypeDef result);

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

HAL_StatusTypeDef FS_SHT4X_Init(FS_Hum_Data_t *data)
{
	HAL_StatusTypeDef result;

	// Keep local pointer to humidity data
	humData = data;

	// Set up serial number read
	buf[0] = SHT4X_READ_SERIAL_NUMBER;
	if (FS_Sensor_Transmit(SHT4X_ADDR, buf, 1) != HAL_OK)
	{
		return HAL_ERROR;
	}

	// Read serial number
	if (FS_Sensor_Receive(SHT4X_ADDR, buf, 6) != HAL_OK)
	{
		return HAL_ERROR;
	}

	// Check serial number CRC
	if ((CRC8(&buf[0], 2) != buf[2])
			|| (CRC8(&buf[3], 2) != buf[5]))
	{
		return HAL_ERROR;
	}

	// Software reset
	buf[0] = SHT4X_SOFT_RESET;
	do
	{
		result = FS_Sensor_Transmit(SHT4X_ADDR, buf, 1);
	}
	while (result != HAL_OK);

	return HAL_OK;
}

HAL_StatusTypeDef FS_SHT4X_Start(void)
{
	const FS_Config_Data_t *config = FS_Config_Get();

	// Reset measurement state
	sht4x_state = SHT4X_STATE_IDLE;

	// Create measurement timers
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &measure_timer_id, hw_ts_Repeated, FS_SHT4X_Measure);
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &read_timer_id, hw_ts_SingleShot, FS_SHT4X_Read);

	// Start measurement timer
	switch(config->hum_odr)
	{
	case 0: // HTS221_ODR_OS
		break;
	case 1: // HTS221_ODR_1
		HW_TS_Start(measure_timer_id, 1000000UL / CFG_TS_TICK_VAL);
		break;
	case 2: // HTS221_ODR_7
		HW_TS_Start(measure_timer_id, 142857UL / CFG_TS_TICK_VAL);
		break;
	case 3: // HTS221_ODR_12_5
		HW_TS_Start(measure_timer_id, 80000UL / CFG_TS_TICK_VAL);
		break;
	default:
		Error_Handler(); // Should never be called
	}

	return HAL_OK;
}

HAL_StatusTypeDef FS_SHT4X_Stop(void)
{
	// Delete measurement timers
	HW_TS_Delete(measure_timer_id);
	HW_TS_Delete(read_timer_id);

	return HAL_OK;
}

static void FS_SHT4X_Measure(void)
{
	if (sht4x_state == SHT4X_STATE_IDLE)
	{
		sht4x_state = SHT4X_STATE_WAITING_FOR_MEASUREMENT;

		humData->time = HAL_GetTick();
		buf[0] = SHT4X_MEASURE_HIGH_PRECISION;
		FS_Sensor_TransmitAsync(SHT4X_ADDR, buf, 1, FS_SHT4X_Measure_Callback);
	}
}

static void FS_SHT4X_Measure_Callback(HAL_StatusTypeDef result)
{
	if (result == HAL_OK)
	{
		// The command was successfully sent. Start a one-shot timer to wait
		// for the sensor's internal measurement to complete.
		HW_TS_Start(read_timer_id, READ_TIMER_TICKS);
	}
	else
	{
		// Abort this measurement cycle and reset the state to allow the next one.
		FS_Log_WriteEvent("Error starting humidity measurement");
		sht4x_state = SHT4X_STATE_IDLE;
	}
}

static void FS_SHT4X_Read(void)
{
	// This is called by the one-shot measurement timer.
	// It is now safe to queue the I2C read command.
	FS_Sensor_ReceiveAsync(SHT4X_ADDR, buf, 6, FS_SHT4X_Read_Callback);
}

static void FS_SHT4X_Read_Callback(HAL_StatusTypeDef result)
{
	uint16_t val;

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
	else
	{
		FS_Log_WriteEvent("Error reading from humidity sensor");
	}

	// This measurement cycle is now complete, reset the state.
	sht4x_state = SHT4X_STATE_IDLE;
}
