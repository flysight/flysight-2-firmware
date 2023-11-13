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

#include "main.h"
#include "app_common.h"
#include "vbat.h"

#define VBAT_TIMER_MSEC     1000
#define VBAT_TIMER_TICKS    (VBAT_TIMER_MSEC*1000/CFG_TS_TICK_VAL)

/* Value of analog reference voltage (Vref+), connected to analog voltage   */
/* supply Vdda (unit: mV).                                                  */
#define VDDA_APPLI                       ((uint32_t)3300)

/* Full-scale digital value with a resolution of 12 bits (voltage range     */
/* determined by analog voltage references Vref+ and Vref-,                 */
/* refer to reference manual).                                              */
#define DIGITAL_SCALE_12BITS             ((uint32_t) 0xFFF)

/**
  * @brief  Macro to calculate the voltage (unit: mVolt)
  *         corresponding to a ADC conversion data (unit: digital value).
  * @note   ADC measurement data must correspond to a resolution of 12bits
  *         (full scale digital value 4095). If not the case, the data must be
  *         preliminarily rescaled to an equivalent resolution of 12 bits.
  * @note   Analog reference voltage (Vref+) must be known from
  *         user board environment.
  * @param  __VREFANALOG_VOLTAGE__ Analog reference voltage (unit: mV)
  * @param  __ADC_DATA__ ADC conversion data (resolution 12 bits)
  *                       (unit: digital value).
  * @retval ADC conversion data equivalent voltage value (unit: mVolt)
  */
#define __ADC_CALC_DATA_VOLTAGE(__VREFANALOG_VOLTAGE__, __ADC_DATA__)       \
  ((__ADC_DATA__) * (__VREFANALOG_VOLTAGE__) / DIGITAL_SCALE_12BITS)

static uint8_t vbat_timer_id;

extern ADC_HandleTypeDef hadc1;

static FS_VBAT_Data_t vbatData;

static void FS_VBAT_Timer(void)
{
	// Enable battery measurement
	HAL_GPIO_WritePin(VBAT_EN_GPIO_Port, VBAT_EN_Pin, GPIO_PIN_SET);

	// Start ADC conversion
    HAL_ADC_Start_IT(&hadc1);
}

void FS_VBAT_Init(void)
{
	// Initialize measurement timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &vbat_timer_id, hw_ts_Repeated, FS_VBAT_Timer);
	HW_TS_Start(vbat_timer_id, VBAT_TIMER_TICKS);
}

void FS_VBAT_DeInit(void)
{
	// Delete timer
	HW_TS_Delete(vbat_timer_id);
}

void FS_VBAT_ConversionComplete(void)
{
	// Disable battery measurement
	HAL_GPIO_WritePin(VBAT_EN_GPIO_Port, VBAT_EN_Pin, GPIO_PIN_RESET);

	// Get battery voltage
	vbatData.time = HAL_GetTick();
	uint16_t temp = HAL_ADC_GetValue(&hadc1);
	vbatData.voltage = __ADC_CALC_DATA_VOLTAGE(VDDA_APPLI, temp * 2);

	// Process this data
	FS_VBAT_ValueReady_Callback();
}

const FS_VBAT_Data_t *FS_VBAT_GetData(void)
{
	return &vbatData;
}

__weak void FS_VBAT_ValueReady_Callback(void)
{
  /* NOTE: This function should not be modified, when the callback is needed,
           the FS_GNSS_TimeReady_Callback could be implemented in the user file
   */
}
