/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32_LPM_IF_H
#define STM32_LPM_IF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/**
  * @brief Enters Low Power Off Mode
  * @param none
  * @retval none
  */
void PWR_EnterOffMode( void );
/**
  * @brief Exits Low Power Off Mode
  * @param none
  * @retval none
  */
void PWR_ExitOffMode( void );

/**
  * @brief Enters Low Power Stop Mode
  * @note ARM exists the function when waking up
  * @param none
  * @retval none
  */
void PWR_EnterStopMode( void );
/**
  * @brief Exits Low Power Stop Mode
  * @note Enable the pll at 32MHz
  * @param none
  * @retval none
  */
void PWR_ExitStopMode( void );

/**
  * @brief Enters Low Power Sleep Mode
  * @note ARM exits the function when waking up
  * @param none
  * @retval none
  */
void PWR_EnterSleepMode( void );

/**
  * @brief Exits Low Power Sleep Mode
  * @note ARM exits the function when waking up
  * @param none
  * @retval none
  */
void PWR_ExitSleepMode( void );

#ifdef __cplusplus
}
#endif

#endif /*STM32_LPM_IF_H */

