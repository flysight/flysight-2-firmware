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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"

#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_ERROR
} MAIN_TransferStateTypeDef;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_ADC1_Init(void);
void MX_I2C1_Init(void);
void MX_SAI1_Init(void);
void MX_SPI2_Init(void);
void MX_USART1_UART_Init(void);
void MX_RNG_Init(void);
void MX_LPUART1_UART_Init(void);

/* USER CODE BEGIN EFP */
void PeriphClock_Config(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VBUS_DIV_Pin GPIO_PIN_2
#define VBUS_DIV_GPIO_Port GPIOA
#define VBUS_DIV_EXTI_IRQn EXTI2_IRQn
#define AUDIO_SDIN_Pin GPIO_PIN_3
#define AUDIO_SDIN_GPIO_Port GPIOC
#define MIC_OUT_Pin GPIO_PIN_2
#define MIC_OUT_GPIO_Port GPIOC
#define SENSOR_SDA_Pin GPIO_PIN_1
#define SENSOR_SDA_GPIO_Port GPIOC
#define AUDIO_LRCLK_Pin GPIO_PIN_9
#define AUDIO_LRCLK_GPIO_Port GPIOB
#define SENSOR_SCL_Pin GPIO_PIN_0
#define SENSOR_SCL_GPIO_Port GPIOC
#define AUDIO_SCL_Pin GPIO_PIN_8
#define AUDIO_SCL_GPIO_Port GPIOB
#define AUDIO_SDA_Pin GPIO_PIN_7
#define AUDIO_SDA_GPIO_Port GPIOB
#define IMU_MOSI_Pin GPIO_PIN_5
#define IMU_MOSI_GPIO_Port GPIOB
#define IMU_MISO_Pin GPIO_PIN_4
#define IMU_MISO_GPIO_Port GPIOB
#define CHG_EN_HI_Pin GPIO_PIN_10
#define CHG_EN_HI_GPIO_Port GPIOC
#define CHG_EN_LO_Pin GPIO_PIN_11
#define CHG_EN_LO_GPIO_Port GPIOC
#define BUTTON_Pin GPIO_PIN_12
#define BUTTON_GPIO_Port GPIOC
#define BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define IMU_NCS_Pin GPIO_PIN_15
#define IMU_NCS_GPIO_Port GPIOA
#define GNSS_TXD_Pin GPIO_PIN_10
#define GNSS_TXD_GPIO_Port GPIOA
#define MMC_NCS_Pin GPIO_PIN_0
#define MMC_NCS_GPIO_Port GPIOD
#define MMC_CLK_Pin GPIO_PIN_1
#define MMC_CLK_GPIO_Port GPIOD
#define AUDIO_BCLK_Pin GPIO_PIN_13
#define AUDIO_BCLK_GPIO_Port GPIOB
#define MAG_INT_Pin GPIO_PIN_6
#define MAG_INT_GPIO_Port GPIOC
#define MAG_INT_EXTI_IRQn EXTI9_5_IRQn
#define MMC_DO_Pin GPIO_PIN_14
#define MMC_DO_GPIO_Port GPIOB
#define MMC_DI_Pin GPIO_PIN_15
#define MMC_DI_GPIO_Port GPIOB
#define GNSS_RXD_Pin GPIO_PIN_6
#define GNSS_RXD_GPIO_Port GPIOB
#define BARO_INT_Pin GPIO_PIN_13
#define BARO_INT_GPIO_Port GPIOC
#define BARO_INT_EXTI_IRQn EXTI15_10_IRQn
#define GNSS_EXTINT_Pin GPIO_PIN_12
#define GNSS_EXTINT_GPIO_Port GPIOB
#define HUM_DRDY_Pin GPIO_PIN_4
#define HUM_DRDY_GPIO_Port GPIOE
#define HUM_DRDY_EXTI_IRQn EXTI4_IRQn
#define DEBUG_TX_Pin GPIO_PIN_11
#define DEBUG_TX_GPIO_Port GPIOB
#define DEBUG_RX_Pin GPIO_PIN_10
#define DEBUG_RX_GPIO_Port GPIOB
#define VBAT_DIV_Pin GPIO_PIN_7
#define VBAT_DIV_GPIO_Port GPIOA
#define IMU_SCK_Pin GPIO_PIN_5
#define IMU_SCK_GPIO_Port GPIOA
#define AUDIO_MCLK_Pin GPIO_PIN_3
#define AUDIO_MCLK_GPIO_Port GPIOA
#define VCC_EN_Pin GPIO_PIN_1
#define VCC_EN_GPIO_Port GPIOH
#define LED_R_Pin GPIO_PIN_14
#define LED_R_GPIO_Port GPIOD
#define MIC_EN_Pin GPIO_PIN_13
#define MIC_EN_GPIO_Port GPIOD
#define VBAT_EN_Pin GPIO_PIN_12
#define VBAT_EN_GPIO_Port GPIOD
#define CHG_STAT_Pin GPIO_PIN_7
#define CHG_STAT_GPIO_Port GPIOD
#define CHG_STAT_EXTI_IRQn EXTI9_5_IRQn
#define GNSS_SAFEBOOT_N_Pin GPIO_PIN_2
#define GNSS_SAFEBOOT_N_GPIO_Port GPIOD
#define IMU_INT1_Pin GPIO_PIN_9
#define IMU_INT1_GPIO_Port GPIOC
#define IMU_INT1_EXTI_IRQn EXTI9_5_IRQn
#define GNSS_PPS_Pin GPIO_PIN_3
#define GNSS_PPS_GPIO_Port GPIOD
#define GNSS_PPS_EXTI_IRQn EXTI3_IRQn
#define LED_G_Pin GPIO_PIN_15
#define LED_G_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
