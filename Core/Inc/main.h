/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
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

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void PeriphClock_Config(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VBUS_DIV_Pin GPIO_PIN_2
#define VBUS_DIV_GPIO_Port GPIOA
#define VBUS_DIV_EXTI_IRQn EXTI2_IRQn
#define IMU_MOSI_Pin GPIO_PIN_5
#define IMU_MOSI_GPIO_Port GPIOB
#define IMU_MISO_Pin GPIO_PIN_4
#define IMU_MISO_GPIO_Port GPIOB
#define BUTTON_Pin GPIO_PIN_12
#define BUTTON_GPIO_Port GPIOC
#define BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define IMU_NCS_Pin GPIO_PIN_15
#define IMU_NCS_GPIO_Port GPIOA
#define GNSS_TXD_Pin GPIO_PIN_10
#define GNSS_TXD_GPIO_Port GPIOA
#define GNSS_RXD_Pin GPIO_PIN_6
#define GNSS_RXD_GPIO_Port GPIOB
#define GNSS_EXTINT_Pin GPIO_PIN_12
#define GNSS_EXTINT_GPIO_Port GPIOB
#define IMU_SCK_Pin GPIO_PIN_5
#define IMU_SCK_GPIO_Port GPIOA
#define VCC_EN_Pin GPIO_PIN_1
#define VCC_EN_GPIO_Port GPIOH
#define LED_R_Pin GPIO_PIN_14
#define LED_R_GPIO_Port GPIOD
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
void   MX_USART1_UART_Init(void);
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
