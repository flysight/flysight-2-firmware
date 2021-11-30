/**
  ******************************************************************************
  * @file    stm32wbxx_nucleo.c
  * @author  MCD Application Team
  * @brief   This file provides set of firmware functions to manage:
  *          - LEDs and push-button available on STM32WBXX-Nucleo Kit 
  *            from STMicroelectronics
  *          - LCD, joystick and microSD available on Adafruit 1.8" TFT LCD 
  *            shield (reference ID 802)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_nucleo.h"

/** @addtogroup BSP
  * @{
  */ 

/** @defgroup STM32WBXX_NUCLEO STM32WBxx-Nucleo
  * @brief This file provides set of firmware functions to manage Leds and push-button
  *        available on STM32WBxx-Nucleo Kit from STMicroelectronics.
  *        It provides also LCD, joystick and uSD functions to communicate with 
  *        Adafruit 1.8" TFT LCD shield (reference ID 802)
  * @{
  */ 

/** @defgroup STM32WBXX_NUCLEO_Private_Defines Private Defines
  * @{
  */ 

/**
  * @brief STM32WBxx NUCLEO BSP Driver
  */
#define __STM32WBxx_NUCLEO_BSP_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define __STM32WBxx_NUCLEO_BSP_VERSION_SUB1   (0x00U) /*!< [23:16] sub1 version */
#define __STM32WBxx_NUCLEO_BSP_VERSION_SUB2   (0x01U) /*!< [15:8]  sub2 version */
#define __STM32WBxx_NUCLEO_BSP_VERSION_RC     (0x00U) /*!< [7:0]  release candidate */ 
#define __STM32WBxx_NUCLEO_BSP_VERSION        ((__STM32WBxx_NUCLEO_BSP_VERSION_MAIN << 24)\
                                             |(__STM32WBxx_NUCLEO_BSP_VERSION_SUB1 << 16)\
                                             |(__STM32WBxx_NUCLEO_BSP_VERSION_SUB2 << 8 )\
                                             |(__STM32WBxx_NUCLEO_BSP_VERSION_RC))   

/**
  * @brief LINK SD Card
  */
#define SD_DUMMY_BYTE            0xFF
#define SD_NO_RESPONSE_EXPECTED  0x80

/**
  * @}
  */ 

/** @defgroup STM32WBXX_NUCLEO_LOW_LEVEL_Private_Variables Private Variables
  * @{
  */ 

/**
 * @brief BUS variables
 */

#ifdef HAL_SPI_MODULE_ENABLED
uint32_t hnucleo_SpixTimeout = NUCLEO_SPIx_TIMEOUT_MAX;        /*<! Value of Timeout when SPI communication fails */
#endif /* HAL_SPI_MODULE_ENABLED */

/**
  * @}
  */ 

/** @defgroup STM32WBXX_NUCLEO_Private_Functions Private Functions
  * @{
  */ 
#ifdef HAL_SPI_MODULE_ENABLED
/* SD IO functions */
void                      SD_IO_Init(void);
void                      SD_IO_CSState(uint8_t state);
void                      SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength);
uint8_t                   SD_IO_WriteByte(uint8_t Data);
#endif /* HAL_SPI_MODULE_ENABLED */
/**
  * @}
  */ 

/* SPI handle */
extern SPI_HandleTypeDef hspi2;

/* Transfer state */
extern volatile MAIN_TransferStateTypeDef main_transfer_state;

/** @defgroup STM32WBXX_NUCLEO_Exported_Functions Exported Functions
  * @{
  */ 

/**
  * @brief  This method returns the STM32WBxx NUCLEO BSP Driver revision
  * @retval version: 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __STM32WBxx_NUCLEO_BSP_VERSION;
}

/**
  * @}
  */

/** @addtogroup STM32WBXX_NUCLEO_Private_Functions
  * @{
  */ 
  
#ifdef HAL_SPI_MODULE_ENABLED
/******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/
/**
  * @brief  SPI Write byte(s) to device
  * @param  DataIn: Pointer to data buffer to write
  * @param  DataOut: Pointer to data buffer for read data
  * @param  DataLength: number of bytes to write
  * @retval None
  */
static void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
  main_transfer_state = TRANSFER_WAIT;
  HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t*) DataIn, DataOut, DataLength);
  while (main_transfer_state == TRANSFER_WAIT);
}

/**
  * @brief  SPI Write a byte to device
  * @param  Value: value to be written
  * @retval None
  */
static uint8_t SPIx_WriteRead(uint8_t Value)
{
  *(__IO uint8_t *)&hspi2.Instance->DR = Value;
  while (!__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_RXNE));
  return *(__IO uint8_t *)&hspi2.Instance->DR;
}

/******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/********************************* LINK SD ************************************/
/**
  * @brief  Initialize the SD Card and put it into StandBy State (Ready for 
  *         data transfer).
  * @retval None
  */
void SD_IO_Init(void)
{
  uint8_t counter = 0;

  /*------------Put SD in SPI mode--------------*/
  /* SD SPI Config */
  MX_SPI2_Init();

  /* Enable SPI */
  __HAL_SPI_ENABLE(&hspi2);

  /* SD chip select high */
  SD_CS_HIGH();

  /* This delay seems to be necessary for subsequent transfers */
  uint32_t tick = HAL_GetTick();
  while (HAL_GetTick() - tick < 10);

  /* Send dummy byte 0xFF, 10 times with CS high */
  /* Rise CS and MOSI for 80 clocks cycles */
  for (counter = 0; counter <= 9; counter++)
  {
	/* Send dummy byte 0xFF */
	SD_IO_WriteByte(SD_DUMMY_BYTE);
  }
}

/**
  * @brief  Set SD interface Chip Select state
  * @param  val: 0 (low) or 1 (high) state
  * @retval None
  */
void SD_IO_CSState(uint8_t val)
{
  if(val == 1) 
  {
    SD_CS_HIGH();
  }
  else
  {
    SD_CS_LOW();
  }
}

/**
  * @brief  Write byte(s) on the SD
  * @param  DataIn: Pointer to data buffer to write
  * @param  DataOut: Pointer to data buffer for read data
  * @param  DataLength: number of bytes to write
  * @retval None
  */
void SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
  /* Send the byte */
  SPIx_WriteReadData(DataIn, DataOut, DataLength);
}

/**
  * @brief  Write a byte on the SD.
  * @param  Data: byte to send.
  * @retval Data written
  */
uint8_t SD_IO_WriteByte(uint8_t Data)
{
  return SPIx_WriteRead(Data);
}

#endif /* HAL_SPI_MODULE_ENABLED */
 
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */    

/**
  * @}
  */ 
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
