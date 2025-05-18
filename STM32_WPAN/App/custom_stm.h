/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.h
  * @author  MCD Application Team
  * @brief   Header for custom_stm.c module.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CUSTOM_STM_H
#define CUSTOM_STM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ble_types.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  /* File_Transfer */
  CUSTOM_STM_FT_PACKET_OUT,
  CUSTOM_STM_FT_PACKET_IN,
  /* Sensor_Data */
  CUSTOM_STM_SD_GNSS_MEASUREMENT,
  CUSTOM_STM_SD_CONTROL_POINT,
  /* Starter_Pistol */
  CUSTOM_STM_SP_CONTROL_POINT,
  CUSTOM_STM_SP_RESULT,
  /* Device_State */
  CUSTOM_STM_DS_MODE,
  CUSTOM_STM_DS_CONTROL_POINT,
} Custom_STM_Char_Opcode_t;

typedef enum
{
  /* FT_Packet_Out */
  CUSTOM_STM_FT_PACKET_OUT_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_FT_PACKET_OUT_NOTIFY_DISABLED_EVT,
  /* FT_Packet_In */
  CUSTOM_STM_FT_PACKET_IN_READ_EVT,
  CUSTOM_STM_FT_PACKET_IN_WRITE_NO_RESP_EVT,
  /* SD_GNSS_Measurement */
  CUSTOM_STM_SD_GNSS_MEASUREMENT_READ_EVT,
  CUSTOM_STM_SD_GNSS_MEASUREMENT_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_SD_GNSS_MEASUREMENT_NOTIFY_DISABLED_EVT,
  /* SD_Control_Point */
  CUSTOM_STM_SD_CONTROL_POINT_WRITE_EVT,
  CUSTOM_STM_SD_CONTROL_POINT_INDICATE_ENABLED_EVT,
  CUSTOM_STM_SD_CONTROL_POINT_INDICATE_DISABLED_EVT,
  /* SP_Control_Point */
  CUSTOM_STM_SP_CONTROL_POINT_WRITE_EVT,
  CUSTOM_STM_SP_CONTROL_POINT_INDICATE_ENABLED_EVT,
  CUSTOM_STM_SP_CONTROL_POINT_INDICATE_DISABLED_EVT,
  /* SP_Result */
  CUSTOM_STM_SP_RESULT_READ_EVT,
  CUSTOM_STM_SP_RESULT_INDICATE_ENABLED_EVT,
  CUSTOM_STM_SP_RESULT_INDICATE_DISABLED_EVT,
  /* DS_Mode */
  CUSTOM_STM_DS_MODE_READ_EVT,
  CUSTOM_STM_DS_MODE_INDICATE_ENABLED_EVT,
  CUSTOM_STM_DS_MODE_INDICATE_DISABLED_EVT,
  /* DS_Control_Point */
  CUSTOM_STM_DS_CONTROL_POINT_WRITE_EVT,
  CUSTOM_STM_DS_CONTROL_POINT_INDICATE_ENABLED_EVT,
  CUSTOM_STM_DS_CONTROL_POINT_INDICATE_DISABLED_EVT,
  CUSTOM_STM_NOTIFICATION_COMPLETE_EVT,

  CUSTOM_STM_BOOT_REQUEST_EVT
} Custom_STM_Opcode_evt_t;

typedef struct
{
  uint8_t * pPayload;
  uint8_t   Length;
} Custom_STM_Data_t;

typedef struct
{
  Custom_STM_Opcode_evt_t       Custom_Evt_Opcode;
  Custom_STM_Data_t             DataTransfered;
  uint16_t                      ConnectionHandle;
  uint8_t                       ServiceInstance;
  uint16_t                      AttrHandle;
} Custom_STM_App_Notification_evt_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
extern uint8_t SizeFt_Packet_Out;
extern uint8_t SizeFt_Packet_In;
extern uint8_t SizeSd_Gnss_Measurement;
extern uint8_t SizeSd_Control_Point;
extern uint8_t SizeSp_Control_Point;
extern uint8_t SizeSp_Result;
extern uint8_t SizeDs_Mode;
extern uint8_t SizeDs_Control_Point;

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void SVCCTL_InitCustomSvc(void);
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification);
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode,  uint8_t *pPayload);
/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*CUSTOM_STM_H */
