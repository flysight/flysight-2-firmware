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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  /* CRS */
  CUSTOM_STM_CRS_TX,
  CUSTOM_STM_CRS_RX,
  /* GNSS */
  CUSTOM_STM_GNSS_PV,
  /* Start */
  CUSTOM_STM_START_CONTROL,
} Custom_STM_Char_Opcode_t;

typedef enum
{
  /* CRS_TX */
  CUSTOM_STM_CRS_TX_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_CRS_TX_NOTIFY_DISABLED_EVT,
  /* CRS_RX */
  CUSTOM_STM_CRS_RX_READ_EVT,
  CUSTOM_STM_CRS_RX_WRITE_NO_RESP_EVT,
  /* GNSS_PV */
  CUSTOM_STM_GNSS_PV_READ_EVT,
  CUSTOM_STM_GNSS_PV_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_GNSS_PV_NOTIFY_DISABLED_EVT,
  /* Start_Control */
  CUSTOM_STM_START_CONTROL_WRITE_EVT,
  CUSTOM_STM_START_CONTROL_INDICATE_ENABLED_EVT,
  CUSTOM_STM_START_CONTROL_INDICATE_DISABLED_EVT,
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
extern uint8_t SizeCrs_Tx;
extern uint8_t SizeCrs_Rx;
extern uint8_t SizeGnss_Pv;
extern uint8_t SizeStart_Control;

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
