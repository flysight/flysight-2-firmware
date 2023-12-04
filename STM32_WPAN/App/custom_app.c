/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "crs.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* CRS */
  uint8_t               Crs_tx_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */
Custom_CRS_Packet_t tx_packet;
Custom_CRS_Packet_t rx_packet;

uint8_t notify_flag;

extern uint8_t SizeCrs_Tx;
extern uint8_t SizeCrs_Rx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* CRS */
static void Custom_Crs_tx_Update_Char(void);
static void Custom_Crs_tx_Send_Notification(void);

/* USER CODE BEGIN PFP */
static void Custom_CRS_OnConnect(void);
static void Custom_CRS_OnTxRead(void);
static void Custom_CRS_OnRxWrite(Custom_STM_App_Notification_evt_t *pNotification);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* CRS */
    case CUSTOM_STM_CRS_TX_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CRS_TX_READ_EVT */
      Custom_CRS_OnTxRead();
      /* USER CODE END CUSTOM_STM_CRS_TX_READ_EVT */
      break;

    case CUSTOM_STM_CRS_TX_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CRS_TX_NOTIFY_ENABLED_EVT */
      notify_flag = 1;
      /* USER CODE END CUSTOM_STM_CRS_TX_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_CRS_TX_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CRS_TX_NOTIFY_DISABLED_EVT */
      notify_flag = 0;
      /* USER CODE END CUSTOM_STM_CRS_TX_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_CRS_RX_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CRS_RX_WRITE_EVT */
      Custom_CRS_OnRxWrite(pNotification);
      /* USER CODE END CUSTOM_STM_CRS_RX_WRITE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */
      Custom_CRS_OnConnect();
      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
  // Register retry transmit task
  UTIL_SEQ_RegTask(1<<CFG_TASK_CUSTOM_CRS_RETRY_TX_ID, UTIL_SEQ_RFU, Custom_CRS_SendNextTxPacket);
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* CRS */
void Custom_Crs_tx_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Crs_tx_UC_1*/

  /* USER CODE END Crs_tx_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_CRS_TX, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Crs_tx_UC_Last*/

  /* USER CODE END Crs_tx_UC_Last*/
  return;
}

void Custom_Crs_tx_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Crs_tx_NS_1*/

  /* USER CODE END Crs_tx_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_CRS_TX, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Crs_tx_NS_Last*/

  /* USER CODE END Crs_tx_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
static void Custom_CRS_OnConnect(void)
{
  // Reset state
  notify_flag = 0;
}

static void Custom_CRS_OnTxRead(void)
{
  // Check if there is another packet to send
  FS_CRS_PushQueue(FS_CRS_EVENT_TX_READ);
}

static void Custom_CRS_OnRxWrite(Custom_STM_App_Notification_evt_t *pNotification)
{
  // Copy packet data
  rx_packet.length = pNotification->DataTransfered.Length;
  memcpy(rx_packet.data, pNotification->DataTransfered.pPayload,
      pNotification->DataTransfered.Length);

  // Handle the packet
  FS_CRS_PushQueue(FS_CRS_EVENT_RX_WRITE);
}

Custom_CRS_Packet_t *Custom_CRS_GetNextTxPacket(void)
{
  return &tx_packet;
}

void Custom_CRS_SendNextTxPacket(void)
{
  tBleStatus res;

  // Try to transmit packet data
  SizeCrs_Tx = tx_packet.length;
  res = Custom_STM_App_Update_Char(CUSTOM_STM_CRS_TX, tx_packet.data);

  if (res != BLE_STATUS_SUCCESS)
  {
    // Retry transmit operation
    UTIL_SEQ_SetTask(1<<CFG_TASK_CUSTOM_CRS_RETRY_TX_ID, CFG_SCH_PRIO_1);
  }
  else if (notify_flag)
  {
    // Check if there is another packet to send
    FS_CRS_PushQueue(FS_CRS_EVENT_TX_READ);
  }
}

Custom_CRS_Packet_t *Custom_CRS_GetNextRxPacket(void)
{
  return &rx_packet;
}
/* USER CODE END FD_LOCAL_FUNCTIONS*/
