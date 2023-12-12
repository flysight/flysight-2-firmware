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
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "crs.h"
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
static Custom_CRS_Packet_t tx_buffer[FS_CRS_WINDOW_LENGTH];
static uint16_t tx_read_index, tx_write_index;

static Custom_CRS_Packet_t rx_buffer[FS_CRS_WINDOW_LENGTH];
static uint16_t rx_read_index, rx_write_index;

static uint8_t notify_flag;
static uint8_t connected_flag = 0;

extern uint8_t SizeCrs_Tx;
extern uint8_t SizeCrs_Rx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* CRS */
static void Custom_Crs_tx_Update_Char(void);
static void Custom_Crs_tx_Send_Notification(void);

/* USER CODE BEGIN PFP */
static void Custom_CRS_OnConnect(void);
static void Custom_CRS_OnDisconnect(void);
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

    case CUSTOM_STM_CRS_RX_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CRS_RX_WRITE_NO_RESP_EVT */
      Custom_CRS_OnRxWrite(pNotification);
      /* USER CODE END CUSTOM_STM_CRS_RX_WRITE_NO_RESP_EVT */
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
      Custom_CRS_OnDisconnect();
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
  UTIL_SEQ_RegTask(1<<CFG_TASK_CUSTOM_CRS_TRANSMIT_ID, UTIL_SEQ_RFU, Custom_CRS_OnTxRead);
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
  // Reset buffer indices
  tx_read_index = 0;
  tx_write_index = 0;

  rx_read_index = 0;
  rx_write_index = 0;

  // Update state
  notify_flag = 0;
  connected_flag = 1;
}

static void Custom_CRS_OnDisconnect(void)
{
  // Update state
  connected_flag = 0;

  // Call update task
  UTIL_SEQ_SetTask(1<<CFG_TASK_FS_CRS_UPDATE_ID, CFG_SCH_PRIO_1);
}

static void Custom_CRS_OnTxRead(void)
{
  Custom_CRS_Packet_t *packet;

  if (tx_read_index < tx_write_index)
  {
    packet = &tx_buffer[tx_read_index % FS_CRS_WINDOW_LENGTH];
    SizeCrs_Tx = packet->length;

    if (Custom_STM_App_Update_Char(CUSTOM_STM_CRS_TX, packet->data) == BLE_STATUS_SUCCESS)
    {
      ++tx_read_index;

      // Call update task
      UTIL_SEQ_SetTask(1<<CFG_TASK_FS_CRS_UPDATE_ID, CFG_SCH_PRIO_1);
    }
  }

  if (notify_flag && (tx_read_index < tx_write_index))
  {
    // Call transmit task
    UTIL_SEQ_SetTask(1<<CFG_TASK_CUSTOM_CRS_TRANSMIT_ID, CFG_SCH_PRIO_1);
  }
}

static void Custom_CRS_OnRxWrite(Custom_STM_App_Notification_evt_t *pNotification)
{
  Custom_CRS_Packet_t *packet;

  if (rx_write_index < rx_read_index + FS_CRS_WINDOW_LENGTH)
  {
	packet = &rx_buffer[(rx_write_index++) % FS_CRS_WINDOW_LENGTH];
	packet->length = pNotification->DataTransfered.Length;
    memcpy(packet->data, pNotification->DataTransfered.pPayload, packet->length);

	// Call update task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_CRS_UPDATE_ID, CFG_SCH_PRIO_1);
  }
  else
  {
    // TODO: Buffer overflow
  }
}

Custom_CRS_Packet_t *Custom_CRS_GetNextTxPacket(void)
{
  Custom_CRS_Packet_t *ret = 0;

  if (tx_write_index < tx_read_index + FS_CRS_WINDOW_LENGTH)
  {
	ret = &tx_buffer[tx_write_index % FS_CRS_WINDOW_LENGTH];
  }

  return ret;
}

void Custom_CRS_SendNextTxPacket(void)
{
  if (tx_write_index < tx_read_index + FS_CRS_WINDOW_LENGTH)
  {
    ++tx_write_index;

    if (notify_flag)
    {
      // Call transmit task
      UTIL_SEQ_SetTask(1<<CFG_TASK_CUSTOM_CRS_TRANSMIT_ID, CFG_SCH_PRIO_1);
    }
  }
}

Custom_CRS_Packet_t *Custom_CRS_GetNextRxPacket(void)
{
  Custom_CRS_Packet_t *ret = 0;

  if (rx_read_index < rx_write_index)
  {
	ret = &rx_buffer[(rx_read_index++) % FS_CRS_WINDOW_LENGTH];
  }

  return ret;
}

uint8_t Custom_CRS_IsConnected(void)
{
  return connected_flag;
}
/* USER CODE END FD_LOCAL_FUNCTIONS*/
