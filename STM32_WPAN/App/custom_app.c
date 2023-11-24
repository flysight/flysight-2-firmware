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
#define BUFFER_LEN (244*2)
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
uint8_t tx_buffer[BUFFER_LEN];
uint16_t tx_read_index, tx_write_index;

uint8_t rx_buffer[BUFFER_LEN];
uint16_t rx_read_index, rx_write_index;

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
static void Custom_CRS_Update(void);
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

      // Call update task
      UTIL_SEQ_SetTask(1<<CFG_TASK_CUSTOM_CRS_UPDATE_ID, CFG_SCH_PRIO_1);
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
  // Register update task
  UTIL_SEQ_RegTask(1<<CFG_TASK_CUSTOM_CRS_UPDATE_ID, UTIL_SEQ_RFU, Custom_CRS_Update);
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

  // Reset state
  notify_flag = 0;
}

static void Custom_CRS_OnTxRead(void)
{
  if (tx_read_index + SizeCrs_Tx <= tx_write_index)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_CRS_TX, tx_buffer + (tx_read_index % BUFFER_LEN));
    tx_read_index += SizeCrs_Tx;
  }
}

static void Custom_CRS_OnRxWrite(Custom_STM_App_Notification_evt_t *pNotification)
{
  if (rx_write_index + SizeCrs_Rx <= rx_read_index + BUFFER_LEN)
  {
    memcpy(rx_buffer + (rx_write_index % BUFFER_LEN), pNotification->DataTransfered.pPayload, SizeCrs_Rx);
    rx_write_index += SizeCrs_Rx;
  }
  else
  {
    // Buffer overflow
  }
}

static void Custom_CRS_Update(void)
{
  uint16_t saved_index = tx_write_index;

  while (notify_flag && (tx_read_index != saved_index))
  {
	  Custom_CRS_OnTxRead();
  }
}

int Custom_CRS_GetChar(void)
{
  int ret = -1;

  if (rx_read_index + 1 <= rx_write_index)
  {
    ret = rx_buffer[rx_read_index % BUFFER_LEN];
    ++rx_read_index;
  }

  return ret;
}

int Custom_CRS_PutChar(int ch)
{
  int ret = -1;

  if (tx_write_index + 1 <= tx_read_index + BUFFER_LEN)
  {
    ret = tx_buffer[tx_write_index % BUFFER_LEN] = (uint8_t) ch;
    ++tx_write_index;
  }

  if (tx_read_index + SizeCrs_Tx <= tx_write_index)
  {
    // Call update task
    UTIL_SEQ_SetTask(1<<CFG_TASK_CUSTOM_CRS_UPDATE_ID, CFG_SCH_PRIO_1);
  }

  return ret;
}
/* USER CODE END FD_LOCAL_FUNCTIONS*/
