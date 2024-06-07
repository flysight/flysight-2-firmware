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
#include "start_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* CRS */
  uint8_t               Crs_tx_Notification_Status;
  /* GNSS */
  uint8_t               Gnss_pv_Notification_Status;
  /* Start */
  uint8_t               Start_control_Indication_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */
  uint8_t               Crs_tx_Flow_Status;
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
static Custom_CRS_Packet_t tx_buffer[FS_CRS_WINDOW_LENGTH+1];
static uint32_t tx_read_index, tx_write_index;

static Custom_CRS_Packet_t rx_buffer[FS_CRS_WINDOW_LENGTH+1];
static uint32_t rx_read_index, rx_write_index;

static uint8_t gnss_pv_packet[28];

static Custom_Start_Packet_t start_buffer[FS_START_WINDOW_LENGTH+1];
static uint32_t start_read_index, start_write_index;

static uint8_t connected_flag = 0;

extern uint8_t SizeCrs_Tx;
extern uint8_t SizeCrs_Rx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* CRS */
static void Custom_Crs_tx_Update_Char(void);
static void Custom_Crs_tx_Send_Notification(void);
/* GNSS */
static void Custom_Gnss_pv_Update_Char(void);
static void Custom_Gnss_pv_Send_Notification(void);
/* Start */
static void Custom_Start_control_Update_Char(void);
static void Custom_Start_control_Send_Indication(void);

/* USER CODE BEGIN PFP */
static void Custom_CRS_OnConnect(void);
static void Custom_CRS_OnDisconnect(void);
static void Custom_CRS_OnRxWrite(Custom_STM_App_Notification_evt_t *pNotification);
static void Custom_CRS_Transmit(void);
static void Custom_GNSS_Transmit(void);
static void Custom_Start_OnControlWrite(Custom_STM_App_Notification_evt_t *pNotification);
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
    case CUSTOM_STM_CRS_TX_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CRS_TX_NOTIFY_ENABLED_EVT */
      Custom_App_Context.Crs_tx_Notification_Status = 1;
      UTIL_SEQ_SetTask(1<<CFG_TASK_CUSTOM_CRS_TRANSMIT_ID, CFG_SCH_PRIO_1);
      /* USER CODE END CUSTOM_STM_CRS_TX_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_CRS_TX_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CRS_TX_NOTIFY_DISABLED_EVT */
      Custom_App_Context.Crs_tx_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_CRS_TX_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_CRS_RX_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CRS_RX_READ_EVT */

      /* USER CODE END CUSTOM_STM_CRS_RX_READ_EVT */
      break;

    case CUSTOM_STM_CRS_RX_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CRS_RX_WRITE_NO_RESP_EVT */
      Custom_CRS_OnRxWrite(pNotification);
      /* USER CODE END CUSTOM_STM_CRS_RX_WRITE_NO_RESP_EVT */
      break;

    /* GNSS */
    case CUSTOM_STM_GNSS_PV_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_GNSS_PV_READ_EVT */

      /* USER CODE END CUSTOM_STM_GNSS_PV_READ_EVT */
      break;

    case CUSTOM_STM_GNSS_PV_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_GNSS_PV_NOTIFY_ENABLED_EVT */
      Custom_App_Context.Gnss_pv_Notification_Status = 1;
      /* USER CODE END CUSTOM_STM_GNSS_PV_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_GNSS_PV_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_GNSS_PV_NOTIFY_DISABLED_EVT */
      Custom_App_Context.Gnss_pv_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_GNSS_PV_NOTIFY_DISABLED_EVT */
      break;

    /* Start */
    case CUSTOM_STM_START_CONTROL_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_START_CONTROL_WRITE_EVT */
      Custom_Start_OnControlWrite(pNotification);
      /* USER CODE END CUSTOM_STM_START_CONTROL_WRITE_EVT */
      break;

    case CUSTOM_STM_START_CONTROL_INDICATE_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_START_CONTROL_INDICATE_ENABLED_EVT */
      Custom_App_Context.Start_control_Indication_Status = 1;
      /* USER CODE END CUSTOM_STM_START_CONTROL_INDICATE_ENABLED_EVT */
      break;

    case CUSTOM_STM_START_CONTROL_INDICATE_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_START_CONTROL_INDICATE_DISABLED_EVT */
        Custom_App_Context.Start_control_Indication_Status = 0;
      /* USER CODE END CUSTOM_STM_START_CONTROL_INDICATE_DISABLED_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
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
  UTIL_SEQ_RegTask(1<<CFG_TASK_CUSTOM_CRS_TRANSMIT_ID, UTIL_SEQ_RFU, Custom_CRS_Transmit);
  UTIL_SEQ_RegTask(1<<CFG_TASK_CUSTOM_GNSS_TRANSMIT_ID, UTIL_SEQ_RFU, Custom_GNSS_Transmit);

  Custom_App_Context.Crs_tx_Notification_Status = 0;
  Custom_App_Context.Crs_tx_Flow_Status = 1;
  Custom_App_Context.Gnss_pv_Notification_Status = 0;
  Custom_App_Context.Start_control_Indication_Status = 0;
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
void Custom_APP_TxPoolAvailableNotification(void)
{
  Custom_App_Context.Crs_tx_Flow_Status = 1;
  UTIL_SEQ_SetTask(1<<CFG_TASK_CUSTOM_CRS_TRANSMIT_ID, CFG_SCH_PRIO_1);
}

uint8_t Custom_APP_IsConnected(void)
{
  return connected_flag;
}
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

/* GNSS */
void Custom_Gnss_pv_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Gnss_pv_UC_1*/

  /* USER CODE END Gnss_pv_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_GNSS_PV, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Gnss_pv_UC_Last*/

  /* USER CODE END Gnss_pv_UC_Last*/
  return;
}

void Custom_Gnss_pv_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Gnss_pv_NS_1*/

  /* USER CODE END Gnss_pv_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_GNSS_PV, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Gnss_pv_NS_Last*/

  /* USER CODE END Gnss_pv_NS_Last*/

  return;
}

/* Start */
void Custom_Start_control_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Start_control_UC_1*/

  /* USER CODE END Start_control_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_START_CONTROL, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Start_control_UC_Last*/

  /* USER CODE END Start_control_UC_Last*/
  return;
}

void Custom_Start_control_Send_Indication(void) /* Property Indication */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Start_control_IS_1*/

  /* USER CODE END Start_control_IS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_START_CONTROL, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Start_control_IS_Last*/

  /* USER CODE END Start_control_IS_Last*/

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

  start_read_index = 0;
  start_write_index = 0;

  // Update state
  connected_flag = 1;
}

static void Custom_CRS_OnDisconnect(void)
{
  // Update state
  connected_flag = 0;

  // Call update task
  UTIL_SEQ_SetTask(1<<CFG_TASK_FS_CRS_UPDATE_ID, CFG_SCH_PRIO_1);
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
    APP_DBG_MSG("Custom_CRS_OnRxWrite: buffer overflow\n");
  }
}

static void Custom_CRS_Transmit(void)
{
  static uint8_t tx_busy = 0;
  Custom_CRS_Packet_t *packet;
  tBleStatus status;

  if (!tx_busy
      && (tx_read_index < tx_write_index)
      && Custom_App_Context.Crs_tx_Notification_Status
      && Custom_App_Context.Crs_tx_Flow_Status)
  {
    tx_busy = 1;

	packet = &tx_buffer[tx_read_index % FS_CRS_WINDOW_LENGTH];
	SizeCrs_Tx = packet->length;

	status = Custom_STM_App_Update_Char(CUSTOM_STM_CRS_TX, packet->data);
	if (status == BLE_STATUS_INSUFFICIENT_RESOURCES)
	{
      Custom_App_Context.Crs_tx_Flow_Status = 0;
	}
	else
	{
      ++tx_read_index;

      // Call update task and transmit next packet
      UTIL_SEQ_SetTask(1<<CFG_TASK_FS_CRS_UPDATE_ID, CFG_SCH_PRIO_1);
      UTIL_SEQ_SetTask(1<<CFG_TASK_CUSTOM_CRS_TRANSMIT_ID, CFG_SCH_PRIO_1);
	}

    tx_busy = 0;
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
    UTIL_SEQ_SetTask(1<<CFG_TASK_CUSTOM_CRS_TRANSMIT_ID, CFG_SCH_PRIO_1);
  }
  else
  {
    APP_DBG_MSG("Custom_CRS_SendNextTxPacket: buffer overflow\n");
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

static void Custom_GNSS_Transmit(void)
{
  Custom_STM_App_Update_Char(CUSTOM_STM_GNSS_PV, gnss_pv_packet);
}

void Custom_GNSS_Update(const FS_GNSS_Data_t *current)
{
  memcpy(&gnss_pv_packet[0], &(current->iTOW), sizeof(current->iTOW));
  memcpy(&gnss_pv_packet[4], &(current->lon), sizeof(current->lon));
  memcpy(&gnss_pv_packet[8], &(current->lat), sizeof(current->lat));
  memcpy(&gnss_pv_packet[12], &(current->hMSL), sizeof(current->hMSL));
  memcpy(&gnss_pv_packet[16], &(current->velN), sizeof(current->velN));
  memcpy(&gnss_pv_packet[20], &(current->velE), sizeof(current->velE));
  memcpy(&gnss_pv_packet[24], &(current->velD), sizeof(current->velD));

  UTIL_SEQ_SetTask(1<<CFG_TASK_CUSTOM_GNSS_TRANSMIT_ID, CFG_SCH_PRIO_1);
}

static void Custom_Start_OnControlWrite(Custom_STM_App_Notification_evt_t *pNotification)
{
  Custom_Start_Packet_t *packet;

  if (start_write_index < start_read_index + FS_START_WINDOW_LENGTH)
  {
	packet = &start_buffer[(start_write_index++) % FS_START_WINDOW_LENGTH];
	packet->length = pNotification->DataTransfered.Length;
    memcpy(packet->data, pNotification->DataTransfered.pPayload, packet->length);

	// Call update task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_START_UPDATE_ID, CFG_SCH_PRIO_1);
  }
  else
  {
    APP_DBG_MSG("Custom_Start_OnControlWrite: buffer overflow\n");
  }
}

Custom_Start_Packet_t *Custom_Start_GetNextControlPacket(void)
{
  Custom_Start_Packet_t *ret = 0;

  if (start_read_index < start_write_index)
  {
	ret = &start_buffer[(start_read_index++) % FS_START_WINDOW_LENGTH];
  }

  return ret;
}
/* USER CODE END FD_LOCAL_FUNCTIONS*/
