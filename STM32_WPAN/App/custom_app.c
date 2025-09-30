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
#include "gnss_ble.h"
#include "mode.h"
#include "ble_tx_queue.h"
#include "sensor_data.h"
#include "device_state.h"
#include "control_point_protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* File_Transfer */
  uint8_t               Ft_packet_out_Notification_Status;
  /* Sensor_Data */
  uint8_t               Sd_gnss_measurement_Notification_Status;
  uint8_t               Sd_control_point_Indication_Status;
  /* Starter_Pistol */
  uint8_t               Sp_control_point_Indication_Status;
  uint8_t               Sp_result_Indication_Status;
  /* Device_State */
  uint8_t               Ds_mode_Indication_Status;
  uint8_t               Ds_control_point_Indication_Status;
  /* Battery */
  uint8_t               Battery_level_Notification_Status;
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
#define TIMEOUT_MSEC  30000
#define TIMEOUT_TICKS (TIMEOUT_MSEC*1000/CFG_TS_TICK_VAL)
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
static Custom_CRS_Packet_t rx_buffer[FS_CRS_WINDOW_LENGTH+1];
static uint32_t rx_read_index, rx_write_index;

static uint8_t gnss_pv_packet[GNSS_BLE_MAX_LEN];

static uint8_t start_result_packet[9];

static uint8_t connected_flag = 0;

extern uint8_t SizeSd_Gnss_Measurement;
extern uint8_t SizeSd_Control_Point;

static uint8_t timeout_timer_id;

static uint8_t current_battery_level_percent = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* File_Transfer */
static void Custom_Ft_packet_out_Update_Char(void);
static void Custom_Ft_packet_out_Send_Notification(void);
/* Sensor_Data */
static void Custom_Sd_gnss_measurement_Update_Char(void);
static void Custom_Sd_gnss_measurement_Send_Notification(void);
static void Custom_Sd_control_point_Update_Char(void);
static void Custom_Sd_control_point_Send_Indication(void);
/* Starter_Pistol */
static void Custom_Sp_control_point_Update_Char(void);
static void Custom_Sp_control_point_Send_Indication(void);
static void Custom_Sp_result_Update_Char(void);
static void Custom_Sp_result_Send_Indication(void);
/* Device_State */
static void Custom_Ds_mode_Update_Char(void);
static void Custom_Ds_mode_Send_Indication(void);
static void Custom_Ds_control_point_Update_Char(void);
static void Custom_Ds_control_point_Send_Indication(void);
/* Battery */
static void Custom_Battery_level_Update_Char(void);
static void Custom_Battery_level_Send_Notification(void);

/* USER CODE BEGIN PFP */
static void Custom_CRS_OnConnect(Custom_App_ConnHandle_Not_evt_t *pNotification);
static void Custom_CRS_OnDisconnect(void);
static void Custom_CRS_OnRxWrite(Custom_STM_App_Notification_evt_t *pNotification);
static void Custom_App_Timeout(void);
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

    /* File_Transfer */
    case CUSTOM_STM_FT_PACKET_OUT_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_FT_PACKET_OUT_NOTIFY_ENABLED_EVT */
      Custom_App_Context.Ft_packet_out_Notification_Status = 1;
      /* USER CODE END CUSTOM_STM_FT_PACKET_OUT_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_FT_PACKET_OUT_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_FT_PACKET_OUT_NOTIFY_DISABLED_EVT */
      Custom_App_Context.Ft_packet_out_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_FT_PACKET_OUT_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_FT_PACKET_IN_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_FT_PACKET_IN_READ_EVT */

      /* USER CODE END CUSTOM_STM_FT_PACKET_IN_READ_EVT */
      break;

    case CUSTOM_STM_FT_PACKET_IN_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_FT_PACKET_IN_WRITE_NO_RESP_EVT */
      Custom_CRS_OnRxWrite(pNotification);
      /* USER CODE END CUSTOM_STM_FT_PACKET_IN_WRITE_NO_RESP_EVT */
      break;

    /* Sensor_Data */
    case CUSTOM_STM_SD_GNSS_MEASUREMENT_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_GNSS_MEASUREMENT_READ_EVT */
      if (!Custom_App_Context.Sd_gnss_measurement_Notification_Status)
      {
        Custom_STM_App_Update_Char(CUSTOM_STM_SD_GNSS_MEASUREMENT, gnss_pv_packet);
      }
      /* USER CODE END CUSTOM_STM_SD_GNSS_MEASUREMENT_READ_EVT */
      break;

    case CUSTOM_STM_SD_GNSS_MEASUREMENT_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_GNSS_MEASUREMENT_NOTIFY_ENABLED_EVT */
      Custom_App_Context.Sd_gnss_measurement_Notification_Status = 1;
      /* USER CODE END CUSTOM_STM_SD_GNSS_MEASUREMENT_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SD_GNSS_MEASUREMENT_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_GNSS_MEASUREMENT_NOTIFY_DISABLED_EVT */
      Custom_App_Context.Sd_gnss_measurement_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_SD_GNSS_MEASUREMENT_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_SD_CONTROL_POINT_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_CONTROL_POINT_WRITE_EVT */
      SensorData_Handle_SD_ControlPointWrite(
          pNotification->DataTransfered.pPayload,
          pNotification->DataTransfered.Length,
          pNotification->ConnectionHandle,
          Custom_App_Context.Sd_control_point_Indication_Status);
      /* USER CODE END CUSTOM_STM_SD_CONTROL_POINT_WRITE_EVT */
      break;

    case CUSTOM_STM_SD_CONTROL_POINT_INDICATE_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_CONTROL_POINT_INDICATE_ENABLED_EVT */
      Custom_App_Context.Sd_control_point_Indication_Status = 1;
      /* USER CODE END CUSTOM_STM_SD_CONTROL_POINT_INDICATE_ENABLED_EVT */
      break;

    case CUSTOM_STM_SD_CONTROL_POINT_INDICATE_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_CONTROL_POINT_INDICATE_DISABLED_EVT */
      Custom_App_Context.Sd_control_point_Indication_Status = 0;
      /* USER CODE END CUSTOM_STM_SD_CONTROL_POINT_INDICATE_DISABLED_EVT */
      break;

    /* Starter_Pistol */
    case CUSTOM_STM_SP_CONTROL_POINT_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SP_CONTROL_POINT_WRITE_EVT */
      FS_StartControl_Handle_SP_ControlPointWrite(
          pNotification->DataTransfered.pPayload,
          pNotification->DataTransfered.Length,
          pNotification->ConnectionHandle,
          Custom_App_Context.Sp_control_point_Indication_Status);
      /* USER CODE END CUSTOM_STM_SP_CONTROL_POINT_WRITE_EVT */
      break;

    case CUSTOM_STM_SP_CONTROL_POINT_INDICATE_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SP_CONTROL_POINT_INDICATE_ENABLED_EVT */
      Custom_App_Context.Sp_control_point_Indication_Status = 1;
      /* USER CODE END CUSTOM_STM_SP_CONTROL_POINT_INDICATE_ENABLED_EVT */
      break;

    case CUSTOM_STM_SP_CONTROL_POINT_INDICATE_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SP_CONTROL_POINT_INDICATE_DISABLED_EVT */
      Custom_App_Context.Sp_control_point_Indication_Status = 0;
      /* USER CODE END CUSTOM_STM_SP_CONTROL_POINT_INDICATE_DISABLED_EVT */
      break;

    case CUSTOM_STM_SP_RESULT_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SP_RESULT_READ_EVT */
      if (!Custom_App_Context.Sp_result_Indication_Status)
      {
        Custom_STM_App_Update_Char(CUSTOM_STM_SP_RESULT, start_result_packet);
      }
      /* USER CODE END CUSTOM_STM_SP_RESULT_READ_EVT */
      break;

    case CUSTOM_STM_SP_RESULT_INDICATE_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SP_RESULT_INDICATE_ENABLED_EVT */
      Custom_App_Context.Sp_result_Indication_Status = 1;
      /* USER CODE END CUSTOM_STM_SP_RESULT_INDICATE_ENABLED_EVT */
      break;

    case CUSTOM_STM_SP_RESULT_INDICATE_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SP_RESULT_INDICATE_DISABLED_EVT */
      Custom_App_Context.Sp_result_Indication_Status = 0;
      /* USER CODE END CUSTOM_STM_SP_RESULT_INDICATE_DISABLED_EVT */
      break;

    /* Device_State */
    case CUSTOM_STM_DS_MODE_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DS_MODE_READ_EVT */
      uint8_t modeVal = (uint8_t) FS_Mode_State();  // Get current state from mode module
      Custom_STM_App_Update_Char(CUSTOM_STM_DS_MODE, &modeVal);
      /* USER CODE END CUSTOM_STM_DS_MODE_READ_EVT */
      break;

    case CUSTOM_STM_DS_MODE_INDICATE_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DS_MODE_INDICATE_ENABLED_EVT */
      Custom_App_Context.Ds_mode_Indication_Status = 1;
      /* USER CODE END CUSTOM_STM_DS_MODE_INDICATE_ENABLED_EVT */
      break;

    case CUSTOM_STM_DS_MODE_INDICATE_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DS_MODE_INDICATE_DISABLED_EVT */
      Custom_App_Context.Ds_mode_Indication_Status = 0;
      /* USER CODE END CUSTOM_STM_DS_MODE_INDICATE_DISABLED_EVT */
      break;

    case CUSTOM_STM_DS_CONTROL_POINT_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DS_CONTROL_POINT_WRITE_EVT */
      DeviceState_Handle_DS_ControlPointWrite(
          pNotification->DataTransfered.pPayload,
          pNotification->DataTransfered.Length,
          pNotification->ConnectionHandle,
          Custom_App_Context.Ds_control_point_Indication_Status);
      /* USER CODE END CUSTOM_STM_DS_CONTROL_POINT_WRITE_EVT */
      break;

    case CUSTOM_STM_DS_CONTROL_POINT_INDICATE_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DS_CONTROL_POINT_INDICATE_ENABLED_EVT */
      Custom_App_Context.Ds_control_point_Indication_Status = 1;
      /* USER CODE END CUSTOM_STM_DS_CONTROL_POINT_INDICATE_ENABLED_EVT */
      break;

    case CUSTOM_STM_DS_CONTROL_POINT_INDICATE_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DS_CONTROL_POINT_INDICATE_DISABLED_EVT */
      Custom_App_Context.Ds_control_point_Indication_Status = 0;
      /* USER CODE END CUSTOM_STM_DS_CONTROL_POINT_INDICATE_DISABLED_EVT */
      break;

    /* Battery */
    case CUSTOM_STM_BATTERY_LEVEL_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BATTERY_LEVEL_READ_EVT */
      APP_DBG_MSG("CUSTOM_STM_BATTERY_LEVEL_READ_EVT received\n");
      // The actual calculation and update should happen when FS_VBAT_ValueReady_Callback runs,
      // or we need a way to trigger FS_VBAT_ValueReady_Callback or get the latest value here.
      // For simplicity, let's assume current_battery_level_percent is up-to-date.
      Custom_STM_App_Update_Char(CUSTOM_STM_BATTERY_LEVEL, &current_battery_level_percent);
      /* USER CODE END CUSTOM_STM_BATTERY_LEVEL_READ_EVT */
      break;

    case CUSTOM_STM_BATTERY_LEVEL_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BATTERY_LEVEL_NOTIFY_ENABLED_EVT */
      APP_DBG_MSG("CUSTOM_STM_BATTERY_LEVEL_NOTIFY_ENABLED_EVT received\n");
      Custom_App_Context.Battery_level_Notification_Status = 1;
      // Optionally, send the current level immediately upon enabling notifications
      Custom_STM_App_Update_Char(CUSTOM_STM_BATTERY_LEVEL, &current_battery_level_percent);
      /* USER CODE END CUSTOM_STM_BATTERY_LEVEL_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_BATTERY_LEVEL_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BATTERY_LEVEL_NOTIFY_DISABLED_EVT */
      APP_DBG_MSG("CUSTOM_STM_BATTERY_LEVEL_NOTIFY_DISABLED_EVT received\n");
      Custom_App_Context.Battery_level_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_BATTERY_LEVEL_NOTIFY_DISABLED_EVT */
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
      Custom_CRS_OnConnect(pNotification);
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
  BLE_TX_Queue_Init();

  SensorData_Init();
  DeviceState_Init();
  // Starter Pistol is initialized by the main mode logic when entering that mode.

  /* File_Transfer */
  Custom_App_Context.Ft_packet_out_Notification_Status = 0;

  /* Sensor_Data */
  Custom_App_Context.Sd_gnss_measurement_Notification_Status = 0;
  Custom_App_Context.Sd_control_point_Indication_Status = 0;

  /* Starter_Pistol */
  Custom_App_Context.Sp_control_point_Indication_Status = 0;
  Custom_App_Context.Sp_result_Indication_Status = 0;

  /* Device_State */
  Custom_App_Context.Ds_mode_Indication_Status = 0;
  Custom_App_Context.Ds_control_point_Indication_Status = 0;

  /* Battery Service */
  Custom_App_Context.Battery_level_Notification_Status = 0;
  current_battery_level_percent = 0;

  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timeout_timer_id, hw_ts_SingleShot, Custom_App_Timeout);
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
void Custom_APP_TxPoolAvailableNotification(void)
{
  BLE_TX_Queue_TxPoolAvailableNotification();
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

/* File_Transfer */
void Custom_Ft_packet_out_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Ft_packet_out_UC_1*/

  /* USER CODE END Ft_packet_out_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_FT_PACKET_OUT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Ft_packet_out_UC_Last*/

  /* USER CODE END Ft_packet_out_UC_Last*/
  return;
}

void Custom_Ft_packet_out_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Ft_packet_out_NS_1*/

  /* USER CODE END Ft_packet_out_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_FT_PACKET_OUT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Ft_packet_out_NS_Last*/

  /* USER CODE END Ft_packet_out_NS_Last*/

  return;
}

/* Sensor_Data */
void Custom_Sd_gnss_measurement_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_gnss_measurement_UC_1*/

  /* USER CODE END Sd_gnss_measurement_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_GNSS_MEASUREMENT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Sd_gnss_measurement_UC_Last*/

  /* USER CODE END Sd_gnss_measurement_UC_Last*/
  return;
}

void Custom_Sd_gnss_measurement_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_gnss_measurement_NS_1*/

  /* USER CODE END Sd_gnss_measurement_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_GNSS_MEASUREMENT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Sd_gnss_measurement_NS_Last*/

  /* USER CODE END Sd_gnss_measurement_NS_Last*/

  return;
}

void Custom_Sd_control_point_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_control_point_UC_1*/

  /* USER CODE END Sd_control_point_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_CONTROL_POINT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Sd_control_point_UC_Last*/

  /* USER CODE END Sd_control_point_UC_Last*/
  return;
}

void Custom_Sd_control_point_Send_Indication(void) /* Property Indication */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_control_point_IS_1*/

  /* USER CODE END Sd_control_point_IS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_CONTROL_POINT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Sd_control_point_IS_Last*/

  /* USER CODE END Sd_control_point_IS_Last*/

  return;
}

/* Starter_Pistol */
void Custom_Sp_control_point_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sp_control_point_UC_1*/

  /* USER CODE END Sp_control_point_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SP_CONTROL_POINT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Sp_control_point_UC_Last*/

  /* USER CODE END Sp_control_point_UC_Last*/
  return;
}

void Custom_Sp_control_point_Send_Indication(void) /* Property Indication */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sp_control_point_IS_1*/

  /* USER CODE END Sp_control_point_IS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SP_CONTROL_POINT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Sp_control_point_IS_Last*/

  /* USER CODE END Sp_control_point_IS_Last*/

  return;
}

void Custom_Sp_result_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sp_result_UC_1*/

  /* USER CODE END Sp_result_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SP_RESULT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Sp_result_UC_Last*/

  /* USER CODE END Sp_result_UC_Last*/
  return;
}

void Custom_Sp_result_Send_Indication(void) /* Property Indication */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sp_result_IS_1*/

  /* USER CODE END Sp_result_IS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SP_RESULT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Sp_result_IS_Last*/

  /* USER CODE END Sp_result_IS_Last*/

  return;
}

/* Device_State */
void Custom_Ds_mode_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Ds_mode_UC_1*/

  /* USER CODE END Ds_mode_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DS_MODE, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Ds_mode_UC_Last*/

  /* USER CODE END Ds_mode_UC_Last*/
  return;
}

void Custom_Ds_mode_Send_Indication(void) /* Property Indication */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Ds_mode_IS_1*/

  /* USER CODE END Ds_mode_IS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DS_MODE, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Ds_mode_IS_Last*/

  /* USER CODE END Ds_mode_IS_Last*/

  return;
}

void Custom_Ds_control_point_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Ds_control_point_UC_1*/

  /* USER CODE END Ds_control_point_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DS_CONTROL_POINT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Ds_control_point_UC_Last*/

  /* USER CODE END Ds_control_point_UC_Last*/
  return;
}

void Custom_Ds_control_point_Send_Indication(void) /* Property Indication */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Ds_control_point_IS_1*/

  /* USER CODE END Ds_control_point_IS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DS_CONTROL_POINT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Ds_control_point_IS_Last*/

  /* USER CODE END Ds_control_point_IS_Last*/

  return;
}

/* Battery */
void Custom_Battery_level_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Battery_level_UC_1*/

  /* USER CODE END Battery_level_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_BATTERY_LEVEL, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Battery_level_UC_Last*/

  /* USER CODE END Battery_level_UC_Last*/
  return;
}

void Custom_Battery_level_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Battery_level_NS_1*/

  /* USER CODE END Battery_level_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_BATTERY_LEVEL, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Battery_level_NS_Last*/

  /* USER CODE END Battery_level_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
static void Custom_CRS_OnConnect(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  // Reset buffer indices
  rx_read_index = 0;
  rx_write_index = 0;

  // Update state
  connected_flag = 1;

  // Remember connection handle
  Custom_App_Context.ConnectionHandle = pNotification->ConnectionHandle;

  // Start timeout timer
  HW_TS_Start(timeout_timer_id, TIMEOUT_TICKS);
}

static void Custom_CRS_OnDisconnect(void)
{
  // Stop timeout timer
  HW_TS_Stop(timeout_timer_id);

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

  // Refresh timeout timer
  HW_TS_Start(timeout_timer_id, TIMEOUT_TICKS);
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

void Custom_GNSS_Update(const FS_GNSS_Data_t *current)
{
  uint8_t length = GNSS_BLE_Build(current, gnss_pv_packet);

  if (Custom_App_Context.Sd_gnss_measurement_Notification_Status)
  {
    BLE_TX_Queue_SendTxPacket(CUSTOM_STM_SD_GNSS_MEASUREMENT,
        gnss_pv_packet, length, &SizeSd_Gnss_Measurement, 0);
  }
}

void Custom_Start_Update(uint16_t year, uint8_t month, uint8_t day,
                         uint8_t hour, uint8_t min, uint8_t sec, uint16_t ms)
{
  // Copy to packet
  memcpy(&start_result_packet[0], &year, sizeof(year));
  memcpy(&start_result_packet[2], &month, sizeof(month));
  memcpy(&start_result_packet[3], &day, sizeof(day));
  memcpy(&start_result_packet[4], &hour, sizeof(hour));
  memcpy(&start_result_packet[5], &min, sizeof(min));
  memcpy(&start_result_packet[6], &sec, sizeof(sec));
  memcpy(&start_result_packet[7], &ms, sizeof(ms));

  if (Custom_App_Context.Sp_result_Indication_Status)
  {
    BLE_TX_Queue_SendTxPacket(CUSTOM_STM_SP_RESULT,
        start_result_packet, SizeSp_Result, 0, 0);
  }
}

static void Custom_App_Timeout(void)
{
  aci_gap_terminate(Custom_App_Context.ConnectionHandle,
		  HCI_REMOTE_USER_TERMINATED_CONNECTION_ERR_CODE);
}

void Custom_Mode_Update(uint8_t newMode)
{
  /*
   * Even if Mode_Notification_Status == 0 (notifications off),
   * we still call BLE_TX_Queue_SendTxPacket so that the
   * characteristic value is always up-to-date when read.
   */
  BLE_TX_Queue_SendTxPacket(CUSTOM_STM_DS_MODE,
      &newMode, sizeof(newMode), &SizeDs_Mode, 0);
}

static uint8_t calculate_battery_percentage(uint16_t voltage_mv) {
  const uint16_t MIN_VOLTAGE_MV = 3300; // Example for LiPo empty
  const uint16_t MAX_VOLTAGE_MV = 4200; // Example for LiPo full
  int32_t percentage;

  if (voltage_mv <= MIN_VOLTAGE_MV) {
    percentage = 0;
  } else if (voltage_mv >= MAX_VOLTAGE_MV) {
    percentage = 100;
  } else {
    percentage = ((int32_t)voltage_mv - MIN_VOLTAGE_MV) * 100 / (MAX_VOLTAGE_MV - MIN_VOLTAGE_MV);
  }
  return (uint8_t)percentage;
}

void Custom_VBAT_Update(const FS_VBAT_Data_t *current)
{
  uint8_t new_battery_level = calculate_battery_percentage(current->voltage);

  // Update the global/context variable
  // This ensures that read requests get the most recent value
  current_battery_level_percent = new_battery_level;

  // If notifications are enabled for the Battery Level characteristic, send an update
  if (Custom_App_Context.Battery_level_Notification_Status)
  {
    // APP_DBG_MSG("Sending battery level notification: %d%%\n", new_battery_level);
    BLE_TX_Queue_SendTxPacket(CUSTOM_STM_BATTERY_LEVEL,
        &new_battery_level, sizeof(new_battery_level), &SizeBattery_Level, 0);
  }
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/
