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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "app_common.h"

#include "dbg_trace.h"
#include "ble.h"
#include "tl.h"
#include "app_ble.h"

#include "stm32_seq.h"
#include "shci.h"
#include "stm32_lpm.h"
#include "otp.h"

#include "custom_app.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "state.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/**
 * security parameters structure
 */
typedef struct _tSecurityParams
{
  /**
   * IO capability of the device
   */
  uint8_t ioCapability;

  /**
   * Authentication requirement of the device
   * Man In the Middle protection required?
   */
  uint8_t mitm_mode;

  /**
   * bonding mode of the device
   */
  uint8_t bonding_mode;

  /**
   * this variable indicates whether to use a fixed pin
   * during the pairing process or a passkey has to be
   * requested to the application during the pairing process
   * 0 implies use fixed pin and 1 implies request for passkey
   */
  uint8_t Use_Fixed_Pin;

  /**
   * minimum encryption key size requirement
   */
  uint8_t encryptionKeySizeMin;

  /**
   * maximum encryption key size requirement
   */
  uint8_t encryptionKeySizeMax;

  /**
   * fixed pin to be used in the pairing process if
   * Use_Fixed_Pin is set to 1
   */
  uint32_t Fixed_Pin;

  /**
   * this flag indicates whether the host has to initiate
   * the security, wait for pairing or does not have any security
   * requirements.
   * 0x00 : no security required
   * 0x01 : host should initiate security by sending the slave security
   *        request command
   * 0x02 : host need not send the clave security request but it
   * has to wait for paiirng to complete before doing any other
   * processing
   */
  uint8_t initiateSecurity;
  /* USER CODE BEGIN tSecurityParams*/

  /* USER CODE END tSecurityParams */
}tSecurityParams;

/**
 * global context
 * contains the variables common to all
 * services
 */
typedef struct _tBLEProfileGlobalContext
{
  /**
   * security requirements of the host
   */
  tSecurityParams bleSecurityParam;

  /**
   * gap service handle
   */
  uint16_t gapServiceHandle;

  /**
   * device name characteristic handle
   */
  uint16_t devNameCharHandle;

  /**
   * appearance characteristic handle
   */
  uint16_t appearanceCharHandle;

  /**
   * connection handle of the current active connection
   * When not in connection, the handle is set to 0xFFFF
   */
  uint16_t connectionHandle;

  /**
   * length of the UUID list to be used while advertising
   */
  uint8_t advtServUUIDlen;

  /**
   * the UUID list to be used while advertising
   */
  uint8_t advtServUUID[100];
  /* USER CODE BEGIN BleGlobalContext_t*/

  /* USER CODE END BleGlobalContext_t */
}BleGlobalContext_t;

typedef struct
{
  BleGlobalContext_t BleApplicationContext_legacy;
  APP_BLE_ConnStatus_t Device_Connection_Status;
  /* USER CODE BEGIN PTD_1*/
  uint8_t Advertising_mgr_timer_Id;
  /* USER CODE END PTD_1 */
}BleApplicationContext_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define FAST_ADV_TIMEOUT               (30*1000*1000/CFG_TS_TICK_VAL) /**< 30s */
#define INITIAL_ADV_TIMEOUT            (60*1000*1000/CFG_TS_TICK_VAL) /**< 60s */

#define BD_ADDR_SIZE_LOCAL    6

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

/**
 *   Identity root key used to derive IRK and DHK(Legacy)
 */
static const uint8_t a_BLE_CfgIrValue[16] = CFG_BLE_IR;

/**
 * Encryption root key used to derive LTK(Legacy) and CSRK
 */
static const uint8_t a_BLE_CfgErValue[16] = CFG_BLE_ER;

/**
 * These are the two tags used to manage a power failure during OTA
 * The MagicKeywordAdress shall be mapped @0x140 from start of the binary image
 * The MagicKeywordvalue is checked in the ble_ota application
 */
PLACE_IN_SECTION("TAG_OTA_END") const uint32_t MagicKeywordValue = 0x94448A29 ;
PLACE_IN_SECTION("TAG_OTA_START") const uint32_t MagicKeywordAddress = (uint32_t)&MagicKeywordValue;

static BleApplicationContext_t BleApplicationContext;

Custom_App_ConnHandle_Not_evt_t HandleNotification;

#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
#define SIZE_TAB_CONN_INT            2
float a_ConnInterval[SIZE_TAB_CONN_INT] = {50, 1000}; /* ms */
uint8_t index_con_int, mutex;
#endif /* L2CAP_REQUEST_NEW_CONN_PARAM != 0 */

/**
 * Advertising Data
 */
uint8_t a_AdvData[15] =
{
  9, AD_TYPE_COMPLETE_LOCAL_NAME, 'F', 'l', 'y', 'S', 'i', 'g', 'h', 't',  /* Complete name */
  4, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 0xDB, 0x09, 0x00 /* Structure version */,
};

/* USER CODE BEGIN PV */
/* Advertising callback */
void (*Adv_Callback)(void) = 0;
void (*Next_Adv_Callback)(void) = 0;

/* Pairing request flag */
uint8_t request_pairing = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void BLE_UserEvtRx(void *p_Payload);
static void BLE_StatusNot(HCI_TL_CmdStatus_t Status);
static void Ble_Tl_Init(void);
static void Ble_Hci_Gap_Gatt_Init(void);
static void Adv_Request(APP_BLE_ConnStatus_t NewStatus);
static void Adv_Cancel(void);
#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
static void BLE_SVC_L2CAP_Conn_Update(uint16_t ConnectionHandle);
static void Connection_Interval_Update_Req(void);
#endif /* L2CAP_REQUEST_NEW_CONN_PARAM != 0 */

/* USER CODE BEGIN PFP */
static void LinkConfiguration(void);
static void FS_Adv_Request(APP_BLE_ConnStatus_t NewStatus);
static void Adv_Update_Req(void);
static void Adv_Update(void);
static void APP_BLE_UpdateAdvertisingData(APP_BLE_ConnStatus_t NewStatus);
static int8_t ble_count_bonded_devices(void);
/* USER CODE END PFP */

/* External variables --------------------------------------------------------*/
extern RNG_HandleTypeDef hrng;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Functions Definition ------------------------------------------------------*/
void APP_BLE_Init(void)
{
  SHCI_CmdStatus_t status;
#if (RADIO_ACTIVITY_EVENT != 0)
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
#endif /* RADIO_ACTIVITY_EVENT != 0 */
  /* USER CODE BEGIN APP_BLE_Init_1 */

  /* USER CODE END APP_BLE_Init_1 */
  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
  {
    {{0,0,0}},                          /**< Header unused */
    {0,                                 /** pBleBufferAddress not used */
     0,                                 /** BleBufferSize not used */
     CFG_BLE_NUM_GATT_ATTRIBUTES,
     CFG_BLE_NUM_GATT_SERVICES,
     CFG_BLE_ATT_VALUE_ARRAY_SIZE,
     CFG_BLE_NUM_LINK,
     CFG_BLE_DATA_LENGTH_EXTENSION,
     CFG_BLE_PREPARE_WRITE_LIST_SIZE,
     CFG_BLE_MBLOCK_COUNT,
     CFG_BLE_MAX_ATT_MTU,
     CFG_BLE_PERIPHERAL_SCA,
     CFG_BLE_CENTRAL_SCA,
     CFG_BLE_LS_SOURCE,
     CFG_BLE_MAX_CONN_EVENT_LENGTH,
     CFG_BLE_HSE_STARTUP_TIME,
     CFG_BLE_VITERBI_MODE,
     CFG_BLE_OPTIONS,
     0,
     CFG_BLE_MAX_COC_INITIATOR_NBR,
     CFG_BLE_MIN_TX_POWER,
     CFG_BLE_MAX_TX_POWER,
     CFG_BLE_RX_MODEL_CONFIG,
     CFG_BLE_MAX_ADV_SET_NBR,
     CFG_BLE_MAX_ADV_DATA_LEN,
     CFG_BLE_TX_PATH_COMPENS,
     CFG_BLE_RX_PATH_COMPENS,
     CFG_BLE_CORE_VERSION,
     CFG_BLE_OPTIONS_EXT
    }
  };

  /**
   * Initialize Ble Transport Layer
   */
  Ble_Tl_Init();

  /**
   * Do not allow standby in the application
   */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

  /**
   * Register the hci transport layer to handle BLE User Asynchronous Events
   */
  UTIL_SEQ_RegTask(1<<CFG_TASK_HCI_ASYNCH_EVT_ID, UTIL_SEQ_RFU, hci_user_evt_proc);

  /**
   * Starts the BLE Stack on CPU2
   */
  status = SHCI_C2_BLE_Init(&ble_init_cmd_packet);
  if (status != SHCI_Success)
  {
    APP_DBG_MSG("  Fail   : SHCI_C2_BLE_Init command, result: 0x%02x\n\r", status);
    /* if you are here, maybe CPU2 doesn't contain STM32WB_Copro_Wireless_Binaries, see Release_Notes.html */
    Error_Handler();
  }
  else
  {
    APP_DBG_MSG("  Success: SHCI_C2_BLE_Init command\n\r");
  }

  /**
   * Initialization of HCI & GATT & GAP layer
   */
  Ble_Hci_Gap_Gatt_Init();

  /**
   * Initialization of the BLE Services
   */
  SVCCTL_Init();

  /**
   * Initialization of the BLE App Context
   */
  BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
  BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0xFFFF;

  /**
   * From here, all initialization are BLE application specific
   */

  UTIL_SEQ_RegTask(1<<CFG_TASK_ADV_CANCEL_ID, UTIL_SEQ_RFU, Adv_Cancel);

  /* USER CODE BEGIN APP_BLE_Init_4 */
  UTIL_SEQ_RegTask(1<<CFG_TASK_LINK_CONFIG_ID, UTIL_SEQ_RFU, LinkConfiguration);
  UTIL_SEQ_RegTask(1<<CFG_TASK_ADV_UPDATE_ID, UTIL_SEQ_RFU, Adv_Update);
  /* USER CODE END APP_BLE_Init_4 */

  /**
   * Initialization of ADV - Ad Manufacturer Element - Support OTA Bit Mask
   */
#if (RADIO_ACTIVITY_EVENT != 0)
  ret = aci_hal_set_radio_activity_mask(0x0006);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_set_radio_activity_mask command, result: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_set_radio_activity_mask command\n\r");
  }
#endif /* RADIO_ACTIVITY_EVENT != 0 */

#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
  index_con_int = 0;
  mutex = 1;
#endif /* L2CAP_REQUEST_NEW_CONN_PARAM != 0 */

  /**
   * Initialize Custom Template Application
   */
  Custom_APP_Init();

  /* USER CODE BEGIN APP_BLE_Init_3 */
  /* Create timer to handle the connection state machine */
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.Advertising_mgr_timer_Id), hw_ts_SingleShot, Adv_Update_Req);
#if 0
  /* USER CODE END APP_BLE_Init_3 */

  /**
   * Make device discoverable
   */
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = NULL;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 0;

  /**
   * Start to Advertise to be connected by a Client
   */
  Adv_Request(APP_BLE_FAST_ADV);

  /* USER CODE BEGIN APP_BLE_Init_2 */
#endif
  /**
   * Make device discoverable
   */
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = NULL;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 0;

  if (FS_State_Get()->enable_ble)
  {
    /**
     * Start to Advertise to be connected by a Client
     */
    FS_Adv_Request(APP_BLE_FAST_ADV);
  }
  /* USER CODE END APP_BLE_Init_2 */

  return;
}

SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification(void *p_Pckt)
{
  hci_event_pckt    *p_event_pckt;
  evt_le_meta_event *p_meta_evt;
  evt_blecore_aci   *p_blecore_evt;
  tBleStatus        ret = BLE_STATUS_INVALID_PARAMS;
  hci_le_enhanced_connection_complete_event_rp0 *p_enhanced_connection_complete_event;
  hci_disconnection_complete_event_rp0        *p_disconnection_complete_event;
#if (CFG_DEBUG_APP_TRACE != 0)
  hci_le_connection_update_complete_event_rp0 *p_connection_update_complete_event;
#endif /* CFG_DEBUG_APP_TRACE != 0 */

  /* PAIRING */
  aci_gap_pairing_complete_event_rp0          *p_pairing_complete;
  /* PAIRING */

  /* USER CODE BEGIN SVCCTL_App_Notification */

  /* USER CODE END SVCCTL_App_Notification */

  p_event_pckt = (hci_event_pckt*) ((hci_uart_pckt *) p_Pckt)->data;

  switch (p_event_pckt->evt)
  {
    case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
    {
      p_disconnection_complete_event = (hci_disconnection_complete_event_rp0 *) p_event_pckt->data;

      if (p_disconnection_complete_event->Connection_Handle == BleApplicationContext.BleApplicationContext_legacy.connectionHandle)
      {
        BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0;
        BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
        APP_DBG_MSG(">>== HCI_DISCONNECTION_COMPLETE_EVT_CODE\n");
        APP_DBG_MSG("     - Connection Handle:   0x%x\n     - Reason:    0x%x\n\r",
                    p_disconnection_complete_event->Connection_Handle,
                    p_disconnection_complete_event->Reason);

        /* USER CODE BEGIN EVT_DISCONN_COMPLETE_2 */

        /* USER CODE END EVT_DISCONN_COMPLETE_2 */
      }

      /* USER CODE BEGIN EVT_DISCONN_COMPLETE_1 */
#if 0
      /* USER CODE END EVT_DISCONN_COMPLETE_1 */

      /* restart advertising */
      Adv_Request(APP_BLE_FAST_ADV);

      /**
       * SPECIFIC to Custom Template APP
       */
      HandleNotification.Custom_Evt_Opcode = CUSTOM_DISCON_HANDLE_EVT;
      HandleNotification.ConnectionHandle = BleApplicationContext.BleApplicationContext_legacy.connectionHandle;
      Custom_APP_Notification(&HandleNotification);
      /* USER CODE BEGIN EVT_DISCONN_COMPLETE */
#endif
      if (FS_State_Get()->enable_ble)
      {
        /* restart advertising */
        FS_Adv_Request(APP_BLE_FAST_ADV);
      }

      /**
       * SPECIFIC to Custom Template APP
       */
      HandleNotification.Custom_Evt_Opcode = CUSTOM_DISCON_HANDLE_EVT;
      HandleNotification.ConnectionHandle = BleApplicationContext.BleApplicationContext_legacy.connectionHandle;
      Custom_APP_Notification(&HandleNotification);
      /* USER CODE END EVT_DISCONN_COMPLETE */
      break; /* HCI_DISCONNECTION_COMPLETE_EVT_CODE */
    }

    case HCI_LE_META_EVT_CODE:
    {
      p_meta_evt = (evt_le_meta_event*) p_event_pckt->data;
      /* USER CODE BEGIN EVT_LE_META_EVENT */

      /* USER CODE END EVT_LE_META_EVENT */
      switch (p_meta_evt->subevent)
      {
        case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
#if (CFG_DEBUG_APP_TRACE != 0)
          p_connection_update_complete_event = (hci_le_connection_update_complete_event_rp0 *) p_meta_evt->data;
          APP_DBG_MSG(">>== HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE\n");
          APP_DBG_MSG("     - Connection Interval:   %.2f ms\n     - Connection latency:    %d\n     - Supervision Timeout: %d ms\n\r",
                       p_connection_update_complete_event->Conn_Interval*1.25,
                       p_connection_update_complete_event->Conn_Latency,
                       p_connection_update_complete_event->Supervision_Timeout*10);
#endif /* CFG_DEBUG_APP_TRACE != 0 */

          /* USER CODE BEGIN EVT_LE_CONN_UPDATE_COMPLETE */

          /* USER CODE END EVT_LE_CONN_UPDATE_COMPLETE */
          break;

        case HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE:
        {
          p_enhanced_connection_complete_event = (hci_le_enhanced_connection_complete_event_rp0 *) p_meta_evt->data;
          /**
           * The connection is done, there is no need anymore to schedule the LP ADV
           */

          APP_DBG_MSG(">>== HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE - Connection handle: 0x%x\n", p_enhanced_connection_complete_event->Connection_Handle);
          APP_DBG_MSG("     - Connection established with Central: @:%02x:%02x:%02x:%02x:%02x:%02x\n",
                      p_enhanced_connection_complete_event->Peer_Address[5],
                      p_enhanced_connection_complete_event->Peer_Address[4],
                      p_enhanced_connection_complete_event->Peer_Address[3],
                      p_enhanced_connection_complete_event->Peer_Address[2],
                      p_enhanced_connection_complete_event->Peer_Address[1],
                      p_enhanced_connection_complete_event->Peer_Address[0]);
          APP_DBG_MSG("     - Connection Interval:   %.2f ms\n     - Connection latency:    %d\n     - Supervision Timeout: %d ms\n\r",
                      p_enhanced_connection_complete_event->Conn_Interval*1.25,
                      p_enhanced_connection_complete_event->Conn_Latency,
                      p_enhanced_connection_complete_event->Supervision_Timeout*10
                     );

          if (BleApplicationContext.Device_Connection_Status == APP_BLE_LP_CONNECTING)
          {
            /* Connection as client */
            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_CLIENT;
          }
          else
          {
            /* Connection as server */
            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_SERVER;
          }
          BleApplicationContext.BleApplicationContext_legacy.connectionHandle = p_enhanced_connection_complete_event->Connection_Handle;
          /**
           * SPECIFIC to Custom Template APP
           */
          HandleNotification.Custom_Evt_Opcode = CUSTOM_CONN_HANDLE_EVT;
          HandleNotification.ConnectionHandle = BleApplicationContext.BleApplicationContext_legacy.connectionHandle;
          Custom_APP_Notification(&HandleNotification);
          /* USER CODE BEGIN HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE */
          /* Stop the timer */
          HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);

          /* Call advertising callback */
          if (Adv_Callback) Adv_Callback();

          /* Update advertising callback */
          Adv_Callback = Next_Adv_Callback;
          Next_Adv_Callback = 0;

          /* Configure the link */
          UTIL_SEQ_SetTask(1 << CFG_TASK_LINK_CONFIG_ID, CFG_SCH_PRIO_1);
          /* USER CODE END HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE */
          break; /* HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE */
        }

        default:
          /* USER CODE BEGIN SUBEVENT_DEFAULT */

          /* USER CODE END SUBEVENT_DEFAULT */
          break;
      }

      /* USER CODE BEGIN META_EVT */

      /* USER CODE END META_EVT */
      break; /* HCI_LE_META_EVT_CODE */
    }

    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      p_blecore_evt = (evt_blecore_aci*) p_event_pckt->data;
      /* USER CODE BEGIN EVT_VENDOR */

      /* USER CODE END EVT_VENDOR */
      switch (p_blecore_evt->ecode)
      {
        /* USER CODE BEGIN ecode */

        /* USER CODE END ecode */

        /**
         * SPECIFIC to Custom Template APP
         */
        case ACI_L2CAP_CONNECTION_UPDATE_RESP_VSEVT_CODE:
#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
          mutex = 1;
#endif /* L2CAP_REQUEST_NEW_CONN_PARAM != 0 */
          /* USER CODE BEGIN EVT_BLUE_L2CAP_CONNECTION_UPDATE_RESP */

          /* USER CODE END EVT_BLUE_L2CAP_CONNECTION_UPDATE_RESP */
          break;

        case ACI_GAP_PROC_COMPLETE_VSEVT_CODE:
          APP_DBG_MSG(">>== ACI_GAP_PROC_COMPLETE_VSEVT_CODE \r");
          /* USER CODE BEGIN EVT_BLUE_GAP_PROCEDURE_COMPLETE */

          /* USER CODE END EVT_BLUE_GAP_PROCEDURE_COMPLETE */
          break; /* ACI_GAP_PROC_COMPLETE_VSEVT_CODE */

#if (RADIO_ACTIVITY_EVENT != 0)
        case ACI_HAL_END_OF_RADIO_ACTIVITY_VSEVT_CODE:
          /* USER CODE BEGIN RADIO_ACTIVITY_EVENT*/

          /* USER CODE END RADIO_ACTIVITY_EVENT*/
          break; /* ACI_HAL_END_OF_RADIO_ACTIVITY_VSEVT_CODE */
#endif /* RADIO_ACTIVITY_EVENT != 0 */

        /* PAIRING */
        case (ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE):
          APP_DBG_MSG(">>== ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE\n");
          /* USER CODE BEGIN ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE*/

          /* USER CODE END ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE*/
          break;

        case ACI_GAP_PASS_KEY_REQ_VSEVT_CODE:
          APP_DBG_MSG(">>== ACI_GAP_PASS_KEY_REQ_VSEVT_CODE \n");

          ret = aci_gap_pass_key_resp(BleApplicationContext.BleApplicationContext_legacy.connectionHandle, CFG_FIXED_PIN);
          if (ret != BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("==>> aci_gap_pass_key_resp : Fail, reason: 0x%x\n", ret);
          }
          else
          {
            APP_DBG_MSG("==>> aci_gap_pass_key_resp : Success \n");
          }
          /* USER CODE BEGIN ACI_GAP_PASS_KEY_REQ_VSEVT_CODE*/

          /* USER CODE END ACI_GAP_PASS_KEY_REQ_VSEVT_CODE*/
          break;

        case ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE:
          APP_DBG_MSG(">>== ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE\n");
          APP_DBG_MSG("     - numeric_value = %ld\n",
                      ((aci_gap_numeric_comparison_value_event_rp0 *)(p_blecore_evt->data))->Numeric_Value);
          APP_DBG_MSG("     - Hex_value = %lx\n",
                      ((aci_gap_numeric_comparison_value_event_rp0 *)(p_blecore_evt->data))->Numeric_Value);
          ret = aci_gap_numeric_comparison_value_confirm_yesno(BleApplicationContext.BleApplicationContext_legacy.connectionHandle, YES);
          if (ret != BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("==>> aci_gap_numeric_comparison_value_confirm_yesno-->YES : Fail, reason: 0x%x\n", ret);
          }
          else
          {
            APP_DBG_MSG("==>> aci_gap_numeric_comparison_value_confirm_yesno-->YES : Success \n");
          }
          /* USER CODE BEGIN ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE*/

          /* USER CODE END ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE*/
          break;

        case ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE:
          p_pairing_complete = (aci_gap_pairing_complete_event_rp0*)p_blecore_evt->data;

          APP_DBG_MSG(">>== ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE\n");
          if (p_pairing_complete->Status != 0)
          {
            APP_DBG_MSG("     - Pairing KO \n     - Status: 0x%x\n     - Reason: 0x%x\n", p_pairing_complete->Status, p_pairing_complete->Reason);
          }
          else
          {
            APP_DBG_MSG("     - Pairing Success\n");
          }
          APP_DBG_MSG("\n");

          /* USER CODE BEGIN ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE*/

          /* USER CODE END ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE*/
          break;
        /* PAIRING */
        case ACI_GATT_INDICATION_VSEVT_CODE:
        {
          APP_DBG_MSG(">>== ACI_GATT_INDICATION_VSEVT_CODE \r");
          aci_gatt_confirm_indication(BleApplicationContext.BleApplicationContext_legacy.connectionHandle);
        }
        break;

        /* USER CODE BEGIN BLUE_EVT */
        case ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE:
          Custom_APP_TxPoolAvailableNotification();
          break;
        /* USER CODE END BLUE_EVT */
      }
      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT */

      /* USER CODE END EVENT_PCKT */

    default:
      /* USER CODE BEGIN ECODE_DEFAULT*/

      /* USER CODE END ECODE_DEFAULT*/
      break;
  }

  return (SVCCTL_UserEvtFlowEnable);
}

APP_BLE_ConnStatus_t APP_BLE_Get_Server_Connection_Status(void)
{
  return BleApplicationContext.Device_Connection_Status;
}

/* USER CODE BEGIN FD*/

/* USER CODE END FD*/

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
static void Ble_Tl_Init(void)
{
  HCI_TL_HciInitConf_t Hci_Tl_Init_Conf;

  Hci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&BleCmdBuffer;
  Hci_Tl_Init_Conf.StatusNotCallBack = BLE_StatusNot;
  hci_init(BLE_UserEvtRx, (void*) &Hci_Tl_Init_Conf);

  return;
}

static void Ble_Hci_Gap_Gatt_Init(void)
{
  uint8_t role;
  uint16_t gap_service_handle, gap_dev_name_char_handle, gap_appearance_char_handle;
  uint32_t a_srd_bd_addr[2] = {0,0};
  uint16_t a_appearance[1] = {BLE_CFG_GAP_APPEARANCE};
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Ble_Hci_Gap_Gatt_Init*/

  /* USER CODE END Ble_Hci_Gap_Gatt_Init*/

  APP_DBG_MSG("==>> Start Ble_Hci_Gap_Gatt_Init function\n");

  /**
   * Initialize HCI layer
   */
  /*HCI Reset to synchronise BLE Stack*/
  ret = hci_reset();
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : hci_reset command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: hci_reset command\n");
  }

  /**
   * Static random Address
   * The two upper bits shall be set to 1
   * The lowest 32bits is read from the UDN to differentiate between devices
   * The RNG may be used to provide a random number on each power on
   */
#if (CFG_IDENTITY_ADDRESS == GAP_STATIC_RANDOM_ADDR)
#if defined(CFG_STATIC_RANDOM_ADDRESS)
  a_srd_bd_addr[0] = CFG_STATIC_RANDOM_ADDRESS & 0xFFFFFFFF;
  a_srd_bd_addr[1] = (uint32_t)((uint64_t)CFG_STATIC_RANDOM_ADDRESS >> 32);
  a_srd_bd_addr[1] |= 0xC000; /* The two upper bits shall be set to 1 */
#else
  FS_Common_GetRandomBytes(a_srd_bd_addr, 2);
  a_srd_bd_addr[1] |= 0xC000; /* The two upper bits shall be set to 1 */
#endif /* CFG_STATIC_RANDOM_ADDRESS */
#endif

#if (CFG_BLE_ADDRESS_TYPE == GAP_STATIC_RANDOM_ADDR)
#endif

#if (CFG_BLE_ADDRESS_TYPE != PUBLIC_ADDR)
  ret = aci_hal_write_config_data(CONFIG_DATA_RANDOM_ADDRESS_OFFSET, CONFIG_DATA_RANDOM_ADDRESS_LEN, (uint8_t*)a_srd_bd_addr);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_RANDOM_ADDRESS_OFFSET, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_RANDOM_ADDRESS_OFFSET\n");
    APP_DBG_MSG("  Random Bluetooth Address: %02x:%02x:%02x:%02x:%02x:%02x\n", (uint8_t)(a_srd_bd_addr[1] >> 8),
                                                                               (uint8_t)(a_srd_bd_addr[1]),
                                                                               (uint8_t)(a_srd_bd_addr[0] >> 24),
                                                                               (uint8_t)(a_srd_bd_addr[0] >> 16),
                                                                               (uint8_t)(a_srd_bd_addr[0] >> 8),
                                                                               (uint8_t)(a_srd_bd_addr[0]));
  }
#endif /* CFG_BLE_ADDRESS_TYPE != GAP_PUBLIC_ADDR */

  /**
   * Write Identity root key used to derive IRK and DHK(Legacy)
   */
  ret = aci_hal_write_config_data(CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, (uint8_t*)a_BLE_CfgIrValue);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_IR_OFFSET, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_IR_OFFSET\n");
  }

  /**
   * Write Encryption root key used to derive LTK and CSRK
   */
  ret = aci_hal_write_config_data(CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, (uint8_t*)a_BLE_CfgErValue);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_ER_OFFSET, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_ER_OFFSET\n");
  }

  /**
   * Set TX Power.
   */
  ret = aci_hal_set_tx_power_level(1, CFG_TX_POWER);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_set_tx_power_level command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_set_tx_power_level command\n");
  }

  /**
   * Initialize GATT interface
   */
  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_init command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_init command\n");
  }

  /**
   * Initialize GAP interface
   */
  role = 0;

#if (BLE_CFG_PERIPHERAL == 1)
  role |= GAP_PERIPHERAL_ROLE;
#endif /* BLE_CFG_PERIPHERAL == 1 */

#if (BLE_CFG_CENTRAL == 1)
  role |= GAP_CENTRAL_ROLE;
#endif /* BLE_CFG_CENTRAL == 1 */

/* USER CODE BEGIN Role_Mngt*/

/* USER CODE END Role_Mngt */

  if (role > 0)
  {
    const char *name = CFG_GAP_DEVICE_NAME;
    ret = aci_gap_init(role,
                       CFG_PRIVACY,
                       CFG_GAP_DEVICE_NAME_LENGTH,
                       &gap_service_handle,
                       &gap_dev_name_char_handle,
                       &gap_appearance_char_handle);

    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gap_init command, result: 0x%x \n", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gap_init command\n");
    }

    ret = aci_gatt_update_char_value(gap_service_handle, gap_dev_name_char_handle, 0, strlen(name), (uint8_t *) name);
    if (ret != BLE_STATUS_SUCCESS)
    {
      BLE_DBG_SVCCTL_MSG("  Fail   : aci_gatt_update_char_value - Device Name\n");
    }
    else
    {
      BLE_DBG_SVCCTL_MSG("  Success: aci_gatt_update_char_value - Device Name\n");
    }
  }

  ret = aci_gatt_update_char_value(gap_service_handle,
                                   gap_appearance_char_handle,
                                   0,
                                   2,
                                   (uint8_t *)&a_appearance);
  if (ret != BLE_STATUS_SUCCESS)
  {
    BLE_DBG_SVCCTL_MSG("  Fail   : aci_gatt_update_char_value - Appearance\n");
  }
  else
  {
    BLE_DBG_SVCCTL_MSG("  Success: aci_gatt_update_char_value - Appearance\n");
  }

  /**
   * Initialize Default PHY
   */
  ret = hci_le_set_default_phy(ALL_PHYS_PREFERENCE,TX_2M_PREFERRED,RX_2M_PREFERRED);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : hci_le_set_default_phy command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: hci_le_set_default_phy command\n");
  }

  /**
   * Initialize IO capability
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability = CFG_IO_CAPABILITY;
  ret = aci_gap_set_io_capability(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gap_set_io_capability command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gap_set_io_capability command\n");
  }

  /**
   * Initialize authentication
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode = CFG_MITM_PROTECTION;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin = CFG_ENCRYPTION_KEY_SIZE_MIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax = CFG_ENCRYPTION_KEY_SIZE_MAX;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin = CFG_USED_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin = CFG_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode = CFG_BONDING_MODE;
  /* USER CODE BEGIN Ble_Hci_Gap_Gatt_Init_1*/
  BleApplicationContext.BleApplicationContext_legacy.gapServiceHandle = gap_service_handle;
  BleApplicationContext.BleApplicationContext_legacy.devNameCharHandle = gap_dev_name_char_handle;
  BleApplicationContext.BleApplicationContext_legacy.appearanceCharHandle = gap_appearance_char_handle;
  APP_BLE_UpdateDeviceName();
  /* USER CODE END Ble_Hci_Gap_Gatt_Init_1*/

  ret = aci_gap_set_authentication_requirement(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
                                               CFG_SC_SUPPORT,
                                               CFG_KEYPRESS_NOTIFICATION_SUPPORT,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin,
                                               CFG_IDENTITY_ADDRESS);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gap_set_authentication_requirement command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gap_set_authentication_requirement command\n");
  }

  /**
   * Initialize whitelist
   */
  if (BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode)
  {
    ret = aci_gap_configure_whitelist();
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gap_configure_whitelist command, result: 0x%x \n", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gap_configure_whitelist command\n");
    }
  }
  APP_DBG_MSG("==>> End Ble_Hci_Gap_Gatt_Init function\n\r");
}

static void Adv_Request(APP_BLE_ConnStatus_t NewStatus)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  BleApplicationContext.Device_Connection_Status = NewStatus;
  /* Start Fast or Low Power Advertising */
  ret = aci_gap_set_discoverable(ADV_TYPE,
                                 CFG_FAST_CONN_ADV_INTERVAL_MIN,
                                 CFG_FAST_CONN_ADV_INTERVAL_MAX,
                                 CFG_BLE_ADDRESS_TYPE,
                                 ADV_FILTER,
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 0);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("==>> aci_gap_set_discoverable - fail, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("==>> aci_gap_set_discoverable - Success\n");
  }

/* USER CODE BEGIN Adv_Request_1*/

/* USER CODE END Adv_Request_1*/

  /* Update Advertising data */
  ret = aci_gap_update_adv_data(sizeof(a_AdvData), (uint8_t*) a_AdvData);
  if (ret != BLE_STATUS_SUCCESS)
  {
      APP_DBG_MSG("==>> Start Fast Advertising Failed , result: %d \n\r", ret);
  }
  else
  {
      APP_DBG_MSG("==>> Success: Start Fast Advertising \n\r");
  }

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTION */

/* USER CODE END FD_LOCAL_FUNCTION */

/*************************************************************
 *
 * SPECIFIC FUNCTIONS FOR CUSTOM
 *
 *************************************************************/
static void Adv_Cancel(void)
{
  /* USER CODE BEGIN Adv_Cancel_1 */

  /* USER CODE END Adv_Cancel_1 */

  if (BleApplicationContext.Device_Connection_Status != APP_BLE_CONNECTED_SERVER)
  {
    tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

    ret = aci_gap_set_non_discoverable();

    BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("** STOP ADVERTISING **  Failed \r\n\r");
    }
    else
    {
      APP_DBG_MSG("  \r\n\r");
      APP_DBG_MSG("** STOP ADVERTISING **  \r\n\r");
    }
  }

  /* USER CODE BEGIN Adv_Cancel_2 */

  /* USER CODE END Adv_Cancel_2 */

  return;
}

#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
void BLE_SVC_L2CAP_Conn_Update(uint16_t ConnectionHandle)
{
  /* USER CODE BEGIN BLE_SVC_L2CAP_Conn_Update_1 */

  /* USER CODE END BLE_SVC_L2CAP_Conn_Update_1 */

  if (mutex == 1)
  {
    mutex = 0;
    index_con_int = (index_con_int + 1)%SIZE_TAB_CONN_INT;
    uint16_t interval_min = CONN_P(a_ConnInterval[index_con_int]);
    uint16_t interval_max = CONN_P(a_ConnInterval[index_con_int]);
    uint16_t peripheral_latency = L2CAP_PERIPHERAL_LATENCY;
    uint16_t timeout_multiplier = L2CAP_TIMEOUT_MULTIPLIER;
    tBleStatus ret;

    ret = aci_l2cap_connection_parameter_update_req(BleApplicationContext.BleApplicationContext_legacy.connectionHandle,
                                                    interval_min, interval_max,
                                                    peripheral_latency, timeout_multiplier);
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("BLE_SVC_L2CAP_Conn_Update(), Failed \r\n\r");
    }
    else
    {
      APP_DBG_MSG("BLE_SVC_L2CAP_Conn_Update(), Successfully \r\n\r");
    }
  }

  /* USER CODE BEGIN BLE_SVC_L2CAP_Conn_Update_2 */

  /* USER CODE END BLE_SVC_L2CAP_Conn_Update_2 */

  return;
}
#endif /* L2CAP_REQUEST_NEW_CONN_PARAM != 0 */

#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
static void Connection_Interval_Update_Req(void)
{
  if (BleApplicationContext.Device_Connection_Status != APP_BLE_FAST_ADV && BleApplicationContext.Device_Connection_Status != APP_BLE_IDLE)
  {
    BLE_SVC_L2CAP_Conn_Update(BleApplicationContext.BleApplicationContext_legacy.connectionHandle);
  }

  return;
}
#endif /* L2CAP_REQUEST_NEW_CONN_PARAM != 0 */

/* USER CODE BEGIN FD_SPECIFIC_FUNCTIONS */
static void LinkConfiguration(void)
{
  tBleStatus status;

  /* See AN5289: How to maximize data throughput */
  APP_DBG_MSG("set data length \n");
  status = hci_le_set_data_length(BleApplicationContext.BleApplicationContext_legacy.connectionHandle,251,2120);
  if (status != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("set data length command error \n");
  }
}

static void FS_Adv_Request(APP_BLE_ConnStatus_t NewStatus)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint16_t Min_Inter, Max_Inter;

  if (NewStatus == APP_BLE_FAST_ADV)
  {
    Min_Inter = CFG_FAST_CONN_ADV_INTERVAL_MIN;
    Max_Inter = CFG_FAST_CONN_ADV_INTERVAL_MAX;
  }
  else
  {
    Min_Inter = CFG_LP_CONN_ADV_INTERVAL_MIN;
    Max_Inter = CFG_LP_CONN_ADV_INTERVAL_MAX;
  }

  /**
   * Stop the timer, it will be restarted for a new shot
   * It does not hurt if the timer was not running
   */
  HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);

  if ((BleApplicationContext.Device_Connection_Status == APP_BLE_FAST_ADV)
      || (BleApplicationContext.Device_Connection_Status == APP_BLE_LP_ADV))
  {
    /* Connection in ADVERTISE mode have to stop the current advertising */
    ret = aci_gap_set_non_discoverable();
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("==>> aci_gap_set_non_discoverable - Stop Advertising Failed , result: %d \n", ret);
    }
    else
    {
      APP_DBG_MSG("==>> aci_gap_set_non_discoverable - Successfully Stopped Advertising \n");
    }
  }

  /* Call advertising callback */
  if (Adv_Callback) Adv_Callback();

  /* Update advertising callback */
  Adv_Callback = Next_Adv_Callback;
  Next_Adv_Callback = 0;

  BleApplicationContext.Device_Connection_Status = NewStatus;

  /**
   * Prepare white list as described in PM0271 5.3.1
   */
  ble_count_bonded_devices();

  if (request_pairing)
  {
    /* Start Fast or Low Power Advertising */
    ret = aci_gap_set_limited_discoverable(ADV_IND,
                                           Min_Inter,
                                           Max_Inter,
                                           CFG_BLE_ADDRESS_TYPE,
                                           ADV_FILTER,
                                           0,
                                           0,
                                           0,
                                           0,
                                           0,
                                           0);
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("==>> aci_gap_set_limited_discoverable - fail, result: 0x%x \n", ret);
    }
    else
    {
      APP_DBG_MSG("==>> aci_gap_set_limited_discoverable - Success\n");
    }

    /* Update advertising data */
    APP_BLE_UpdateAdvertisingData(NewStatus);
  }
  else
  {
    /* Start Fast or Low Power Advertising */
    ret = aci_gap_set_undirected_connectable(Min_Inter,
                                             Max_Inter,
                                             CFG_BLE_ADDRESS_TYPE,
                                             HCI_ADV_FILTER_WHITELIST_SCAN_CONNECT);
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("==>> aci_gap_set_undirected_connectable - fail, result: 0x%x \n", ret);
    }
    else
    {
      APP_DBG_MSG("==>> aci_gap_set_undirected_connectable - Success\n");
    }

    /* Update advertising data */
    APP_BLE_UpdateAdvertisingData(NewStatus);
  }

  if (NewStatus == APP_BLE_FAST_ADV)
  {
    HW_TS_Start(BleApplicationContext.Advertising_mgr_timer_Id, FAST_ADV_TIMEOUT);
  }

  return;
}

static void Adv_Update_Req(void)
{
  /**
   * The code shall be executed in the background as an aci command may be sent
   * The background is the only place where the application can make sure a new aci command
   * is not sent if there is a pending one
   */
  UTIL_SEQ_SetTask(1 << CFG_TASK_ADV_UPDATE_ID, CFG_SCH_PRIO_1);
}

static void Adv_Update(void)
{
  FS_Adv_Request(APP_BLE_LP_ADV);
}

void APP_BLE_RequestPairing(void (*Callback)(void))
{
  /* Prepare for pairing request */
  Next_Adv_Callback = Callback;
  request_pairing = 1;

  /* Request fast advertising */
  FS_Adv_Request(APP_BLE_FAST_ADV);
}

void APP_BLE_CancelPairing(void)
{
  /* Prepare for pairing request */
  Next_Adv_Callback = 0;
  request_pairing = 0;

  if (BleApplicationContext.Device_Connection_Status == APP_BLE_FAST_ADV)
  {
    /* Request low power advertising */
    FS_Adv_Request(APP_BLE_LP_ADV);
  }
}

void APP_BLE_UpdateDeviceName(void)
{
  /**
   * For details of Bluetooth device name characteristic, see:
   * https://www.bluetooth.com/specifications/css-11/
   */

  const char *name = FS_State_Get()->device_name;
  const size_t name_len = strlen(name);
  uint8_t char_length;
  tBleStatus ret;

  if (BleApplicationContext.BleApplicationContext_legacy.gapServiceHandle)
  {
    /* Get device name length */
    char_length = MIN(name_len, 30);

    /* Update device name characteristic */
    ret = aci_gatt_update_char_value(
        BleApplicationContext.BleApplicationContext_legacy.gapServiceHandle,
        BleApplicationContext.BleApplicationContext_legacy.devNameCharHandle,
        0,
        char_length, (uint8_t *) name);
    if (ret != BLE_STATUS_SUCCESS)
    {
      BLE_DBG_SVCCTL_MSG("  Fail   : aci_gatt_update_char_value - Device Name\n");
    }
    else
    {
      BLE_DBG_SVCCTL_MSG("  Success: aci_gatt_update_char_value - Device Name\n");
    }
  }

  /* Update advertising data */
  APP_BLE_UpdateAdvertisingData(BleApplicationContext.Device_Connection_Status);
}

void APP_BLE_Reset(void)
{
  tBleStatus ret;

  /* Clear list of bonded devices */
  ret = aci_gap_clear_security_db();
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gap_clear_security_db command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gap_clear_security_db command\n");
  }

  /* Re-initialize advertising */
  FS_Adv_Request(BleApplicationContext.Device_Connection_Status);
}
static void APP_BLE_UpdateAdvertisingData(APP_BLE_ConnStatus_t NewStatus)
{
  /**
   * For details of Bluetooth advertised local name, see:
   * https://www.bluetooth.com/specifications/css-11/
   */

  const char *name = FS_State_Get()->device_name;
  const size_t name_len = strlen(name);
  uint8_t mfg_data[] = { 4, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 0xDB, 0x09, 0x00 };
  uint8_t adv_data[31];
  uint8_t k = 0;
  uint8_t ad_length;
  uint8_t ad_type;
  tBleStatus ret;

  if ((NewStatus == APP_BLE_FAST_ADV) || (NewStatus == APP_BLE_LP_ADV))
  {
    /* Copy manufacturer specific data */
    memcpy(&(adv_data[k]), mfg_data, sizeof(mfg_data));
    k += sizeof(mfg_data);

    /* Get short local name if needed */
    if (name_len > sizeof(adv_data) - k - 2)
    {
      ad_length = sizeof(adv_data) - k - 2;
      ad_type = AD_TYPE_SHORTENED_LOCAL_NAME;
    }
    else
    {
      ad_length = name_len;
      ad_type = AD_TYPE_COMPLETE_LOCAL_NAME;
    }

    /* Copy local name */
    adv_data[k++] = 1 + ad_length;
    adv_data[k++] = ad_type;
    memcpy(&(adv_data[k]), name, ad_length);
    k += ad_length;

    /* Update advertising data */
    ret = aci_gap_update_adv_data(k, adv_data);
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("==>> aci_gap_update_adv_data - fail, result: 0x%x \n", ret);
    }
    else
    {
      APP_DBG_MSG("==>> aci_gap_update_adv_data - Success\n");
    }
  }
}

static int8_t ble_count_bonded_devices(void)
{
  uint8_t total = 0;
  Bonded_Device_Entry_t devices[16];
  Identity_Entry_t Private_devices[16];

  tBleStatus ret = aci_gap_configure_whitelist();
  APP_DBG_MSG("\taci_gap_configure_whitelist = 0x%02X\r\n", ret);

  ret = aci_gap_get_bonded_devices(&total, devices);
  if(BLE_STATUS_SUCCESS == ret)
  {
    if(total == 0)
    {
      APP_DBG_MSG("No previous bonded devices\r\n");
    }
    else
    {
      for(uint8_t k = 0; k < total; k++)
      {
        APP_DBG_MSG("Bonded device %d / %d - %s BLE address %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                    k, total,
                    devices[k].Address_Type == 0 ? "Public" : "Random",
                    devices[k].Address[5], devices[k].Address[4], devices[k].Address[3],
                    devices[k].Address[2], devices[k].Address[1], devices[k].Address[0]);
        Private_devices[k].Peer_Identity_Address_Type = devices[k].Address_Type;
        Private_devices[k].Peer_Identity_Address[5] = devices[k].Address[5];
        Private_devices[k].Peer_Identity_Address[4] = devices[k].Address[4];
        Private_devices[k].Peer_Identity_Address[3] = devices[k].Address[3];
        Private_devices[k].Peer_Identity_Address[2] = devices[k].Address[2];
        Private_devices[k].Peer_Identity_Address[1] = devices[k].Address[1];
        Private_devices[k].Peer_Identity_Address[0] = devices[k].Address[0];
      }

      ret = aci_gap_add_devices_to_resolving_list(total, Private_devices, 0);
      if(BLE_STATUS_SUCCESS == ret)
      {
        APP_DBG_MSG("aci_gap_add_devices_to_resolving_list success \r\n");
      }
      else
      {
        APP_DBG_MSG("aci_gap_add_devices_to_resolving_list fail %x \r\n", ret);
      }
    }
  }
  else
  {
    APP_DBG_MSG("ACI_GAP_GET_BONDED_DEVICES error %x\r\n", ret);
    total = -1;
  }
  return total;
}
/* USER CODE END FD_SPECIFIC_FUNCTIONS */
/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/
void hci_notify_asynch_evt(void* p_Data)
{
  UTIL_SEQ_SetTask(1 << CFG_TASK_HCI_ASYNCH_EVT_ID, CFG_SCH_PRIO_0);

  return;
}

void hci_cmd_resp_release(uint32_t Flag)
{
  UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);

  return;
}

void hci_cmd_resp_wait(uint32_t Timeout)
{
  UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);

  return;
}

static void BLE_UserEvtRx(void *p_Payload)
{
  SVCCTL_UserEvtFlowStatus_t svctl_return_status;
  tHCI_UserEvtRxParam *p_param;

  p_param = (tHCI_UserEvtRxParam *)p_Payload;

  svctl_return_status = SVCCTL_UserEvtRx((void *)&(p_param->pckt->evtserial));
  if (svctl_return_status != SVCCTL_UserEvtFlowDisable)
  {
    p_param->status = HCI_TL_UserEventFlow_Enable;
  }
  else
  {
    p_param->status = HCI_TL_UserEventFlow_Disable;
  }

  return;
}

static void BLE_StatusNot(HCI_TL_CmdStatus_t Status)
{
  uint32_t task_id_list;
  switch (Status)
  {
    case HCI_TL_CmdBusy:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_PauseTask(task_id_list);
      /* USER CODE BEGIN HCI_TL_CmdBusy */

      /* USER CODE END HCI_TL_CmdBusy */
      break;

    case HCI_TL_CmdAvailable:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_ResumeTask(task_id_list);
      /* USER CODE BEGIN HCI_TL_CmdAvailable */

      /* USER CODE END HCI_TL_CmdAvailable */
      break;

    default:
      /* USER CODE BEGIN Status */

      /* USER CODE END Status */
      break;
  }

  return;
}

void SVCCTL_ResumeUserEventFlow(void)
{
  hci_resume_flow();

  return;
}

/* USER CODE BEGIN FD_WRAP_FUNCTIONS */

/* USER CODE END FD_WRAP_FUNCTIONS */
