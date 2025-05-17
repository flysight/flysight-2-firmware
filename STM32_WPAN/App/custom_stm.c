/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.c
  * @author  MCD Application Team
  * @brief   Custom Example Service.
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
#include "common_blesvc.h"
#include "custom_stm.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint16_t  CustomCrsHdle;                    /**< CRS handle */
  uint16_t  CustomCrs_TxHdle;                  /**< CRS_TX handle */
  uint16_t  CustomCrs_RxHdle;                  /**< CRS_RX handle */
  uint16_t  CustomGnssHdle;                    /**< GNSS handle */
  uint16_t  CustomGnss_PvHdle;                  /**< GNSS_PV handle */
  uint16_t  CustomGnss_ControlHdle;                  /**< GNSS_Control handle */
  uint16_t  CustomStartHdle;                    /**< Start handle */
  uint16_t  CustomStart_ControlHdle;                  /**< Start_Control handle */
  uint16_t  CustomStart_ResultHdle;                  /**< Start_Result handle */
/* USER CODE BEGIN Context */
  /* Place holder for Characteristic Descriptors Handle*/

/* USER CODE END Context */
}CustomContext_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define UUID_128_SUPPORTED  1

#if (UUID_128_SUPPORTED == 1)
#define BM_UUID_LENGTH  UUID_TYPE_128
#else
#define BM_UUID_LENGTH  UUID_TYPE_16
#endif

#define BM_REQ_CHAR_SIZE    (3)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET         2
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET              1
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
uint8_t SizeCrs_Tx = 244;
uint8_t SizeCrs_Rx = 244;
uint8_t SizeGnss_Pv = 44;
uint8_t SizeGnss_Control = 2;
uint8_t SizeStart_Control = 1;
uint8_t SizeStart_Result = 9;

/**
 * START of Section BLE_DRIVER_CONTEXT
 */
static CustomContext_t CustomContext;

/**
 * END of Section BLE_DRIVER_CONTEXT
 */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *pckt);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/* USER CODE BEGIN PFD */

/* USER CODE END PFD */

/* Private functions ----------------------------------------------------------*/

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

#define COPY_CRS_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_CRS_TX_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_CRS_RX_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_GNSS_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_GNSS_PV_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_GNSS_CONTROL_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x06,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_START_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_START_CONTROL_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x03,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_START_RESULT_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x04,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)

/* USER CODE BEGIN PF */

/* USER CODE END PF */

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blecore_aci *blecore_evt;
  aci_gatt_attribute_modified_event_rp0 *attribute_modified;
  aci_gatt_read_permit_req_event_rp0    *read_req;
  aci_gatt_notification_complete_event_rp0    *notification_complete;
  Custom_STM_App_Notification_evt_t     Notification;
  /* USER CODE BEGIN Custom_STM_Event_Handler_1 */

  /* USER CODE END Custom_STM_Event_Handler_1 */

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

  switch (event_pckt->evt)
  {
    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      blecore_evt = (evt_blecore_aci*)event_pckt->data;
      switch (blecore_evt->ecode)
      {
        case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */
          attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blecore_evt->data;
          if (attribute_modified->Attr_Handle == (CustomContext.CustomCrs_TxHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1 */

            /* USER CODE END CUSTOM_STM_Service_1_Char_1 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_1_Char_1_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_1_Char_1_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_CRS_TX_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_1_Char_1_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_1_Char_1_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_CRS_TX_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_1_Char_1_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_default */

                /* USER CODE END CUSTOM_STM_Service_1_Char_1_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomCrs_TxHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomGnss_PvHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1 */

            /* USER CODE END CUSTOM_STM_Service_2_Char_1 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_2_Char_1_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_GNSS_PV_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_GNSS_PV_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_default */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomGnss_PvHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomGnss_ControlHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2 */

            /* USER CODE END CUSTOM_STM_Service_2_Char_2 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_2_Char_2_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_2_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_GNSS_CONTROL_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_2_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_2_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_GNSS_CONTROL_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_2_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_default */

                /* USER CODE END CUSTOM_STM_Service_2_Char_2_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomGnss_ControlHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomStart_ControlHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1 */

            /* USER CODE END CUSTOM_STM_Service_3_Char_1 */

            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_3_Char_1_attribute_modified */

              /* Disabled Indication management */
              case (!(COMSVC_Indication)):
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_attribute_modified */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_START_CONTROL_INDICATE_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_Disabled_END */
                break;

                /* Enabled Indication management */
              case COMSVC_Indication:
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_COMSVC_Indication_BEGIN */

              /* USER CODE END CUSTOM_STM_Service_3_Char_1_COMSVC_Indication_BEGIN */
              Notification.Custom_Evt_Opcode = CUSTOM_STM_START_CONTROL_INDICATE_ENABLED_EVT;
              Custom_STM_App_Notification(&Notification);
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_COMSVC_Indication_END */

              /* USER CODE END CUSTOM_STM_Service_3_Char_1_COMSVC_Indication_END */
              break;

              default:
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_default */

              /* USER CODE END CUSTOM_STM_Service_3_Char_1_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomStart_ControlHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomStart_ResultHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_2 */

            /* USER CODE END CUSTOM_STM_Service_3_Char_2 */

            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_2_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_3_Char_2_attribute_modified */

              /* Disabled Indication management */
              case (!(COMSVC_Indication)):
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_2_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_3_Char_2_attribute_modified */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_START_RESULT_INDICATE_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_2_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_3_Char_2_Disabled_END */
                break;

                /* Enabled Indication management */
              case COMSVC_Indication:
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_2_COMSVC_Indication_BEGIN */

              /* USER CODE END CUSTOM_STM_Service_3_Char_2_COMSVC_Indication_BEGIN */
              Notification.Custom_Evt_Opcode = CUSTOM_STM_START_RESULT_INDICATE_ENABLED_EVT;
              Custom_STM_App_Notification(&Notification);
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_2_COMSVC_Indication_END */

              /* USER CODE END CUSTOM_STM_Service_3_Char_2_COMSVC_Indication_END */
              break;

              default:
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_2_default */

              /* USER CODE END CUSTOM_STM_Service_3_Char_2_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomStart_ResultHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomCrs_RxHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
            Notification.Custom_Evt_Opcode = CUSTOM_STM_CRS_RX_WRITE_NO_RESP_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
            /* USER CODE END CUSTOM_STM_Service_1_Char_2_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomCrs_RxHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomGnss_ControlHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
            Notification.Custom_Evt_Opcode = CUSTOM_STM_GNSS_CONTROL_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
            /* USER CODE END CUSTOM_STM_Service_2_Char_2_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomGnss_ControlHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomStart_ControlHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
            Notification.Custom_Evt_Opcode = CUSTOM_STM_START_CONTROL_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
            /* USER CODE END CUSTOM_STM_Service_3_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomStart_ControlHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */
          break;

        case ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE :
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */
          read_req = (aci_gatt_read_permit_req_event_rp0*)blecore_evt->data;
          if (read_req->Attribute_Handle == (CustomContext.CustomGnss_PvHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /*USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_1 */
            Notification.Custom_Evt_Opcode = CUSTOM_STM_GNSS_PV_READ_EVT;
            Custom_STM_App_Notification(&Notification);
            /*USER CODE END CUSTOM_STM_Service_2_Char_1_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_1*/
            aci_gatt_allow_read(read_req->Connection_Handle);
            /*USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_2 */

            /*USER CODE END CUSTOM_STM_Service_2_Char_1_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_2*/
          } /* if (read_req->Attribute_Handle == (CustomContext.CustomGnss_PvHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (read_req->Attribute_Handle == (CustomContext.CustomStart_ResultHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /*USER CODE BEGIN CUSTOM_STM_Service_3_Char_2_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_1 */
            Notification.Custom_Evt_Opcode = CUSTOM_STM_START_RESULT_READ_EVT;
            Custom_STM_App_Notification(&Notification);
            /*USER CODE END CUSTOM_STM_Service_3_Char_2_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_1*/
            aci_gatt_allow_read(read_req->Connection_Handle);
            /*USER CODE BEGIN CUSTOM_STM_Service_3_Char_2_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_2 */

            /*USER CODE END CUSTOM_STM_Service_3_Char_2_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_2*/
          } /* if (read_req->Attribute_Handle == (CustomContext.CustomStart_ResultHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_END */
          break;

        case ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */
          break;

		case ACI_GATT_NOTIFICATION_COMPLETE_VSEVT_CODE:
        {
          /* USER CODE BEGIN EVT_BLUE_GATT_NOTIFICATION_COMPLETE_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_NOTIFICATION_COMPLETE_BEGIN */
          notification_complete = (aci_gatt_notification_complete_event_rp0*)blecore_evt->data;
          Notification.Custom_Evt_Opcode = CUSTOM_STM_NOTIFICATION_COMPLETE_EVT;
          Notification.AttrHandle = notification_complete->Attr_Handle;
          Custom_STM_App_Notification(&Notification);
          /* USER CODE BEGIN EVT_BLUE_GATT_NOTIFICATION_COMPLETE_END */

          /* USER CODE END EVT_BLUE_GATT_NOTIFICATION_COMPLETE_END */
          break;
        }

        /* USER CODE BEGIN BLECORE_EVT */

        /* USER CODE END BLECORE_EVT */
        default:
          /* USER CODE BEGIN EVT_DEFAULT */

          /* USER CODE END EVT_DEFAULT */
          break;
      }
      /* USER CODE BEGIN EVT_VENDOR*/

      /* USER CODE END EVT_VENDOR*/
      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT_CASES*/

      /* USER CODE END EVENT_PCKT_CASES*/

    default:
      /* USER CODE BEGIN EVENT_PCKT*/

      /* USER CODE END EVENT_PCKT*/
      break;
  }

  /* USER CODE BEGIN Custom_STM_Event_Handler_2 */

  /* USER CODE END Custom_STM_Event_Handler_2 */

  return(return_value);
}/* end Custom_STM_Event_Handler */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void SVCCTL_InitCustomSvc(void)
{

  Char_UUID_t  uuid;
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint8_t max_attr_record;

  /* USER CODE BEGIN SVCCTL_InitCustomSvc_1 */

  /* USER CODE END SVCCTL_InitCustomSvc_1 */

  /**
   *  Register the event handler to the BLE controller
   */
  SVCCTL_RegisterSvcHandler(Custom_STM_Event_Handler);

  /**
   *          CRS
   *
   * Max_Attribute_Records = 1 + 2*2 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for CRS +
   *                                2 for CRS_TX +
   *                                2 for CRS_RX +
   *                                1 for CRS_TX configuration descriptor +
   *                              = 6
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 6;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  COPY_CRS_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomCrsHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: CRS, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: CRS \n\r");
  }

  /**
   *  CRS_TX
   */
  COPY_CRS_TX_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomCrsHdle,
                          UUID_TYPE_128, &uuid,
                          SizeCrs_Tx,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_ENCRY_WRITE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomCrs_TxHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : CRS_TX, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : CRS_TX \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char1/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char1 */
  /**
   *  CRS_RX
   */
  COPY_CRS_RX_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomCrsHdle,
                          UUID_TYPE_128, &uuid,
                          SizeCrs_Rx,
                          CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RESP,
                          ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_ENCRY_WRITE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomCrs_RxHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : CRS_RX, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : CRS_RX \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char2/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char2 */

  /**
   *          GNSS
   *
   * Max_Attribute_Records = 1 + 2*2 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for GNSS +
   *                                2 for GNSS_PV +
   *                                2 for GNSS_Control +
   *                                1 for GNSS_PV configuration descriptor +
   *                                1 for GNSS_Control configuration descriptor +
   *                              = 7
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 7;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  COPY_GNSS_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomGnssHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: GNSS, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: GNSS \n\r");
  }

  /**
   *  GNSS_PV
   */
  COPY_GNSS_PV_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomGnssHdle,
                          UUID_TYPE_128, &uuid,
                          SizeGnss_Pv,
                          CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_ENCRY_WRITE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomGnss_PvHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : GNSS_PV, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : GNSS_PV \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service2_Char1/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service2_Char1 */
  /**
   *  GNSS_Control
   */
  COPY_GNSS_CONTROL_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomGnssHdle,
                          UUID_TYPE_128, &uuid,
                          SizeGnss_Control,
                          CHAR_PROP_WRITE | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_ENCRY_WRITE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomGnss_ControlHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : GNSS_CONTROL, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : GNSS_CONTROL \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service2_Char2/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service2_Char2 */

  /**
   *          Start
   *
   * Max_Attribute_Records = 1 + 2*2 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for Start +
   *                                2 for Start_Control +
   *                                2 for Start_Result +
   *                                1 for Start_Control configuration descriptor +
   *                                1 for Start_Result configuration descriptor +
   *                              = 7
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 7;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  COPY_START_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomStartHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: Start, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: Start \n\r");
  }

  /**
   *  Start_Control
   */
  COPY_START_CONTROL_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomStartHdle,
                          UUID_TYPE_128, &uuid,
                          SizeStart_Control,
                          CHAR_PROP_WRITE | CHAR_PROP_INDICATE,
                          ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_ENCRY_WRITE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomStart_ControlHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : START_CONTROL, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : START_CONTROL \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service3_Char1/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service3_Char1 */
  /**
   *  Start_Result
   */
  COPY_START_RESULT_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomStartHdle,
                          UUID_TYPE_128, &uuid,
                          SizeStart_Result,
                          CHAR_PROP_READ | CHAR_PROP_INDICATE,
                          ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_ENCRY_WRITE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomStart_ResultHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : START_RESULT, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : START_RESULT \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service3_Char2/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service3_Char2 */

  /* USER CODE BEGIN SVCCTL_InitCustomSvc_2 */

  /* USER CODE END SVCCTL_InitCustomSvc_2 */

  return;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 *
 */
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_1 */

  /* USER CODE END Custom_STM_App_Update_Char_1 */

  switch (CharOpcode)
  {

    case CUSTOM_STM_CRS_TX:
      ret = aci_gatt_update_char_value(CustomContext.CustomCrsHdle,
                                       CustomContext.CustomCrs_TxHdle,
                                       0, /* charValOffset */
                                       SizeCrs_Tx, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value CRS_TX command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value CRS_TX command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_1*/
      break;

    case CUSTOM_STM_CRS_RX:
      ret = aci_gatt_update_char_value(CustomContext.CustomCrsHdle,
                                       CustomContext.CustomCrs_RxHdle,
                                       0, /* charValOffset */
                                       SizeCrs_Rx, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value CRS_RX command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value CRS_RX command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_2*/
      break;

    case CUSTOM_STM_GNSS_PV:
      ret = aci_gatt_update_char_value(CustomContext.CustomGnssHdle,
                                       CustomContext.CustomGnss_PvHdle,
                                       0, /* charValOffset */
                                       SizeGnss_Pv, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value GNSS_PV command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value GNSS_PV command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_2_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_2_Char_1*/
      break;

    case CUSTOM_STM_GNSS_CONTROL:
      ret = aci_gatt_update_char_value(CustomContext.CustomGnssHdle,
                                       CustomContext.CustomGnss_ControlHdle,
                                       0, /* charValOffset */
                                       SizeGnss_Control, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value GNSS_CONTROL command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value GNSS_CONTROL command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_2_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_2_Char_2*/
      break;

    case CUSTOM_STM_START_CONTROL:
      ret = aci_gatt_update_char_value(CustomContext.CustomStartHdle,
                                       CustomContext.CustomStart_ControlHdle,
                                       0, /* charValOffset */
                                       SizeStart_Control, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value START_CONTROL command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value START_CONTROL command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_3_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_3_Char_1*/
      break;

    case CUSTOM_STM_START_RESULT:
      ret = aci_gatt_update_char_value(CustomContext.CustomStartHdle,
                                       CustomContext.CustomStart_ResultHdle,
                                       0, /* charValOffset */
                                       SizeStart_Result, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value START_RESULT command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value START_RESULT command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_3_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_3_Char_2*/
      break;

    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_2 */

  /* USER CODE END Custom_STM_App_Update_Char_2 */

  return ret;
}
