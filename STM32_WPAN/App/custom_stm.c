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
  uint16_t  CustomFile_TransferHdle;                    /**< File_Transfer handle */
  uint16_t  CustomFt_Packet_OutHdle;                  /**< FT_Packet_Out handle */
  uint16_t  CustomFt_Packet_InHdle;                  /**< FT_Packet_In handle */
  uint16_t  CustomSensor_DataHdle;                    /**< Sensor_Data handle */
  uint16_t  CustomSd_Gnss_MeasurementHdle;                  /**< SD_GNSS_Measurement handle */
  uint16_t  CustomSd_Control_PointHdle;                  /**< SD_Control_Point handle */
  uint16_t  CustomSd_Baro_MeasurementHdle;                  /**< SD_BARO_Measurement handle */
  uint16_t  CustomStarter_PistolHdle;                    /**< Starter_Pistol handle */
  uint16_t  CustomSp_Control_PointHdle;                  /**< SP_Control_Point handle */
  uint16_t  CustomSp_ResultHdle;                  /**< SP_Result handle */
  uint16_t  CustomDevice_StateHdle;                    /**< Device_State handle */
  uint16_t  CustomDs_ModeHdle;                  /**< DS_Mode handle */
  uint16_t  CustomDs_Control_PointHdle;                  /**< DS_Control_Point handle */
  uint16_t  CustomBatteryHdle;                    /**< Battery handle */
  uint16_t  CustomBattery_LevelHdle;                  /**< Battery_Level handle */
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
uint8_t SizeFt_Packet_Out = 244;
uint8_t SizeFt_Packet_In = 244;
uint8_t SizeSd_Gnss_Measurement = 44;
uint8_t SizeSd_Control_Point = 20;
uint8_t SizeSd_Baro_Measurement = 12;
uint8_t SizeSp_Control_Point = 20;
uint8_t SizeSp_Result = 9;
uint8_t SizeDs_Mode = 1;
uint8_t SizeDs_Control_Point = 20;
uint8_t SizeBattery_Level = 1;

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
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

#define COPY_FILE_TRANSFER_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_FT_PACKET_OUT_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_FT_PACKET_IN_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_SENSOR_DATA_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_SD_GNSS_MEASUREMENT_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_SD_CONTROL_POINT_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x06,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_STARTER_PISTOL_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_SP_CONTROL_POINT_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x03,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_SP_RESULT_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x04,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_DEVICE_STATE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x03,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_DS_MODE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x05,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_DS_CONTROL_POINT_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x07,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_SD_BARO_MEASUREMENT_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x08,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)

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
          if (attribute_modified->Attr_Handle == (CustomContext.CustomFt_Packet_OutHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
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
                Notification.Custom_Evt_Opcode = CUSTOM_STM_FT_PACKET_OUT_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_1_Char_1_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_1_Char_1_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_FT_PACKET_OUT_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_1_Char_1_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_default */

                /* USER CODE END CUSTOM_STM_Service_1_Char_1_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomFt_Packet_OutHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomSd_Gnss_MeasurementHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
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
                Notification.Custom_Evt_Opcode = CUSTOM_STM_SD_GNSS_MEASUREMENT_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_SD_GNSS_MEASUREMENT_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_default */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomSd_Gnss_MeasurementHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomSd_Control_PointHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2 */

            /* USER CODE END CUSTOM_STM_Service_2_Char_2 */

            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_2_Char_2_attribute_modified */

              /* Disabled Indication management */
              case (!(COMSVC_Indication)):
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_2_attribute_modified */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_SD_CONTROL_POINT_INDICATE_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_2_Disabled_END */
                break;

                /* Enabled Indication management */
              case COMSVC_Indication:
              /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_COMSVC_Indication_BEGIN */

              /* USER CODE END CUSTOM_STM_Service_2_Char_2_COMSVC_Indication_BEGIN */
              Notification.Custom_Evt_Opcode = CUSTOM_STM_SD_CONTROL_POINT_INDICATE_ENABLED_EVT;
              Custom_STM_App_Notification(&Notification);
              /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_COMSVC_Indication_END */

              /* USER CODE END CUSTOM_STM_Service_2_Char_2_COMSVC_Indication_END */
              break;

              default:
              /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_default */

              /* USER CODE END CUSTOM_STM_Service_2_Char_2_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomSD_Control_PointHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomSd_Baro_MeasurementHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_3 */

            /* USER CODE END CUSTOM_STM_Service_2_Char_3 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_3_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_2_Char_3_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_3_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_3_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_SD_BARO_MEASUREMENT_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_3_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_3_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_3_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_3_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_SD_BARO_MEASUREMENT_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_3_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_3_COMSVC_Notification_END */
                break;

              default:
              /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_3_default */

              /* USER CODE END CUSTOM_STM_Service_2_Char_3_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomSd_Baro_MeasurementHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomSp_Control_PointHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
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
                Notification.Custom_Evt_Opcode = CUSTOM_STM_SP_CONTROL_POINT_INDICATE_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_Disabled_END */
                break;

                /* Enabled Indication management */
              case COMSVC_Indication:
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_COMSVC_Indication_BEGIN */

              /* USER CODE END CUSTOM_STM_Service_3_Char_1_COMSVC_Indication_BEGIN */
              Notification.Custom_Evt_Opcode = CUSTOM_STM_SP_CONTROL_POINT_INDICATE_ENABLED_EVT;
              Custom_STM_App_Notification(&Notification);
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_COMSVC_Indication_END */

              /* USER CODE END CUSTOM_STM_Service_3_Char_1_COMSVC_Indication_END */
              break;

              default:
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_default */

              /* USER CODE END CUSTOM_STM_Service_3_Char_1_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomSP_Control_PointHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomSp_ResultHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
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
                Notification.Custom_Evt_Opcode = CUSTOM_STM_SP_RESULT_INDICATE_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_2_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_3_Char_2_Disabled_END */
                break;

                /* Enabled Indication management */
              case COMSVC_Indication:
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_2_COMSVC_Indication_BEGIN */

              /* USER CODE END CUSTOM_STM_Service_3_Char_2_COMSVC_Indication_BEGIN */
              Notification.Custom_Evt_Opcode = CUSTOM_STM_SP_RESULT_INDICATE_ENABLED_EVT;
              Custom_STM_App_Notification(&Notification);
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_2_COMSVC_Indication_END */

              /* USER CODE END CUSTOM_STM_Service_3_Char_2_COMSVC_Indication_END */
              break;

              default:
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_2_default */

              /* USER CODE END CUSTOM_STM_Service_3_Char_2_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomSP_ResultHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomDs_ModeHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1 */

            /* USER CODE END CUSTOM_STM_Service_4_Char_1 */

            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_4_Char_1_attribute_modified */

              /* Disabled Indication management */
              case (!(COMSVC_Indication)):
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_4_Char_1_attribute_modified */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_DS_MODE_INDICATE_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_4_Char_1_Disabled_END */
                break;

                /* Enabled Indication management */
              case COMSVC_Indication:
              /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_COMSVC_Indication_BEGIN */

              /* USER CODE END CUSTOM_STM_Service_4_Char_1_COMSVC_Indication_BEGIN */
              Notification.Custom_Evt_Opcode = CUSTOM_STM_DS_MODE_INDICATE_ENABLED_EVT;
              Custom_STM_App_Notification(&Notification);
              /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_COMSVC_Indication_END */

              /* USER CODE END CUSTOM_STM_Service_4_Char_1_COMSVC_Indication_END */
              break;

              default:
              /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_default */

              /* USER CODE END CUSTOM_STM_Service_4_Char_1_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomDS_ModeHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomDs_Control_PointHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_2 */

            /* USER CODE END CUSTOM_STM_Service_4_Char_2 */

            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_2_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_4_Char_2_attribute_modified */

              /* Disabled Indication management */
              case (!(COMSVC_Indication)):
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_2_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_4_Char_2_attribute_modified */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_DS_CONTROL_POINT_INDICATE_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_2_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_4_Char_2_Disabled_END */
                break;

                /* Enabled Indication management */
              case COMSVC_Indication:
              /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_2_COMSVC_Indication_BEGIN */

              /* USER CODE END CUSTOM_STM_Service_4_Char_2_COMSVC_Indication_BEGIN */
              Notification.Custom_Evt_Opcode = CUSTOM_STM_DS_CONTROL_POINT_INDICATE_ENABLED_EVT;
              Custom_STM_App_Notification(&Notification);
              /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_2_COMSVC_Indication_END */

              /* USER CODE END CUSTOM_STM_Service_4_Char_2_COMSVC_Indication_END */
              break;

              default:
              /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_2_default */

              /* USER CODE END CUSTOM_STM_Service_4_Char_2_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomDS_Control_PointHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomBattery_LevelHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_1 */

            /* USER CODE END CUSTOM_STM_Service_5_Char_1 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_1_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_5_Char_1_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_1_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_5_Char_1_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_BATTERY_LEVEL_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_1_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_5_Char_1_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_1_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_5_Char_1_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_BATTERY_LEVEL_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_1_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_5_Char_1_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_1_default */

                /* USER CODE END CUSTOM_STM_Service_5_Char_1_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomBattery_LevelHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomFt_Packet_InHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
            Notification.Custom_Evt_Opcode = CUSTOM_STM_FT_PACKET_IN_WRITE_NO_RESP_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
            /* USER CODE END CUSTOM_STM_Service_1_Char_2_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomFt_Packet_InHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomSd_Control_PointHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
            Notification.Custom_Evt_Opcode = CUSTOM_STM_SD_CONTROL_POINT_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
            /* USER CODE END CUSTOM_STM_Service_2_Char_2_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomSd_Control_PointHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomSp_Control_PointHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
            Notification.Custom_Evt_Opcode = CUSTOM_STM_SP_CONTROL_POINT_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
            /* USER CODE END CUSTOM_STM_Service_3_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomSp_Control_PointHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomDs_Control_PointHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_2_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
            Notification.Custom_Evt_Opcode = CUSTOM_STM_DS_CONTROL_POINT_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
            /* USER CODE END CUSTOM_STM_Service_4_Char_2_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomDs_Control_PointHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */
          break;

        case ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE :
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */
          read_req = (aci_gatt_read_permit_req_event_rp0*)blecore_evt->data;
          if (read_req->Attribute_Handle == (CustomContext.CustomSd_Gnss_MeasurementHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /*USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_1 */
            Notification.Custom_Evt_Opcode = CUSTOM_STM_SD_GNSS_MEASUREMENT_READ_EVT;
            Custom_STM_App_Notification(&Notification);
            /*USER CODE END CUSTOM_STM_Service_2_Char_1_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_1*/
            aci_gatt_allow_read(read_req->Connection_Handle);
            /*USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_2 */

            /*USER CODE END CUSTOM_STM_Service_2_Char_1_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_2*/
          } /* if (read_req->Attribute_Handle == (CustomContext.CustomSd_Gnss_MeasurementHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (read_req->Attribute_Handle == (CustomContext.CustomSp_ResultHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /*USER CODE BEGIN CUSTOM_STM_Service_3_Char_2_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_1 */
            Notification.Custom_Evt_Opcode = CUSTOM_STM_SP_RESULT_READ_EVT;
            Custom_STM_App_Notification(&Notification);
            /*USER CODE END CUSTOM_STM_Service_3_Char_2_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_1*/
            aci_gatt_allow_read(read_req->Connection_Handle);
            /*USER CODE BEGIN CUSTOM_STM_Service_3_Char_2_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_2 */

            /*USER CODE END CUSTOM_STM_Service_3_Char_2_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_2*/
          } /* if (read_req->Attribute_Handle == (CustomContext.CustomSp_ResultHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (read_req->Attribute_Handle == (CustomContext.CustomDs_ModeHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /*USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_1 */
            Notification.Custom_Evt_Opcode = CUSTOM_STM_DS_MODE_READ_EVT;
            Custom_STM_App_Notification(&Notification);
            /*USER CODE END CUSTOM_STM_Service_4_Char_1_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_1*/
            aci_gatt_allow_read(read_req->Connection_Handle);
            /*USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_2 */

            /*USER CODE END CUSTOM_STM_Service_4_Char_1_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_2*/
          } /* if (read_req->Attribute_Handle == (CustomContext.CustomDs_ModeHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (read_req->Attribute_Handle == (CustomContext.CustomBattery_LevelHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /*USER CODE BEGIN CUSTOM_STM_Service_5_Char_1_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_1 */
            Notification.Custom_Evt_Opcode = CUSTOM_STM_BATTERY_LEVEL_READ_EVT;
            Notification.ConnectionHandle = read_req->Connection_Handle;
            Custom_STM_App_Notification(&Notification);
            /*USER CODE END CUSTOM_STM_Service_5_Char_1_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_1*/
            aci_gatt_allow_read(read_req->Connection_Handle);
            /*USER CODE BEGIN CUSTOM_STM_Service_5_Char_1_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_2 */

            /*USER CODE END CUSTOM_STM_Service_5_Char_1_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_2*/
          } /* if (read_req->Attribute_Handle == (CustomContext.CustomBattery_LevelHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
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
   *          File_Transfer
   *
   * Max_Attribute_Records = 1 + 2*2 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for File_Transfer +
   *                                2 for FT_Packet_Out +
   *                                2 for FT_Packet_In +
   *                                1 for FT_Packet_Out configuration descriptor +
   *                              = 6
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 6;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  COPY_FILE_TRANSFER_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomFile_TransferHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: File_Transfer, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: File_Transfer \n\r");
  }

  /**
   *  FT_Packet_Out
   */
  COPY_FT_PACKET_OUT_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomFile_TransferHdle,
                          UUID_TYPE_128, &uuid,
                          SizeFt_Packet_Out,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_ENCRY_WRITE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomFt_Packet_OutHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : FT_PACKET_OUT, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : FT_PACKET_OUT \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char1/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char1 */
  /**
   *  FT_Packet_In
   */
  COPY_FT_PACKET_IN_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomFile_TransferHdle,
                          UUID_TYPE_128, &uuid,
                          SizeFt_Packet_In,
                          CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RESP,
                          ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_ENCRY_WRITE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomFt_Packet_InHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : FT_PACKET_IN, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : FT_PACKET_IN \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char2/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char2 */

  /**
   *          Sensor_Data
   *
   * Max_Attribute_Records = 1 + 2*2 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for Sensor_Data +
   *                                2 for SD_GNSS_Measurement +
   *                                2 for SD_Control_Point +
   *                                2 for SD_BARO_Measurement +
   *                                1 for SD_GNSS_Measurement configuration descriptor +
   *                                1 for SD_Control_Point configuration descriptor +
   *                                1 for SD_BARO_Measurement configuration descriptor +
   *                              = 10
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 10;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  COPY_SENSOR_DATA_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomSensor_DataHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: Sensor_Data, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: Sensor_Data \n\r");
  }

  /**
   *  SD_GNSS_Measurement
   */
  COPY_SD_GNSS_MEASUREMENT_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomSensor_DataHdle,
                          UUID_TYPE_128, &uuid,
                          SizeSd_Gnss_Measurement,
                          CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_ENCRY_WRITE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomSd_Gnss_MeasurementHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : SD_GNSS_MEASUREMENT, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : SD_GNSS_MEASUREMENT \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service2_Char1/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service2_Char1 */
  /**
   *  SD_Control_Point
   */
  COPY_SD_CONTROL_POINT_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomSensor_DataHdle,
                          UUID_TYPE_128, &uuid,
                          SizeSd_Control_Point,
                          CHAR_PROP_WRITE | CHAR_PROP_INDICATE,
                          ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_ENCRY_WRITE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomSd_Control_PointHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : SD_CONTROL_POINT, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : SD_CONTROL_POINT \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service2_Char2/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service2_Char2 */

  /**
   *  SD_BARO_Measurement
   */
  COPY_SD_BARO_MEASUREMENT_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomSensor_DataHdle,
                          UUID_TYPE_128, &uuid,
                          SizeSd_Baro_Measurement,
                          CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_ENCRY_WRITE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomSd_Baro_MeasurementHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : SD_BARO_MEASUREMENT, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : SD_BARO_MEASUREMENT \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service2_Char3/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service2_Char3 */

  /**
   *          Starter_Pistol
   *
   * Max_Attribute_Records = 1 + 2*2 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for Starter_Pistol +
   *                                2 for SP_Control_Point +
   *                                2 for SP_Result +
   *                                1 for SP_Control_Point configuration descriptor +
   *                                1 for SP_Result configuration descriptor +
   *                              = 7
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 7;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  COPY_STARTER_PISTOL_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomStarter_PistolHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: Starter_Pistol, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: Starter_Pistol \n\r");
  }

  /**
   *  SP_Control_Point
   */
  COPY_SP_CONTROL_POINT_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomStarter_PistolHdle,
                          UUID_TYPE_128, &uuid,
                          SizeSp_Control_Point,
                          CHAR_PROP_WRITE | CHAR_PROP_INDICATE,
                          ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_ENCRY_WRITE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomSp_Control_PointHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : SP_CONTROL_POINT, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : SP_CONTROL_POINT \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service3_Char1/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service3_Char1 */
  /**
   *  SP_Result
   */
  COPY_SP_RESULT_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomStarter_PistolHdle,
                          UUID_TYPE_128, &uuid,
                          SizeSp_Result,
                          CHAR_PROP_READ | CHAR_PROP_INDICATE,
                          ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_ENCRY_WRITE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomSp_ResultHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : SP_RESULT, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : SP_RESULT \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service3_Char2/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service3_Char2 */

  /**
   *          Device_State
   *
   * Max_Attribute_Records = 1 + 2*2 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for Device_State +
   *                                2 for DS_Mode +
   *                                2 for DS_Control_Point +
   *                                1 for DS_Mode configuration descriptor +
   *                                1 for DS_Control_Point configuration descriptor +
   *                              = 7
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 7;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  COPY_DEVICE_STATE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomDevice_StateHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: Device_State, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: Device_State \n\r");
  }

  /**
   *  DS_Mode
   */
  COPY_DS_MODE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomDevice_StateHdle,
                          UUID_TYPE_128, &uuid,
                          SizeDs_Mode,
                          CHAR_PROP_READ | CHAR_PROP_INDICATE,
                          ATTR_PERMISSION_ENCRY_READ,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomDs_ModeHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : DS_MODE, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : DS_MODE \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service4_Char1/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service4_Char1 */
  /**
   *  DS_Control_Point
   */
  COPY_DS_CONTROL_POINT_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomDevice_StateHdle,
                          UUID_TYPE_128, &uuid,
                          SizeDs_Control_Point,
                          CHAR_PROP_WRITE | CHAR_PROP_INDICATE,
                          ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_ENCRY_WRITE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomDs_Control_PointHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : DS_CONTROL_POINT, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : DS_CONTROL_POINT \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service4_Char2/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service4_Char2 */

  /**
   *          Battery
   *
   * Max_Attribute_Records = 1 + 2*1 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for Battery +
   *                                2 for Battery_Level +
   *                                1 for Battery_Level configuration descriptor +
   *                              = 4
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 4;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  uuid.Char_UUID_16 = 0x180f;
  ret = aci_gatt_add_service(UUID_TYPE_16,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomBatteryHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: Battery, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: Battery \n\r");
  }

  /**
   *  Battery_Level
   */
  uuid.Char_UUID_16 = 0x2a19;
  ret = aci_gatt_add_char(CustomContext.CustomBatteryHdle,
                          UUID_TYPE_16, &uuid,
                          SizeBattery_Level,
                          CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_ENCRY_READ,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomBattery_LevelHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : BATTERY_LEVEL, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : BATTERY_LEVEL \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service5_Char1/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service5_Char1 */

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

    case CUSTOM_STM_FT_PACKET_OUT:
      ret = aci_gatt_update_char_value(CustomContext.CustomFile_TransferHdle,
                                       CustomContext.CustomFt_Packet_OutHdle,
                                       0, /* charValOffset */
                                       SizeFt_Packet_Out, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value FT_PACKET_OUT command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value FT_PACKET_OUT command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_1*/
      break;

    case CUSTOM_STM_FT_PACKET_IN:
      ret = aci_gatt_update_char_value(CustomContext.CustomFile_TransferHdle,
                                       CustomContext.CustomFt_Packet_InHdle,
                                       0, /* charValOffset */
                                       SizeFt_Packet_In, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value FT_PACKET_IN command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value FT_PACKET_IN command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_2*/
      break;

    case CUSTOM_STM_SD_GNSS_MEASUREMENT:
      ret = aci_gatt_update_char_value(CustomContext.CustomSensor_DataHdle,
                                       CustomContext.CustomSd_Gnss_MeasurementHdle,
                                       0, /* charValOffset */
                                       SizeSd_Gnss_Measurement, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value SD_GNSS_MEASUREMENT command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value SD_GNSS_MEASUREMENT command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_2_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_2_Char_1*/
      break;

    case CUSTOM_STM_SD_CONTROL_POINT:
      ret = aci_gatt_update_char_value(CustomContext.CustomSensor_DataHdle,
                                       CustomContext.CustomSd_Control_PointHdle,
                                       0, /* charValOffset */
                                       SizeSd_Control_Point, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value SD_CONTROL_POINT command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value SD_CONTROL_POINT command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_2_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_2_Char_2*/
      break;

    case CUSTOM_STM_SD_BARO_MEASUREMENT:
      ret = aci_gatt_update_char_value(CustomContext.CustomSensor_DataHdle,
                                       CustomContext.CustomSd_Baro_MeasurementHdle,
                                       0, /* charValOffset */
                                       SizeSd_Baro_Measurement, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value SD_BARO_MEASUREMENT command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value SD_BARO_MEASUREMENT command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_2_Char_3*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_2_Char_3*/
      break;

    case CUSTOM_STM_SP_CONTROL_POINT:
      ret = aci_gatt_update_char_value(CustomContext.CustomStarter_PistolHdle,
                                       CustomContext.CustomSp_Control_PointHdle,
                                       0, /* charValOffset */
                                       SizeSp_Control_Point, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value SP_CONTROL_POINT command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value SP_CONTROL_POINT command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_3_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_3_Char_1*/
      break;

    case CUSTOM_STM_SP_RESULT:
      ret = aci_gatt_update_char_value(CustomContext.CustomStarter_PistolHdle,
                                       CustomContext.CustomSp_ResultHdle,
                                       0, /* charValOffset */
                                       SizeSp_Result, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value SP_RESULT command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value SP_RESULT command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_3_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_3_Char_2*/
      break;

    case CUSTOM_STM_DS_MODE:
      ret = aci_gatt_update_char_value(CustomContext.CustomDevice_StateHdle,
                                       CustomContext.CustomDs_ModeHdle,
                                       0, /* charValOffset */
                                       SizeDs_Mode, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value DS_MODE command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value DS_MODE command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_4_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_4_Char_1*/
      break;

    case CUSTOM_STM_DS_CONTROL_POINT:
      ret = aci_gatt_update_char_value(CustomContext.CustomDevice_StateHdle,
                                       CustomContext.CustomDs_Control_PointHdle,
                                       0, /* charValOffset */
                                       SizeDs_Control_Point, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value DS_CONTROL_POINT command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value DS_CONTROL_POINT command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_4_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_4_Char_2*/
      break;

    case CUSTOM_STM_BATTERY_LEVEL:
      ret = aci_gatt_update_char_value(CustomContext.CustomBatteryHdle,
                                       CustomContext.CustomBattery_LevelHdle,
                                       0, /* charValOffset */
                                       SizeBattery_Level, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value BATTERY_LEVEL command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value BATTERY_LEVEL command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_5_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_5_Char_1*/
      break;

    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_2 */

  /* USER CODE END Custom_STM_App_Update_Char_2 */

  return ret;
}
