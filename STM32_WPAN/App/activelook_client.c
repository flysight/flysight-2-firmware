/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2025 Bionic Avionics Inc.                                   **
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

#include "activelook_client.h"
#include "app_common.h"
#include "app_conf.h"
#include "dbg_trace.h"
#include "ble.h"
#include "tl.h"
#include "log.h"
#include "stm32_seq.h"
#include <string.h>

#ifndef UNPACK_2_BYTE_PARAMETER
#define UNPACK_2_BYTE_PARAMETER(ptr) \
    (uint16_t)( ((uint16_t)(*((uint8_t *)(ptr)))) \
              | ((uint16_t)(*(((uint8_t *)(ptr))+1)) << 8U) )
#endif

/* ActiveLook Commands service UUID
 * 0783B03E-8535-B5A0-7140-A304D2495CB7 */
static const uint8_t ACTIVELK_SERVICE_UUID[16] =
{
  0xB7, 0x5C, 0x49, 0xD2,
  0x04, 0xA3, 0x40, 0x71,
  0xA0, 0xB5, 0x35, 0x85,
  0x3E, 0xB0, 0x83, 0x07
};

/* Rx characteristic UUID (write to glasses)
 * 0783B03E-8535-B5A0-7140-A304D2495CBA */
static const uint8_t ACTIVELK_RX_CHAR_UUID[16] =
{
  0xBA, 0x5C, 0x49, 0xD2,
  0x04, 0xA3, 0x40, 0x71,
  0xA0, 0xB5, 0x35, 0x85,
  0x3E, 0xB0, 0x83, 0x07
};

/* Tx characteristic UUID (notify from glasses)
 * 0783B03E-8535-B5A0-7140-A304D2495CB8 */
static const uint8_t ACTIVELK_TX_CHAR_UUID[16] =
{
  0xB8, 0x5C, 0x49, 0xD2,
  0x04, 0xA3, 0x40, 0x71,
  0xA0, 0xB5, 0x35, 0x85,
  0x3E, 0xB0, 0x83, 0x07
};

/* CTRL characteristic UUID (flow control, notify from glasses)
 * 0783B03E-8535-B5A0-7140-A304D2495CB9 */
static const uint8_t ACTIVELK_CTRL_CHAR_UUID[16] =
{
  0xB9, 0x5C, 0x49, 0xD2,
  0x04, 0xA3, 0x40, 0x71,
  0xA0, 0xB5, 0x35, 0x85,
  0x3E, 0xB0, 0x83, 0x07
};

/* Discovery states, now extended to handle battery service as well */
typedef enum {
    DISC_STATE_IDLE = 0,
    DISC_STATE_EXCH_MTU,
    DISC_STATE_SVC_IN_PROGRESS,
    DISC_STATE_CHAR_IN_PROGRESS,
    DISC_STATE_DESC_IN_PROGRESS,
    DISC_STATE_BATTERY_NOTIFY_WRITE,
    DISC_STATE_TX_DESC_IN_PROGRESS,
    DISC_STATE_TX_NOTIFY_WRITE,
    DISC_STATE_CTRL_DESC_IN_PROGRESS,
    DISC_STATE_CTRL_NOTIFY_WRITE
} DiscoveryState_t;

/* We define a separate enum to track which service we are scanning for chars. */
typedef enum {
    SERVICE_NONE = 0,
    SERVICE_ACTIVELOOK,
    SERVICE_BATTERY
} WhichService_t;

typedef struct
{
    uint16_t connHandle;
    DiscoveryState_t discState;

    /* ActiveLook service discovery */
    uint8_t  serviceFound;
    uint16_t serviceStartHandle;
    uint16_t serviceEndHandle;

    uint8_t  rxCharFound;
    uint16_t rxCharHandle;

    uint8_t  txCharFound;
    uint16_t txCharHandle;

    uint16_t txCCCDHandle;

    uint8_t  ctrlCharFound;
    uint16_t ctrlCharHandle;
    uint16_t ctrlCCCDHandle;
    uint8_t  flowCtrlPaused;
    uint8_t  writePending;
    uint8_t  uploadActive;

    uint32_t cfgReadVersion;
    uint8_t  cfgReadPending;
    void (*cfgReadCallback)(uint8_t found, uint32_t version);

    /* Store battery service info */
    uint8_t  batteryServiceFound;
    uint16_t batteryServiceStartHandle;
    uint16_t batteryServiceEndHandle;

    uint8_t  batteryCharFound;
    uint16_t batteryCharHandle;
    uint8_t  lastBatteryPercent; /* store the last known battery percent */

    /* Sore the discovered CCC descriptor for the battery char */
    uint16_t batteryCCCDHandle;

    /* Keep track of which service we are currently discovering chars for */
    WhichService_t whichService;

    /* Track which handle we requested to read, so we know how to interpret the response. */
    uint16_t currentReadHandle;

    /* Callback interface if set */
    const FS_ActiveLook_ClientCb_t *cb;
} FS_ActiveLook_Client_Context_t;

static FS_ActiveLook_Client_Context_t g_ctx;

/******************************************************************************
 * Initialize
 ******************************************************************************/
void FS_ActiveLook_Client_Init(void)
{
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.discState = DISC_STATE_IDLE;
}

/******************************************************************************
 * Register optional callback interface
 ******************************************************************************/
void FS_ActiveLook_Client_RegisterCb(const FS_ActiveLook_ClientCb_t *cb)
{
    g_ctx.cb = cb;
}

/******************************************************************************
 * Start discovery (including MTU exchange) after connecting
 ******************************************************************************/
void FS_ActiveLook_Client_StartDiscovery(uint16_t connectionHandle)
{
    g_ctx.connHandle = connectionHandle;
    g_ctx.discState  = DISC_STATE_EXCH_MTU;  /* Start with MTU exchange */

    /* Clear everything so we re-discover from scratch */
    g_ctx.serviceFound          = 0;
    g_ctx.serviceStartHandle    = 0;
    g_ctx.serviceEndHandle      = 0;
    g_ctx.rxCharFound           = 0;
    g_ctx.rxCharHandle          = 0;
    g_ctx.txCharFound           = 0;
    g_ctx.txCharHandle          = 0;
    g_ctx.txCCCDHandle          = 0;
    g_ctx.ctrlCharFound         = 0;
    g_ctx.ctrlCharHandle        = 0;
    g_ctx.ctrlCCCDHandle        = 0;
    g_ctx.flowCtrlPaused        = 0;
    g_ctx.writePending          = 0;
    g_ctx.uploadActive          = 0;
    g_ctx.cfgReadPending        = 0;
    g_ctx.cfgReadCallback       = NULL;

    g_ctx.batteryServiceFound   = 0;
    g_ctx.batteryServiceStartHandle = 0;
    g_ctx.batteryServiceEndHandle   = 0;
    g_ctx.batteryCharFound      = 0;
    g_ctx.batteryCharHandle     = 0;
    g_ctx.lastBatteryPercent    = 255; /* e.g. 255 can mean "unknown" */

    g_ctx.whichService = SERVICE_NONE;

    /* Step 1: request a bigger ATT MTU from the peripheral */
    tBleStatus s = aci_gatt_exchange_config(connectionHandle);
    if (s == BLE_STATUS_SUCCESS)
    {
        APP_DBG_MSG("ActiveLook_Client: Requesting MTU exchange...\n");
    }
    else
    {
        APP_DBG_MSG("ActiveLook_Client: aci_gatt_exchange_config fail=0x%02X\n", s);
        g_ctx.discState = DISC_STATE_IDLE;  // stop
    }
}

/******************************************************************************
 * Event Handler
 ******************************************************************************/
void FS_ActiveLook_Client_EventHandler(void *p_blecore_evt, uint8_t hci_event_evt_code)
{
    evt_blecore_aci *blecore_evt = (evt_blecore_aci*) p_blecore_evt;

    switch (blecore_evt->ecode)
    {
        /**********************************************************************
         * The peripheral responded to MTU exchange:
         *********************************************************************/
        case ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE:
        {
            /* This event indicates the peripheral accepted some MTU.  */
            aci_att_exchange_mtu_resp_event_rp0 *mtu_resp =
                (aci_att_exchange_mtu_resp_event_rp0*) blecore_evt->data;
            APP_DBG_MSG("ActiveLook_Client: ACI_ATT_EXCHANGE_MTU_RESP, final MTU=%d\r\n",
                        mtu_resp->Server_RX_MTU);
            /* Wait for ACI_GATT_PROC_COMPLETE_VSEVT_CODE to know the procedure is done. */
        }
        break;

        /**********************************************************************
         * GATT procedure complete => check if we were in MTU exchange, or
         * discovering service, or discovering char, etc.
         *********************************************************************/
        case ACI_GATT_PROC_COMPLETE_VSEVT_CODE:
        {
            aci_gatt_proc_complete_event_rp0 *pc =
                (aci_gatt_proc_complete_event_rp0*) blecore_evt->data;

            if (pc->Connection_Handle != g_ctx.connHandle)
                break; /* Not for us */

            if (g_ctx.writePending)
            {
                g_ctx.writePending = 0;
                if (!g_ctx.uploadActive)
                    UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
            }

            if (g_ctx.discState == DISC_STATE_EXCH_MTU)
            {
                /* Done exchanging MTU => discover all primary services */
                g_ctx.discState = DISC_STATE_SVC_IN_PROGRESS;
                tBleStatus s = aci_gatt_disc_all_primary_services(g_ctx.connHandle);
                if (s == BLE_STATUS_SUCCESS)
                {
                    APP_DBG_MSG("ActiveLook_Client: MTU ok, now discovering all services...\n");
                }
                else
                {
                    APP_DBG_MSG("ActiveLook_Client: disc_all_primary_services fail=0x%02X\n", s);
                    g_ctx.discState = DISC_STATE_IDLE;
                }
            }
            else if (g_ctx.discState == DISC_STATE_SVC_IN_PROGRESS)
            {
                /* Done discovering all services. Next, discover chars. */
                if (g_ctx.serviceFound)
                {
                    g_ctx.discState    = DISC_STATE_CHAR_IN_PROGRESS;
                    g_ctx.whichService = SERVICE_ACTIVELOOK;
                    tBleStatus s = aci_gatt_disc_all_char_of_service(
                                       g_ctx.connHandle,
                                       g_ctx.serviceStartHandle,
                                       g_ctx.serviceEndHandle);
                    if (s == BLE_STATUS_SUCCESS)
                    {
                        APP_DBG_MSG("ActiveLook_Client: Discovering chars in ActiveLook service...\n");
                    }
                    else
                    {
                        APP_DBG_MSG("ActiveLook_Client: disc_all_char_of_service fail=0x%02X\n", s);
                        g_ctx.discState = DISC_STATE_IDLE;
                    }
                }
                else if (g_ctx.batteryServiceFound)
                {
                    g_ctx.discState    = DISC_STATE_CHAR_IN_PROGRESS;
                    g_ctx.whichService = SERVICE_BATTERY;
                    tBleStatus s = aci_gatt_disc_all_char_of_service(
                                       g_ctx.connHandle,
                                       g_ctx.batteryServiceStartHandle,
                                       g_ctx.batteryServiceEndHandle);
                    if (s == BLE_STATUS_SUCCESS)
                    {
                        APP_DBG_MSG("ActiveLook_Client: No AL service, but battery found; discovering battery char...\n");
                    }
                    else
                    {
                        APP_DBG_MSG("ActiveLook_Client: disc_all_char_of_service(battery) fail=0x%02X\n", s);
                        g_ctx.discState = DISC_STATE_IDLE;
                    }
                }
                else
                {
                    APP_DBG_MSG("ActiveLook_Client: No known services found.\n");
                    g_ctx.discState = DISC_STATE_IDLE;
                }
            }
            else if (g_ctx.discState == DISC_STATE_CHAR_IN_PROGRESS)
            {
                /* Done discovering chars for whichever service we were on. */
                if (g_ctx.whichService == SERVICE_ACTIVELOOK)
                {
                    /* If battery service also found, discover battery chars now */
                    if (g_ctx.batteryServiceFound)
                    {
                        g_ctx.whichService = SERVICE_BATTERY;
                        tBleStatus s = aci_gatt_disc_all_char_of_service(
                                           g_ctx.connHandle,
                                           g_ctx.batteryServiceStartHandle,
                                           g_ctx.batteryServiceEndHandle);
                        if (s == BLE_STATUS_SUCCESS)
                        {
                            APP_DBG_MSG("ActiveLook_Client: Now discovering battery char...\n");
                            return; /* Wait next event for the battery char discovery response */
                        }
                        else
                        {
                            APP_DBG_MSG("ActiveLook_Client: disc_all_char_of_service(battery) fail=0x%02X\n", s);
                        }
                    }
                }

                /* If we found batteryChar, let's now discover descriptors for it, else finalize. */
                if (g_ctx.batteryCharFound && (g_ctx.batteryCharHandle != 0))
                {
                    /* Move to descriptor discovery for battery. */
                    g_ctx.discState = DISC_STATE_DESC_IN_PROGRESS;
                    /* The standard approach: discover descriptors from (charValueHandle+1) to serviceEndHandle */
                    tBleStatus s = aci_gatt_disc_all_char_desc(
                                       g_ctx.connHandle,
                                       g_ctx.batteryCharHandle,
									   g_ctx.batteryCharHandle + 2);
                    if (s == BLE_STATUS_SUCCESS)
                    {
                        APP_DBG_MSG("ActiveLook_Client: Discovering battery descriptors...\n");
                        return;
                    }
                    else
                    {
                        APP_DBG_MSG("ActiveLook_Client: desc_all_char_desc(battery) fail=0x%02X\n", s);
                    }
                }

                /* If no battery or if the above fails, finalize. */
                g_ctx.discState = DISC_STATE_IDLE;
                APP_DBG_MSG("ActiveLook_Client: Char discovery complete. Rx=0x%04X, Battery=0x%04X\n",
                            g_ctx.rxCharHandle, g_ctx.batteryCharHandle);

                if (g_ctx.rxCharFound && g_ctx.cb && g_ctx.cb->OnDiscoveryComplete)
                {
                    g_ctx.cb->OnDiscoveryComplete();
                }
            }
            else if (g_ctx.discState == DISC_STATE_DESC_IN_PROGRESS)
            {
                /* Done discovering battery descriptors. Enable battery notifications. */
                APP_DBG_MSG("ActiveLook_Client: Battery descriptor discovery complete.\n");

                if (g_ctx.batteryCCCDHandle != 0)
                {
                    tBleStatus s2 = FS_ActiveLook_Client_EnableBatteryNotifications();
                    APP_DBG_MSG("EnableBatteryNotifications => 0x%02X\n", s2);
                    if (s2 == BLE_STATUS_SUCCESS)
                    {
                        g_ctx.discState = DISC_STATE_BATTERY_NOTIFY_WRITE;
                    }
                    else
                    {
                        /* Battery notify failed, but continue to TX discovery */
                        g_ctx.discState = DISC_STATE_IDLE;
                        if (g_ctx.rxCharFound && g_ctx.cb && g_ctx.cb->OnDiscoveryComplete)
                            g_ctx.cb->OnDiscoveryComplete();
                    }
                }
                else
                {
                    /* No battery CCCD, skip to TX discovery or finalize */
                    g_ctx.discState = DISC_STATE_IDLE;
                    if (g_ctx.rxCharFound && g_ctx.cb && g_ctx.cb->OnDiscoveryComplete)
                        g_ctx.cb->OnDiscoveryComplete();
                }
            }
            else if (g_ctx.discState == DISC_STATE_BATTERY_NOTIFY_WRITE)
            {
                if (g_ctx.txCharFound && (g_ctx.txCharHandle != 0))
                {
                    g_ctx.discState = DISC_STATE_TX_DESC_IN_PROGRESS;
                    tBleStatus s3 = aci_gatt_disc_all_char_desc(
                                       g_ctx.connHandle,
                                       g_ctx.txCharHandle,
                                       g_ctx.txCharHandle + 2);
                    if (s3 == BLE_STATUS_SUCCESS)
                    {
                        APP_DBG_MSG("ActiveLook_Client: Discovering TX descriptors...\n");
                    }
                    else
                    {
                        APP_DBG_MSG("ActiveLook_Client: TX desc discovery fail=0x%02X\n", s3);
                        g_ctx.discState = DISC_STATE_IDLE;
                        if (g_ctx.rxCharFound && g_ctx.cb && g_ctx.cb->OnDiscoveryComplete)
                            g_ctx.cb->OnDiscoveryComplete();
                    }
                }
                else
                {
                    g_ctx.discState = DISC_STATE_IDLE;
                    if (g_ctx.rxCharFound && g_ctx.cb && g_ctx.cb->OnDiscoveryComplete)
                        g_ctx.cb->OnDiscoveryComplete();
                }
            }
            else if (g_ctx.discState == DISC_STATE_TX_DESC_IN_PROGRESS)
            {
                /* TX descriptor discovery done. Enable TX notifications. */
                g_ctx.discState = DISC_STATE_IDLE;
                if (g_ctx.txCCCDHandle != 0)
                {
                    uint16_t enable = 0x0001;
                    tBleStatus s4 = aci_gatt_write_char_desc(
                                       g_ctx.connHandle,
                                       g_ctx.txCCCDHandle,
                                       2,
                                       (uint8_t*)&enable);
                    if (s4 == BLE_STATUS_SUCCESS)
                    {
                        APP_DBG_MSG("ActiveLook_Client: TX notifications enabled\n");
                        g_ctx.discState = DISC_STATE_TX_NOTIFY_WRITE;
                    }
                    else
                    {
                        APP_DBG_MSG("ActiveLook_Client: TX notify enable fail=0x%02X\n", s4);
                        if (g_ctx.rxCharFound && g_ctx.cb && g_ctx.cb->OnDiscoveryComplete)
                            g_ctx.cb->OnDiscoveryComplete();
                    }
                }
                else
                {
                    APP_DBG_MSG("ActiveLook_Client: No TX CCCD found\n");
                    if (g_ctx.rxCharFound && g_ctx.cb && g_ctx.cb->OnDiscoveryComplete)
                        g_ctx.cb->OnDiscoveryComplete();
                }
            }
            else if (g_ctx.discState == DISC_STATE_TX_NOTIFY_WRITE)
            {
                APP_DBG_MSG("ActiveLook_Client: TX notifications enabled OK\n");

                if (g_ctx.ctrlCharFound && (g_ctx.ctrlCharHandle != 0))
                {
                    g_ctx.discState = DISC_STATE_CTRL_DESC_IN_PROGRESS;
                    tBleStatus s5 = aci_gatt_disc_all_char_desc(
                                       g_ctx.connHandle,
                                       g_ctx.ctrlCharHandle,
                                       g_ctx.ctrlCharHandle + 2);
                    if (s5 == BLE_STATUS_SUCCESS)
                    {
                        APP_DBG_MSG("ActiveLook_Client: Discovering CTRL descriptors...\n");
                    }
                    else
                    {
                        APP_DBG_MSG("ActiveLook_Client: CTRL desc discovery fail=0x%02X\n", s5);
                        g_ctx.discState = DISC_STATE_IDLE;
                        if (g_ctx.rxCharFound && g_ctx.cb && g_ctx.cb->OnDiscoveryComplete)
                            g_ctx.cb->OnDiscoveryComplete();
                    }
                }
                else
                {
                    g_ctx.discState = DISC_STATE_IDLE;
                    APP_DBG_MSG("ActiveLook_Client: All discovery complete (no CTRL)\n");
                    if (g_ctx.rxCharFound && g_ctx.cb && g_ctx.cb->OnDiscoveryComplete)
                        g_ctx.cb->OnDiscoveryComplete();
                }
            }
            else if (g_ctx.discState == DISC_STATE_CTRL_DESC_IN_PROGRESS)
            {
                g_ctx.discState = DISC_STATE_IDLE;
                if (g_ctx.ctrlCCCDHandle != 0)
                {
                    uint16_t enable = 0x0001;
                    tBleStatus s6 = aci_gatt_write_char_desc(
                                       g_ctx.connHandle,
                                       g_ctx.ctrlCCCDHandle,
                                       2,
                                       (uint8_t*)&enable);
                    if (s6 == BLE_STATUS_SUCCESS)
                    {
                        APP_DBG_MSG("ActiveLook_Client: CTRL notifications enabling...\n");
                        g_ctx.discState = DISC_STATE_CTRL_NOTIFY_WRITE;
                    }
                    else
                    {
                        APP_DBG_MSG("ActiveLook_Client: CTRL notify enable fail=0x%02X\n", s6);
                        if (g_ctx.rxCharFound && g_ctx.cb && g_ctx.cb->OnDiscoveryComplete)
                            g_ctx.cb->OnDiscoveryComplete();
                    }
                }
                else
                {
                    APP_DBG_MSG("ActiveLook_Client: No CTRL CCCD found\n");
                    if (g_ctx.rxCharFound && g_ctx.cb && g_ctx.cb->OnDiscoveryComplete)
                        g_ctx.cb->OnDiscoveryComplete();
                }
            }
            else if (g_ctx.discState == DISC_STATE_CTRL_NOTIFY_WRITE)
            {
                g_ctx.discState = DISC_STATE_IDLE;
                FS_Log_WriteEvent("AL CTRL notifications enabled");
                APP_DBG_MSG("ActiveLook_Client: All discovery complete, TX+CTRL notifications active\n");
                if (g_ctx.rxCharFound && g_ctx.cb && g_ctx.cb->OnDiscoveryComplete)
                    g_ctx.cb->OnDiscoveryComplete();
            }
        }
        break;

        /**********************************************************************
         * "Read by group type" => primary service
         *********************************************************************/
        case ACI_ATT_READ_BY_GROUP_TYPE_RESP_VSEVT_CODE:
        {
            if (g_ctx.discState == DISC_STATE_SVC_IN_PROGRESS)
            {
                aci_att_read_by_group_type_resp_event_rp0 *pr =
                    (aci_att_read_by_group_type_resp_event_rp0*) blecore_evt->data;

                uint8_t idx = 0;
                while (idx < pr->Data_Length)
                {
                    uint16_t startHdl = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx]);
                    uint16_t endHdl   = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx+2]);
                    const uint8_t *uuid = &pr->Attribute_Data_List[idx+4];

                    /* The length is pr->Attribute_Data_Length for each record. */
                    /* If it’s 6, the UUID is 16 bits. If it’s 20, the UUID is 128 bits. */

                    if (pr->Attribute_Data_Length == 20)
                    {
                        /* 128-bit UUID */
                        if (memcmp(uuid, ACTIVELK_SERVICE_UUID, 16) == 0)
                        {
                            APP_DBG_MSG("ActiveLook_Client: Found ActiveLook Service 0x%04X–0x%04X\n", startHdl, endHdl);
                            g_ctx.serviceFound       = 1;
                            g_ctx.serviceStartHandle = startHdl;
                            g_ctx.serviceEndHandle   = endHdl;
                        }
                    }
                    else if (pr->Attribute_Data_Length == 6)
                    {
                        /* 16-bit UUID => check if 0x180F (Battery) */
                        uint16_t short_uuid = UNPACK_2_BYTE_PARAMETER(uuid);
                        if (short_uuid == BATTERY_SERVICE_UUID)
                        {
                            APP_DBG_MSG("ActiveLook_Client: Found Battery Service 0x%04X–0x%04X\n", startHdl, endHdl);
                            g_ctx.batteryServiceFound       = 1;
                            g_ctx.batteryServiceStartHandle = startHdl;
                            g_ctx.batteryServiceEndHandle   = endHdl;
                        }
                    }

                    idx += pr->Attribute_Data_Length;
                }
            }
        }
        break;

        /**********************************************************************
         * "Read by type response" => characteristics
         *********************************************************************/
        case ACI_ATT_READ_BY_TYPE_RESP_VSEVT_CODE:
        {
            if (g_ctx.discState == DISC_STATE_CHAR_IN_PROGRESS)
            {
                aci_att_read_by_type_resp_event_rp0 *pr =
                    (aci_att_read_by_type_resp_event_rp0*) blecore_evt->data;
                uint8_t idx = 0;
                while (idx < pr->Data_Length)
                {
                    /* typical format: 2B decl handle, 1B props, 2B value handle, then the UUID */
                    uint16_t declHandle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx]);
                    idx += 2;

                    uint8_t properties = pr->Handle_Value_Pair_Data[idx++];
                    uint16_t valHandle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx]);
                    idx += 2;

                    /* length of the UUID depends on pr->Handle_Value_Pair_Length */
                    uint8_t uuidLen = pr->Handle_Value_Pair_Length - 5; // we used up 5 bytes so far
                    const uint8_t *uuid = &pr->Handle_Value_Pair_Data[idx];
                    idx += uuidLen;

                    if (g_ctx.whichService == SERVICE_ACTIVELOOK)
                    {
                        if ((uuidLen == 16) && (memcmp(uuid, ACTIVELK_RX_CHAR_UUID, 16) == 0))
                        {
                            APP_DBG_MSG("ActiveLook_Client: Found RxChar=0x%04X\n", valHandle);
                            g_ctx.rxCharFound  = 1;
                            g_ctx.rxCharHandle = valHandle;
                        }
                        else if ((uuidLen == 16) && (memcmp(uuid, ACTIVELK_TX_CHAR_UUID, 16) == 0))
                        {
                            APP_DBG_MSG("ActiveLook_Client: Found TxChar=0x%04X\n", valHandle);
                            g_ctx.txCharFound  = 1;
                            g_ctx.txCharHandle = valHandle;
                        }
                        else if ((uuidLen == 16) && (memcmp(uuid, ACTIVELK_CTRL_CHAR_UUID, 16) == 0))
                        {
                            APP_DBG_MSG("ActiveLook_Client: Found CtrlChar=0x%04X\n", valHandle);
                            g_ctx.ctrlCharFound  = 1;
                            g_ctx.ctrlCharHandle = valHandle;
                        }
                    }
                    else if (g_ctx.whichService == SERVICE_BATTERY)
                    {
                        /* Compare to the known 16-bit Battery Level char UUID=0x2A19 */
                        if (uuidLen == 2)
                        {
                            uint16_t short_uuid = UNPACK_2_BYTE_PARAMETER(uuid);
                            if (short_uuid == BATTERY_LEVEL_CHAR_UUID)
                            {
                                APP_DBG_MSG("ActiveLook_Client: Found BatteryChar=0x%04X\n", valHandle);
                                g_ctx.batteryCharFound  = 1;
                                g_ctx.batteryCharHandle = valHandle;
                            }
                        }
                    }
                }
            }
        }
        break;

        /* Listen for notifications (including Battery Level) */
        case ACI_GATT_NOTIFICATION_VSEVT_CODE:
        {
            aci_gatt_notification_event_rp0 *pNotif =
                (aci_gatt_notification_event_rp0*) blecore_evt->data;

            if (pNotif->Connection_Handle == g_ctx.connHandle)
            {
                /* If it’s from the battery char handle, parse it. It's a 1-byte value [0..100]. */
                if (pNotif->Attribute_Handle == g_ctx.batteryCharHandle)
                {
                    if (pNotif->Attribute_Value_Length >= 1)
                    {
                        g_ctx.lastBatteryPercent = pNotif->Attribute_Value[0];
                        APP_DBG_MSG("ActiveLook_Client: Battery notification => %d%%\n", g_ctx.lastBatteryPercent);
                    }
                }
                /* Parse ActiveLook TX notifications (command responses) */
                if (g_ctx.txCharFound &&
                    pNotif->Attribute_Handle == g_ctx.txCharHandle &&
                    pNotif->Attribute_Value_Length >= 2)
                {
                    uint8_t *d = pNotif->Attribute_Value;
                    uint16_t len = pNotif->Attribute_Value_Length;

                    /* cfgRead response: FF D1 [fmt] [len] [version:4] [counts:5] AA */
                    if (d[0] == 0xFF && d[1] == 0xD1 && g_ctx.cfgReadPending)
                    {
                        g_ctx.cfgReadPending = 0;
                        /* Parse version from payload (after header) */
                        uint8_t hdrLen = (d[2] & 0x20) ? 5 : 4;
                        if (len >= hdrLen + 4)
                        {
                            uint32_t ver = ((uint32_t)d[hdrLen] << 24) |
                                           ((uint32_t)d[hdrLen+1] << 16) |
                                           ((uint32_t)d[hdrLen+2] << 8) |
                                           (uint32_t)d[hdrLen+3];
                            g_ctx.cfgReadVersion = ver;
                            APP_DBG_MSG("ActiveLook_Client: cfgRead version=%lu\n",
                                        (unsigned long)ver);
                            if (g_ctx.cfgReadCallback)
                                g_ctx.cfgReadCallback(1, ver);
                        }
                    }
                    else if (d[0] == 0xFF && d[1] == 0xE2)
                    {
                        uint8_t errCmd = (len > 4) ? d[4] : 0;
                        uint8_t errCode = (len > 5) ? d[5] : 0;
                        FS_Log_WriteEvent("AL TX err: cmd=0x%02X code=%d", errCmd, errCode);

                        if (g_ctx.cfgReadPending)
                        {
                            g_ctx.cfgReadPending = 0;
                            if (g_ctx.cfgReadCallback)
                                g_ctx.cfgReadCallback(0, 0);
                        }
                    }
                }
                /* CTRL characteristic notifications (flow control) */
                if (g_ctx.ctrlCharFound &&
                    pNotif->Attribute_Handle == g_ctx.ctrlCharHandle &&
                    pNotif->Attribute_Value_Length >= 1)
                {
                    uint8_t val = pNotif->Attribute_Value[0];
                    switch (val)
                    {
                    case 0x01:
                        g_ctx.flowCtrlPaused = 0;
                        FS_Log_WriteEvent("AL CTRL: flow ON");
                        break;
                    case 0x02:
                        g_ctx.flowCtrlPaused = 1;
                        FS_Log_WriteEvent("AL CTRL: flow OFF");
                        break;
                    case 0x03:
                        FS_Log_WriteEvent("AL CTRL err: corrupt cmd");
                        break;
                    case 0x04:
                        FS_Log_WriteEvent("AL CTRL err: RX overflow");
                        break;
                    case 0x06:
                        FS_Log_WriteEvent("AL CTRL err: missing cfgWrite");
                        break;
                    default:
                        FS_Log_WriteEvent("AL CTRL unknown=0x%02X", val);
                        break;
                    }
                }
            }
        }
        break;

        case ACI_ATT_FIND_INFO_RESP_VSEVT_CODE:
        {
            if (g_ctx.discState == DISC_STATE_DESC_IN_PROGRESS ||
                g_ctx.discState == DISC_STATE_TX_DESC_IN_PROGRESS ||
                g_ctx.discState == DISC_STATE_CTRL_DESC_IN_PROGRESS)
            {
                aci_att_find_info_resp_event_rp0 *resp =
                    (aci_att_find_info_resp_event_rp0 *)blecore_evt->data;

                if (resp->Format == 0x01)
                {
                    uint8_t numDesc = (resp->Event_Data_Length - 1) / 4;
                    uint8_t *ptr    = resp->Handle_UUID_Pair;
                    for (uint8_t i = 0; i < numDesc; i++)
                    {
                        uint16_t descHandle = UNPACK_2_BYTE_PARAMETER(ptr);
                        ptr += 2;
                        uint16_t descUUID   = UNPACK_2_BYTE_PARAMETER(ptr);
                        ptr += 2;

                        if (descUUID == 0x2902)
                        {
                            if (g_ctx.discState == DISC_STATE_DESC_IN_PROGRESS)
                            {
                                g_ctx.batteryCCCDHandle = descHandle;
                                APP_DBG_MSG("ActiveLook_Client: Found Battery CCCD=0x%04X\n", descHandle);
                            }
                            else if (g_ctx.discState == DISC_STATE_TX_DESC_IN_PROGRESS)
                            {
                                g_ctx.txCCCDHandle = descHandle;
                                APP_DBG_MSG("ActiveLook_Client: Found TX CCCD=0x%04X\n", descHandle);
                            }
                            else if (g_ctx.discState == DISC_STATE_CTRL_DESC_IN_PROGRESS)
                            {
                                g_ctx.ctrlCCCDHandle = descHandle;
                                APP_DBG_MSG("ActiveLook_Client: Found CTRL CCCD=0x%04X\n", descHandle);
                            }
                        }
                    }
                }
            }
        }
        break;

        case ACI_ATT_READ_RESP_VSEVT_CODE:
        {
            aci_att_read_resp_event_rp0 *pRead =
                (aci_att_read_resp_event_rp0*) blecore_evt->data;

            /* Check if it's our connection */
            if (pRead->Connection_Handle == g_ctx.connHandle)
            {
                /* If we had queued up a read to batteryCharHandle, interpret it now */
                if (g_ctx.currentReadHandle == g_ctx.batteryCharHandle)
                {
                    if (pRead->Event_Data_Length >= 1)
                    {
                        g_ctx.lastBatteryPercent = pRead->Attribute_Value[0];
                        APP_DBG_MSG("ActiveLook_Client: Immediate battery read => %d%%\n",
                                    g_ctx.lastBatteryPercent);
                    }
                }

                /* Reset it so we don't confuse subsequent read responses. */
                g_ctx.currentReadHandle = 0;
            }
        }
        break;

        default:
            break;
    }
}

/******************************************************************************
 * Check if Rx handle is ready (ActiveLook commands)
 ******************************************************************************/
uint8_t FS_ActiveLook_Client_IsReady(void)
{
    return (g_ctx.rxCharFound && g_ctx.rxCharHandle != 0);
}

/******************************************************************************
 * Write data to Rx characteristic (WWR)
 ******************************************************************************/
tBleStatus FS_ActiveLook_Client_WriteWithoutResp(const uint8_t *data, uint16_t length)
{
    if (!FS_ActiveLook_Client_IsReady())
    {
        APP_DBG_MSG("ActiveLook_Client: Not ready, no Rx handle.\n");
        return BLE_STATUS_FAILED;
    }

    tBleStatus s = aci_gatt_write_without_resp(g_ctx.connHandle,
                                               g_ctx.rxCharHandle,
                                               length,
                                               (uint8_t*)data);
    if (s != BLE_STATUS_SUCCESS)
    {
        APP_DBG_MSG("ActiveLook_Client: WWR error=0x%02X\n", s);
    }
    return s;
}

/******************************************************************************
 * Write data to Rx characteristic (Write With Response)
 ******************************************************************************/
tBleStatus FS_ActiveLook_Client_WriteWithResp(const uint8_t *data, uint16_t length)
{
    if (!FS_ActiveLook_Client_IsReady())
        return BLE_STATUS_FAILED;

    if (g_ctx.writePending)
        return BLE_STATUS_BUSY;

    tBleStatus s = aci_gatt_write_char_value(g_ctx.connHandle,
                                              g_ctx.rxCharHandle,
                                              length,
                                              (uint8_t*)data);
    if (s == BLE_STATUS_SUCCESS)
        g_ctx.writePending = 1;

    return s;
}

/******************************************************************************
 * Enable battery notifications
 ******************************************************************************/
tBleStatus FS_ActiveLook_Client_EnableBatteryNotifications(void)
{
    if (!g_ctx.batteryCharFound || (g_ctx.batteryCharHandle == 0))
    {
        APP_DBG_MSG("ActiveLook_Client: No Battery char found!\n");
        return BLE_STATUS_FAILED;
    }

    if (g_ctx.batteryCCCDHandle == 0)
    {
        APP_DBG_MSG("ActiveLook_Client: No Battery CCCD handle discovered!\n");
        return BLE_STATUS_FAILED;
    }

    /* The BLE spec for notifications: 0x0001 in little-endian => 0x01,0x00 */
    uint16_t enable = 0x0001;
    tBleStatus s = aci_gatt_write_char_desc(
                       g_ctx.connHandle,
                       g_ctx.batteryCCCDHandle,
                       2,
                       (uint8_t*)&enable);
    if (s == BLE_STATUS_SUCCESS)
    {
        APP_DBG_MSG("ActiveLook_Client: Battery notifications enabled.\n");
    }
    else
    {
        APP_DBG_MSG("ActiveLook_Client: EnableBatteryNotifications fail=0x%02X\n", s);
    }
    return s;
}

/******************************************************************************
 * Get the last known battery level
 ******************************************************************************/
uint8_t FS_ActiveLook_Client_GetBatteryLevel(void)
{
    return g_ctx.lastBatteryPercent;
}

void FS_ActiveLook_Client_ReadBatteryLevel(void)
{
    if (g_ctx.batteryCharFound && (g_ctx.batteryCharHandle != 0))
    {
        g_ctx.currentReadHandle = g_ctx.batteryCharHandle;
        aci_gatt_read_char_value(g_ctx.connHandle, g_ctx.batteryCharHandle);
    }
}

/******************************************************************************
 * Send cfgRead command to query config version
 ******************************************************************************/
void FS_ActiveLook_Client_CfgRead(void (*callback)(uint8_t found, uint32_t version))
{
    g_ctx.cfgReadCallback = callback;
    g_ctx.cfgReadPending = 1;

    /* Build cfgRead frame: FF D1 00 [len] "FLYSIGHT\0\0\0\0" AA */
    uint8_t packet[20];
    uint8_t idx = 0;
    packet[idx++] = 0xFF;
    packet[idx++] = 0xD1;
    packet[idx++] = 0x00;
    uint8_t lenPos = idx++;

    /* Config name padded to 12 bytes */
    const char *name = "FLYSIGHT";
    uint8_t nameLen = 8;
    memcpy(&packet[idx], name, nameLen);
    idx += nameLen;
    for (uint8_t pad = nameLen; pad < 12; pad++)
        packet[idx++] = 0x00;

    packet[idx++] = 0xAA;
    packet[lenPos] = idx;

    FS_ActiveLook_Client_WriteWithoutResp(packet, idx);
}

uint8_t FS_ActiveLook_Client_CanSend(void)
{
    return !g_ctx.flowCtrlPaused;
}

void FS_ActiveLook_Client_SetUploadActive(uint8_t active)
{
    g_ctx.uploadActive = active;
}

