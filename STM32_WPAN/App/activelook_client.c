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
#include "dbg_trace.h"
#include "ble.h"
#include "tl.h"
#include <string.h>

#ifndef UNPACK_2_BYTE_PARAMETER
#define UNPACK_2_BYTE_PARAMETER(ptr) \
    (uint16_t)( ((uint16_t)(*((uint8_t *)(ptr)))) \
              | ((uint16_t)(*(((uint8_t *)(ptr))+1)) << 8U) )
#endif

/* 16-byte UUID for the Activelook Commands service
 * 0783B03E-8535-B5A0-7140-A304D2495CB7 */
static const uint8_t ACTIVELK_SERVICE_UUID[16] =
{
  0xB7, 0x5C, 0x49, 0xD2,
  0x04, 0xA3, 0x40, 0x71,
  0xA0, 0xB5, 0x35, 0x85,
  0x3E, 0xB0, 0x83, 0x07
};

/* 16-byte UUID for the Rx characteristic
 * 0783B03E-8535-B5A0-7140-A304D2495CBA */
static const uint8_t ACTIVELK_RX_CHAR_UUID[16] =
{
  0xBA, 0x5C, 0x49, 0xD2,
  0x04, 0xA3, 0x40, 0x71,
  0xA0, 0xB5, 0x35, 0x85,
  0x3E, 0xB0, 0x83, 0x07
};

typedef enum {
    DISC_STATE_IDLE = 0,
    DISC_STATE_EXCH_MTU,
    DISC_STATE_SVC_IN_PROGRESS,
    DISC_STATE_CHAR_IN_PROGRESS
} DiscoveryState_t;

typedef struct
{
    uint16_t connHandle;
    DiscoveryState_t discState;

    uint8_t  serviceFound;
    uint16_t serviceStartHandle;
    uint16_t serviceEndHandle;

    uint8_t  rxCharFound;
    uint16_t rxCharHandle;

    const FS_Activelook_ClientCb_t *cb;
} FS_Activelook_Client_Context_t;

static FS_Activelook_Client_Context_t g_ctx;

/******************************************************************************
 * Initialize
 ******************************************************************************/
void FS_Activelook_Client_Init(void)
{
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.discState = DISC_STATE_IDLE;
}

/******************************************************************************
 * Register optional callback interface
 ******************************************************************************/
void FS_Activelook_Client_RegisterCb(const FS_Activelook_ClientCb_t *cb)
{
    g_ctx.cb = cb;
}

/******************************************************************************
 * Start discovery (including MTU exchange) after connecting
 ******************************************************************************/
void FS_Activelook_Client_StartDiscovery(uint16_t connectionHandle)
{
    g_ctx.connHandle = connectionHandle;
    g_ctx.discState  = DISC_STATE_EXCH_MTU;  /* <--- Start with MTU exchange */
    g_ctx.serviceFound       = 0;
    g_ctx.rxCharFound        = 0;
    g_ctx.serviceStartHandle = 0;
    g_ctx.serviceEndHandle   = 0;
    g_ctx.rxCharHandle       = 0;

    /* Step 1: request a bigger ATT MTU from the peripheral */
    tBleStatus s = aci_gatt_exchange_config(connectionHandle);
    if (s == BLE_STATUS_SUCCESS)
    {
        APP_DBG_MSG("ActivelookClient: Requesting MTU exchange...\n");
    }
    else
    {
        APP_DBG_MSG("ActivelookClient: aci_gatt_exchange_config fail=0x%02X\n", s);
        g_ctx.discState = DISC_STATE_IDLE;  // stop
    }
}

/******************************************************************************
 * Event Handler
 ******************************************************************************/
void FS_Activelook_Client_EventHandler(void *p_blecore_evt, uint8_t hci_event_evt_code)
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
            APP_DBG_MSG("ActivelookClient: ACI_ATT_EXCHANGE_MTU_RESP, final MTU=%d\r\n",
                        mtu_resp->Server_RX_MTU);
            /* We still must wait for ACI_GATT_PROC_COMPLETE_VSEVT_CODE, which
               means the procedure is fully done in the stack. */
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

            /* See which sub-procedure we just completed by our discState */
            if (g_ctx.discState == DISC_STATE_EXCH_MTU)
            {
                /* The MTU exchange is done; proceed to discover the service. */
                g_ctx.discState = DISC_STATE_SVC_IN_PROGRESS;
                tBleStatus s = aci_gatt_disc_all_primary_services(g_ctx.connHandle);
                if (s == BLE_STATUS_SUCCESS)
                {
                    APP_DBG_MSG("ActivelookClient: MTU ok, now discovering service...\n");
                }
                else
                {
                    APP_DBG_MSG("ActivelookClient: disc_all_primary_services fail=0x%02X\n", s);
                    g_ctx.discState = DISC_STATE_IDLE;
                }
            }
            else if (g_ctx.discState == DISC_STATE_SVC_IN_PROGRESS)
            {
                /* Done discovering services. If we found it, discover chars. */
                if (g_ctx.serviceFound)
                {
                    g_ctx.discState = DISC_STATE_CHAR_IN_PROGRESS;
                    tBleStatus s = aci_gatt_disc_all_char_of_service(
                                       g_ctx.connHandle,
                                       g_ctx.serviceStartHandle,
                                       g_ctx.serviceEndHandle );
                    if (s == BLE_STATUS_SUCCESS)
                    {
                        APP_DBG_MSG("ActivelookClient: Discovering chars...\n");
                    }
                    else
                    {
                        APP_DBG_MSG("ActivelookClient: disc_all_char_of_service fail=0x%02X\n", s);
                        g_ctx.discState = DISC_STATE_IDLE;
                    }
                }
                else
                {
                    APP_DBG_MSG("ActivelookClient: service not found.\n");
                    g_ctx.discState = DISC_STATE_IDLE;
                }
            }
            else if (g_ctx.discState == DISC_STATE_CHAR_IN_PROGRESS)
            {
                /* Done discovering characteristics. */
                g_ctx.discState = DISC_STATE_IDLE;
                APP_DBG_MSG("ActivelookClient: Char discovery complete, Rx=0x%04X\n",
                            g_ctx.rxCharHandle);

                if (g_ctx.rxCharFound && g_ctx.cb && g_ctx.cb->OnDiscoveryComplete)
                {
                    g_ctx.cb->OnDiscoveryComplete();
                }
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

                    if (memcmp(uuid, ACTIVELK_SERVICE_UUID, 16) == 0)
                    {
                        APP_DBG_MSG("ActivelookClient: Found Service 0x%04X–0x%04X\n", startHdl, endHdl);
                        g_ctx.serviceFound       = 1;
                        g_ctx.serviceStartHandle = startHdl;
                        g_ctx.serviceEndHandle   = endHdl;
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
                    /* 2B decl handle, 1B props, 2B value handle, 16B UUID */
                    idx += 2; /* skip decl handle */
                    idx += 1; /* skip props */
                    uint16_t valH = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx]);
                    idx += 2;
                    const uint8_t *uuid = &pr->Handle_Value_Pair_Data[idx];
                    idx += 16;

                    if (memcmp(uuid, ACTIVELK_RX_CHAR_UUID, 16) == 0)
                    {
                        APP_DBG_MSG("ActivelookClient: Found RxChar=0x%04X\n", valH);
                        g_ctx.rxCharFound  = 1;
                        g_ctx.rxCharHandle = valH;
                    }
                }
            }
        }
        break;

        default:
            break;
    }
}

/******************************************************************************
 * Check if Rx handle is ready
 ******************************************************************************/
uint8_t FS_Activelook_Client_IsReady(void)
{
    return (g_ctx.rxCharFound && g_ctx.rxCharHandle != 0);
}

/******************************************************************************
 * Write data to Rx characteristic (WWR)
 ******************************************************************************/
tBleStatus FS_Activelook_Client_WriteWithoutResp(const uint8_t *data, uint16_t length)
{
    if (!FS_Activelook_Client_IsReady())
    {
        APP_DBG_MSG("ActivelookClient: Not ready, no Rx handle.\n");
        return BLE_STATUS_FAILED;
    }

    tBleStatus s = aci_gatt_write_without_resp(g_ctx.connHandle,
                                               g_ctx.rxCharHandle,
                                               length,
                                               (uint8_t*)data);
    if (s != BLE_STATUS_SUCCESS)
    {
        APP_DBG_MSG("ActivelookClient: WWR error=0x%02X\n", s);
    }
    return s;
}
