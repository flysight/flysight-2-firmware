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

#ifndef ACTIVELOOK_CLIENT_H
#define ACTIVELOOK_CLIENT_H

#include <stdint.h>
#include "ble_types.h"

typedef struct
{
    void (*OnDiscoveryComplete)(void);
} FS_ActiveLook_ClientCb_t;

/* Initialize the ActiveLook client state machine */
void FS_ActiveLook_Client_Init(void);

/* Register optional callback interface */
void FS_ActiveLook_Client_RegisterCb(const FS_ActiveLook_ClientCb_t *cb);

/* Start discovery (including MTU exchange) after connecting */
void FS_ActiveLook_Client_StartDiscovery(uint16_t connectionHandle);

/* The main BLE event handler for ActiveLook */
void FS_ActiveLook_Client_EventHandler(void *p_blecore_evt, uint8_t hci_event_evt_code);

/* Check if ActiveLook Rx characteristic is ready */
uint8_t FS_ActiveLook_Client_IsReady(void);

/* Write data to Rx characteristic (WriteWithoutResp) */
tBleStatus FS_ActiveLook_Client_WriteWithoutResp(const uint8_t *data, uint16_t length);

/**
 * @brief Enable battery notifications for the standard Battery Service.
 *        This writes 0x01 to the CCC descriptor of the Battery Level char.
 *        Should be called after service/char discovery is complete.
 *
 * @return tBleStatus  BLE_STATUS_SUCCESS or an error code.
 */
tBleStatus FS_ActiveLook_Client_EnableBatteryNotifications(void);

/**
 * @brief Get the last known battery level from the glasses (0–100%).
 * @return uint8_t
 */
uint8_t FS_ActiveLook_Client_GetBatteryLevel(void);

#endif /* ACTIVELOOK_CLIENT_H */
