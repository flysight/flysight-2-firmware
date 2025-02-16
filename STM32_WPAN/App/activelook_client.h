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

void FS_ActiveLook_Client_Init(void);
void FS_ActiveLook_Client_RegisterCb(const FS_ActiveLook_ClientCb_t *cb);
void FS_ActiveLook_Client_StartDiscovery(uint16_t connectionHandle);

void FS_ActiveLook_Client_EventHandler(void *p_blecore_evt, uint8_t hci_event_evt_code);

uint8_t FS_ActiveLook_Client_IsReady(void);
tBleStatus FS_ActiveLook_Client_WriteWithoutResp(const uint8_t *data, uint16_t length);

#endif /* ACTIVELOOK_CLIENT_H */
