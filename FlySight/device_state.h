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

#ifndef DEVICE_STATE_H_
#define DEVICE_STATE_H_

#include <stdint.h>

/**
 * @brief Handles commands written to the Device State (DS) Control Point characteristic.
 *
 * @param payload Pointer to the incoming command payload (first byte is cmd_opcode).
 * @param length Length of the payload.
 * @param conn_handle Connection handle.
 * @param notification_enabled_flag Flag indicating if notifications are enabled for the DS Control Point.
 */
void DeviceState_Handle_DS_ControlPointWrite(const uint8_t *payload, uint8_t length,
                                             uint16_t conn_handle, uint8_t notification_enabled_flag);

/**
 * @brief Initializes the Device State service module.
 */
void DeviceState_Init(void);

#endif /* DEVICE_STATE_H_ */
