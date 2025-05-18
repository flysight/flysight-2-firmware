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

#ifndef SENSOR_DATA_H_
#define SENSOR_DATA_H_

#include <stdint.h>

// --- Sensor Data (SD) Control Point Command Opcodes (Owned by this module) ---
#define SD_CMD_SET_GNSS_BLE_MASK   0x01 // Payload: [new_mask (1 byte)]
#define SD_CMD_GET_GNSS_BLE_MASK   0x02 // Payload: (none)
                                        // Response Data: [current_mask (1 byte)]
// ... other future SD specific commands, e.g., for enabling/disabling other sensor streams over BLE ...

/**
 * @brief Handles commands written to the Sensor Data (SD) Control Point characteristic.
 *
 * @param payload Pointer to the incoming command payload (first byte is cmd_opcode).
 * @param length Length of the payload.
 * @param conn_handle Connection handle (currently unused by this handler but passed for consistency).
 * @param notification_enabled_flag Flag indicating if notifications are enabled for the SD Control Point.
 */
void SensorData_Handle_SD_ControlPointWrite(const uint8_t *payload, uint8_t length,
                                            uint16_t conn_handle, uint8_t notification_enabled_flag);

/**
 * @brief Initializes the Sensor Data service module.
 */
void SensorData_Init(void); // If any specific init is needed for this service layer

#endif /* SENSOR_DATA_H_ */
