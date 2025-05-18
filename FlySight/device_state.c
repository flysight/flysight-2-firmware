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

#include <string.h>     // For memcpy, strlen, snprintf
#include <stdio.h>      // For snprintf

#include "main.h"       // For NVIC_SystemReset and APP_DBG_MSG
#include "device_state.h"
#include "control_point_protocol.h"
#include "custom_stm.h"
#include "ble_tx_queue.h"
#include "version.h"    // For GIT_TAG
#include "state.h"      // For FS_State_Get()->device_id
#include "dbg_trace.h"

// Device State (DS) Control Point command opcodes
#define DS_CMD_GET_FW_VERSION  0x01 // Payload: (none)
                                    // Response Data: [version_string (variable length)]
#define DS_CMD_REBOOT_DEVICE   0x02 // Payload: (none)
#define DS_CMD_GET_DEVICE_ID   0x03 // Payload: (none)
                                    // Response Data: [device_id_hex_string (24 bytes)]

extern uint8_t SizeDs_Control_Point; // From custom_stm.c

void DeviceState_Init(void) {
    // Initialization for the device state service layer, if any.
}

// Helper to convert a uint32_t array (like device ID) to a hex string
static void uint32_array_to_hex_string(const uint32_t *data, uint32_t count, char *out_str, size_t out_str_len) {
    size_t offset = 0;
    for (uint32_t i = 0; i < count; ++i) {
        int written = snprintf(out_str + offset, out_str_len - offset, "%08lx", data[i]);
        if (written < 0 || (size_t)written >= (out_str_len - offset)) {
            // Buffer too small or error
            if (offset < out_str_len) out_str[offset] = '\0'; // Ensure null termination if possible
            return;
        }
        offset += written;
    }
}


void DeviceState_Handle_DS_ControlPointWrite(const uint8_t *payload, uint8_t length,
                                             uint16_t conn_handle, uint8_t notification_enabled_flag) {
    (void)conn_handle;

    uint8_t received_cmd_opcode = 0xFF;
    uint8_t status = CP_STATUS_ERROR_UNKNOWN;
    uint8_t response_data_buf[MAX_CP_OPTIONAL_RESPONSE_DATA_LEN];
    uint8_t response_data_len = 0;

    if (length < 1) {
        status = CP_STATUS_INVALID_PARAMETER;
    } else {
        received_cmd_opcode = payload[0];
        // const uint8_t *params = &payload[1]; // No params used yet
        uint8_t params_len = length - 1;

        switch (received_cmd_opcode) {
            case DS_CMD_GET_FW_VERSION:
                if (params_len != 0) {
                    status = CP_STATUS_INVALID_PARAMETER;
                } else {
                    strncpy((char*)response_data_buf, GIT_TAG, MAX_CP_OPTIONAL_RESPONSE_DATA_LEN);
                    response_data_buf[MAX_CP_OPTIONAL_RESPONSE_DATA_LEN] = '\0'; // Ensure null termination
                    response_data_len = strlen((char*)response_data_buf);
                    status = CP_STATUS_SUCCESS;
                }
                break;

            case DS_CMD_REBOOT_DEVICE:
                if (params_len != 0) {
                    status = CP_STATUS_INVALID_PARAMETER;
                } else {
                    APP_DBG_MSG("DS_CMD_REBOOT_DEVICE: Rebooting...\n");
                    status = CP_STATUS_SUCCESS; // We'll send success, then reboot
                    // Queue the response *before* reset. The reset might be too fast.
                    // This is a common pattern: acknowledge, then act.
                }
                break;

            case DS_CMD_GET_DEVICE_ID:
                if (params_len != 0) {
                    status = CP_STATUS_INVALID_PARAMETER;
                } else {
                    char hex_id_str[25]; // 3 * 8 hex chars + null
                    uint32_array_to_hex_string(FS_State_Get()->device_id, 3, hex_id_str, sizeof(hex_id_str));

                    response_data_len = strlen(hex_id_str);
                    if (response_data_len <= MAX_CP_OPTIONAL_RESPONSE_DATA_LEN) {
                         memcpy(response_data_buf, hex_id_str, response_data_len);
                         status = CP_STATUS_SUCCESS;
                    } else {
                        status = CP_STATUS_ERROR_UNKNOWN; // Buffer too small for ID string
                        response_data_len = 0;
                    }
                }
                break;

            default:
                status = CP_STATUS_CMD_NOT_SUPPORTED;
                break;
        }
    }

    if (notification_enabled_flag) {
        uint8_t final_response_packet[3 + MAX_CP_OPTIONAL_RESPONSE_DATA_LEN];
        uint8_t total_response_len = 3;

        final_response_packet[0] = CP_RESPONSE_ID;
        final_response_packet[1] = received_cmd_opcode;
        final_response_packet[2] = status;

        if (status == CP_STATUS_SUCCESS && response_data_len > 0) {
             if (response_data_len <= MAX_CP_OPTIONAL_RESPONSE_DATA_LEN) {
                memcpy(&final_response_packet[3], response_data_buf, response_data_len);
                total_response_len += response_data_len;
            } else {
                final_response_packet[2] = CP_STATUS_ERROR_UNKNOWN;
                response_data_len = 0;
                total_response_len = 3;
                 APP_DBG_MSG("DeviceState: Response data too long for cmd %02X\n", received_cmd_opcode);
            }
        }

        SizeDs_Control_Point = total_response_len;
        BLE_TX_Queue_SendTxPacket(CUSTOM_STM_DS_CONTROL_POINT,
                                  final_response_packet,
                                  total_response_len,
                                  &SizeDs_Control_Point,
                                  0);
    }

    // Handle actions that should occur *after* sending the response (like reboot)
    if (received_cmd_opcode == DS_CMD_REBOOT_DEVICE && status == CP_STATUS_SUCCESS) {
        // Optional: Add a small delay if needed to ensure BLE packet has a chance to be sent
        // For critical operations, ensuring the packet is truly on air is hard without deeper BLE stack hooks.
        // HAL_Delay(100); // Example: Small delay, but generally not ideal in event handlers
        NVIC_SystemReset();
    }
}
