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
#include <stdbool.h>

#include "main.h"
#include "device_state.h"
#include "control_point_protocol.h"
#include "custom_stm.h"
#include "ble_tx_queue.h"
#include "mode.h"
#include "version.h"    // For GIT_TAG
#include "state.h"      // For FS_State_Get()->device_id
#include "dbg_trace.h"

// Device State (DS) Control Point command opcodes
#define DS_CMD_GET_FW_VERSION  0x01 // Payload: (none)
                                    // Response Data: [version_string (variable length)]
#define DS_CMD_REBOOT_DEVICE   0x02 // Payload: (none)
#define DS_CMD_GET_DEVICE_ID   0x03 // Payload: (none)
                                    // Response Data: [device_id_hex_string (24 bytes)]
#define DS_CMD_INSTALL_UPLOADED_FIRMWARE 0x04 // Payload: (none)

#define DS_CMD_REQUEST_SLEEP   0x10 // Payload: (none)
#define DS_CMD_REQUEST_ACTIVE  0x11 // Payload: (none)
#define DS_CMD_REQUEST_START   0x12 // Payload: (none)
#define DS_CMD_REQUEST_CONFIG  0x13 // Payload: (none)
#define DS_CMD_REQUEST_PAIRING 0x14 // Payload: (none)

#define DS_RESPONSE_FALLBACK_MSEC 200
#define DS_RESPONSE_FALLBACK_TICKS (DS_RESPONSE_FALLBACK_MSEC*1000/CFG_TS_TICK_VAL)
#define DS_RESPONSE_DEFER_TICKS 1U

typedef enum
{
    DS_PENDING_REQUEST_NONE,
    DS_PENDING_REQUEST_MODE,
    DS_PENDING_REQUEST_TERMINAL
} DS_PendingRequestKind_t;

typedef struct
{
    DS_PendingRequestKind_t kind;
    FS_Mode_State_t target_state;
    FS_TerminalAction_t terminal_action;
    bool waiting_for_tx;
    bool waiting_for_completion;
} DS_PendingRequest_t;

extern uint8_t SizeDs_Control_Point; // From custom_stm.c

static uint8_t response_timer_id;
static DS_PendingRequest_t pending_request = {
    DS_PENDING_REQUEST_NONE,
    FS_MODE_STATE_COUNT,
    FS_TERMINAL_ACTION_NONE,
    false,
    false
};

static uint8_t mode_result_to_cp_status(FS_Mode_RequestResult_t result);
static bool pending_request_busy(void);
static bool arm_pending_request(const DS_PendingRequest_t *request,
                                bool wait_for_indication);
static DS_PendingRequest_t take_pending_request(void);
static void run_pending_request(void);
static void response_fallback_timer(void);
static void response_tx_accepted(void);
static FS_Mode_RequestResult_t validate_terminal_request(void);
static FS_Mode_RequestResult_t validate_mode_request(FS_Mode_State_t target_state);
static bool mode_request_from_opcode(uint8_t opcode, FS_Mode_State_t *target_state);
static void send_response(uint8_t received_cmd_opcode, uint8_t status,
                          const uint8_t *response_data,
                          uint8_t response_data_len,
                          BLE_TX_Queue_callback_t callback);

void DeviceState_Init(void) {
    HW_TS_Create(CFG_TIM_PROC_ID_ISR, &response_timer_id,
                 hw_ts_SingleShot, response_fallback_timer);
}

void DeviceState_Handle_DS_ControlPointWrite(const uint8_t *payload, uint8_t length,
                                             uint16_t conn_handle, uint8_t notification_enabled_flag) {
    (void)conn_handle;

    uint8_t received_cmd_opcode = 0xFF;
    uint8_t status = CP_STATUS_ERROR_UNKNOWN;
    uint8_t response_data_buf[MAX_CP_OPTIONAL_RESPONSE_DATA_LEN];
    uint8_t response_data_len = 0;
    DS_PendingRequest_t request = {
        DS_PENDING_REQUEST_NONE,
        FS_MODE_STATE_COUNT,
        FS_TERMINAL_ACTION_NONE,
        false,
        false
    };

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
                    strncpy((char*)response_data_buf, GIT_TAG, MAX_CP_OPTIONAL_RESPONSE_DATA_LEN - 1);
                    response_data_buf[MAX_CP_OPTIONAL_RESPONSE_DATA_LEN - 1] = '\0'; // Ensure null termination
                    response_data_len = strlen((char*)response_data_buf);
                    status = CP_STATUS_SUCCESS;
                }
                break;

            case DS_CMD_REBOOT_DEVICE:
                if (params_len != 0) {
                    status = CP_STATUS_INVALID_PARAMETER;
                } else {
                    status = mode_result_to_cp_status(validate_terminal_request());
                    if (status == CP_STATUS_SUCCESS) {
                        request.kind = DS_PENDING_REQUEST_TERMINAL;
                        request.terminal_action = FS_TERMINAL_ACTION_RESET;
                    }
                }
                break;

            case DS_CMD_GET_DEVICE_ID:
                if (params_len != 0) {
                    status = CP_STATUS_INVALID_PARAMETER;
                } else {
                    // 12 raw bytes: the previous 24-character hex string
                    // exceeded MAX_CP_OPTIONAL_RESPONSE_DATA_LEN, so this
                    // command could never succeed
                    memcpy(response_data_buf, FS_State_Get()->device_id, 12);
                    response_data_len = 12;
                    status = CP_STATUS_SUCCESS;
                }
                break;

            case DS_CMD_INSTALL_UPLOADED_FIRMWARE:
                if (params_len != 0) {
                    status = CP_STATUS_INVALID_PARAMETER;
                } else {
                    status = mode_result_to_cp_status(validate_terminal_request());
                    if (status == CP_STATUS_SUCCESS) {
                        request.kind = DS_PENDING_REQUEST_TERMINAL;
                        request.terminal_action = FS_TERMINAL_ACTION_INSTALL_UPLOADED_FIRMWARE;
                    }
                }
                break;

            case DS_CMD_REQUEST_SLEEP:
            case DS_CMD_REQUEST_ACTIVE:
            case DS_CMD_REQUEST_START:
            case DS_CMD_REQUEST_CONFIG:
            case DS_CMD_REQUEST_PAIRING:
                if (params_len != 0) {
                    status = CP_STATUS_INVALID_PARAMETER;
                } else if (!mode_request_from_opcode(received_cmd_opcode, &request.target_state)) {
                    status = CP_STATUS_CMD_NOT_SUPPORTED;
                } else {
                    status = mode_result_to_cp_status(validate_mode_request(request.target_state));
                    if (status == CP_STATUS_SUCCESS) {
                        request.kind = DS_PENDING_REQUEST_MODE;
                    }
                }
                break;

            default:
                status = CP_STATUS_CMD_NOT_SUPPORTED;
                break;
        }
    }

    if ((status == CP_STATUS_SUCCESS) &&
        (request.kind != DS_PENDING_REQUEST_NONE) &&
        !arm_pending_request(&request, notification_enabled_flag != 0)) {
        status = CP_STATUS_BUSY;
        request.kind = DS_PENDING_REQUEST_NONE;
    }

    if (notification_enabled_flag) {
        send_response(received_cmd_opcode,
                      status,
                      response_data_buf,
                      response_data_len,
                      request.kind == DS_PENDING_REQUEST_NONE ?
                          0 : response_tx_accepted);
    }

    if ((request.kind != DS_PENDING_REQUEST_NONE) && !notification_enabled_flag) {
        HW_TS_Start(response_timer_id, DS_RESPONSE_DEFER_TICKS);
    }
}

void DeviceState_Handle_DS_ControlPointNotificationComplete(uint16_t attr_handle)
{
    bool should_run = false;
    uint32_t primask_bit;

    if (!Custom_STM_IsDsControlPointNotificationComplete(attr_handle)) {
        return;
    }

    primask_bit = __get_PRIMASK();
    __disable_irq();
    if ((pending_request.kind != DS_PENDING_REQUEST_NONE) &&
        pending_request.waiting_for_completion) {
        should_run = true;
    }
    __set_PRIMASK(primask_bit);

    if (should_run) {
        HW_TS_Stop(response_timer_id);
        run_pending_request();
    }
}

static uint8_t mode_result_to_cp_status(FS_Mode_RequestResult_t result)
{
    switch (result) {
        case FS_MODE_REQUEST_ACCEPTED:
            return CP_STATUS_SUCCESS;
        case FS_MODE_REQUEST_INVALID:
            return CP_STATUS_INVALID_PARAMETER;
        case FS_MODE_REQUEST_BUSY:
            return CP_STATUS_BUSY;
        case FS_MODE_REQUEST_NOT_ALLOWED:
            return CP_STATUS_OPERATION_NOT_PERMITTED;
        default:
            return CP_STATUS_ERROR_UNKNOWN;
    }
}

static bool pending_request_busy(void)
{
    bool busy;
    uint32_t primask_bit = __get_PRIMASK();

    __disable_irq();
    busy = (pending_request.kind != DS_PENDING_REQUEST_NONE);
    __set_PRIMASK(primask_bit);

    return busy;
}

static bool arm_pending_request(const DS_PendingRequest_t *request,
                                bool wait_for_indication)
{
    bool accepted = false;
    uint32_t primask_bit = __get_PRIMASK();

    __disable_irq();
    if (pending_request.kind == DS_PENDING_REQUEST_NONE) {
        pending_request = *request;
        pending_request.waiting_for_tx = wait_for_indication;
        pending_request.waiting_for_completion = false;
        accepted = true;
    }
    __set_PRIMASK(primask_bit);

    if (accepted && wait_for_indication) {
        HW_TS_Start(response_timer_id, DS_RESPONSE_FALLBACK_TICKS);
    }

    return accepted;
}

static DS_PendingRequest_t take_pending_request(void)
{
    DS_PendingRequest_t request;
    uint32_t primask_bit = __get_PRIMASK();

    __disable_irq();
    request = pending_request;
    pending_request.kind = DS_PENDING_REQUEST_NONE;
    pending_request.target_state = FS_MODE_STATE_COUNT;
    pending_request.terminal_action = FS_TERMINAL_ACTION_NONE;
    pending_request.waiting_for_tx = false;
    pending_request.waiting_for_completion = false;
    __set_PRIMASK(primask_bit);

    return request;
}

static void run_pending_request(void)
{
    DS_PendingRequest_t request = take_pending_request();
    FS_Mode_RequestResult_t result = FS_MODE_REQUEST_INVALID;

    if (request.kind == DS_PENDING_REQUEST_MODE) {
        result = FS_Mode_RequestState(request.target_state);
    } else if (request.kind == DS_PENDING_REQUEST_TERMINAL) {
        result = FS_Mode_RequestTerminalAction(request.terminal_action);
    }

    if ((request.kind != DS_PENDING_REQUEST_NONE) &&
        (result != FS_MODE_REQUEST_ACCEPTED)) {
        APP_DBG_MSG("DeviceState: deferred request dropped (kind=%u result=%u)\n",
                    (unsigned int)request.kind,
                    (unsigned int)result);
    }
}

static void response_fallback_timer(void)
{
    run_pending_request();
}

static void response_tx_accepted(void)
{
    uint32_t primask_bit = __get_PRIMASK();

    __disable_irq();
    if ((pending_request.kind != DS_PENDING_REQUEST_NONE) &&
        pending_request.waiting_for_tx) {
        pending_request.waiting_for_tx = false;
        pending_request.waiting_for_completion = true;
    }
    __set_PRIMASK(primask_bit);
}

static FS_Mode_RequestResult_t validate_terminal_request(void)
{
    if (pending_request_busy()) {
        return FS_MODE_REQUEST_BUSY;
    }

    if (FS_Mode_State() == FS_MODE_STATE_USB) {
        return FS_MODE_REQUEST_NOT_ALLOWED;
    }

    return FS_MODE_REQUEST_ACCEPTED;
}

static FS_Mode_RequestResult_t validate_mode_request(FS_Mode_State_t target_state)
{
    FS_Mode_State_t current_state = FS_Mode_State();

    if (pending_request_busy()) {
        return FS_MODE_REQUEST_BUSY;
    }

    if ((target_state >= FS_MODE_STATE_COUNT) ||
        (target_state == FS_MODE_STATE_USB)) {
        return FS_MODE_REQUEST_INVALID;
    }

    if (current_state == FS_MODE_STATE_USB) {
        return FS_MODE_REQUEST_NOT_ALLOWED;
    }

    if ((current_state == FS_MODE_STATE_SLEEP) ||
        (target_state == FS_MODE_STATE_SLEEP)) {
        return FS_MODE_REQUEST_ACCEPTED;
    }

    return FS_MODE_REQUEST_NOT_ALLOWED;
}

static bool mode_request_from_opcode(uint8_t opcode, FS_Mode_State_t *target_state)
{
    switch (opcode) {
        case DS_CMD_REQUEST_SLEEP:
            *target_state = FS_MODE_STATE_SLEEP;
            return true;
        case DS_CMD_REQUEST_ACTIVE:
            *target_state = FS_MODE_STATE_ACTIVE;
            return true;
        case DS_CMD_REQUEST_START:
            *target_state = FS_MODE_STATE_START;
            return true;
        case DS_CMD_REQUEST_CONFIG:
            *target_state = FS_MODE_STATE_CONFIG;
            return true;
        case DS_CMD_REQUEST_PAIRING:
            *target_state = FS_MODE_STATE_PAIRING;
            return true;
        default:
            return false;
    }
}

static void send_response(uint8_t received_cmd_opcode, uint8_t status,
                          const uint8_t *response_data,
                          uint8_t response_data_len,
                          BLE_TX_Queue_callback_t callback)
{
    uint8_t final_response_packet[3 + MAX_CP_OPTIONAL_RESPONSE_DATA_LEN];
    uint8_t total_response_len = 3;

    final_response_packet[0] = CP_RESPONSE_ID;
    final_response_packet[1] = received_cmd_opcode;
    final_response_packet[2] = status;

    if ((status == CP_STATUS_SUCCESS) && (response_data_len > 0)) {
        if (response_data_len <= MAX_CP_OPTIONAL_RESPONSE_DATA_LEN) {
            memcpy(&final_response_packet[3], response_data, response_data_len);
            total_response_len += response_data_len;
        } else {
            final_response_packet[2] = CP_STATUS_ERROR_UNKNOWN;
            APP_DBG_MSG("DeviceState: Response data too long for cmd %02X\n",
                        received_cmd_opcode);
        }
    }

    SizeDs_Control_Point = total_response_len;
    BLE_TX_Queue_SendTxPacket(CUSTOM_STM_DS_CONTROL_POINT,
                              final_response_packet,
                              total_response_len,
                              &SizeDs_Control_Point,
                              callback);
}
