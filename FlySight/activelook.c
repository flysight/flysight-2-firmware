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

#include "main.h"
#include "activelook.h"
#include "activelook_client.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "stm32_seq.h"
#include <string.h>

/* Forward declaration */
static void OnActivelookDiscoveryComplete(void);
static void FS_Activelook_SendHelloWorld(void);

/* We'll define the callback struct */
static const FS_Activelook_ClientCb_t s_alk_cb =
{
    .OnDiscoveryComplete = OnActivelookDiscoveryComplete
};

void FS_Activelook_Init(void)
{
    /* Register the callback before scanning/connecting. */
    FS_Activelook_Client_RegisterCb(&s_alk_cb);

    /* Start scanning for BLE peripherals,
       leading eventually to connect to the Activelook device. */
    UTIL_SEQ_SetTask(1 << CFG_TASK_START_SCAN_ID, CFG_SCH_PRIO_0);
}

void FS_Activelook_DeInit(void)
{
    /* Disconnect from BLE device #1 */
    UTIL_SEQ_SetTask(1 << CFG_TASK_DISCONN_DEV_1_ID, CFG_SCH_PRIO_0);
}

/* This function is called once the client discovered the Rx characteristic. */
static void OnActivelookDiscoveryComplete(void)
{
    APP_DBG_MSG("Activelook: Discovery complete. Let's do 'Hello, world!'...\n");

    /* Example: call your HelloWorld sending function */
    FS_Activelook_SendHelloWorld();
}

/*
 * Example function that builds the 'txt' command to display "Hello, world!"
 * as you had previously. We can call it from OnActivelookDiscoveryComplete
 * or any time later, as long as FS_Activelook_Client_IsReady() is true.
 */
static void FS_Activelook_SendHelloWorld(void)
{
    if (!FS_Activelook_Client_IsReady())
    {
        APP_DBG_MSG("Activelook: Not ready, cannot send Hello!\n");
        return;
    }

    uint8_t packet[26];
    uint8_t index;

    index = 0;
    packet[index++] = 0xFF;
    packet[index++] = 0x01;     // 'clear'
    packet[index++] = 0x00;
    packet[index++] = 5;        // total length
    packet[index++] = 0xAA;

    APP_DBG_MSG("Activelook: Clearing screen\n");
    FS_Activelook_Client_WriteWithoutResp(packet, index);

    index = 0;
    packet[index++] = 0xFF;
    packet[index++] = 0x37;     // 'txt'
    packet[index++] = 0x00;
    packet[index++] = 26;       // total length

    // s16 x=255, y=128
    packet[index++] = 0; packet[index++] = 255;
    packet[index++] = 0; packet[index++] = 128;

    // rotation=0, font=2, color=15
    packet[index++] = 4;
    packet[index++] = 2;
    packet[index++] = 15;

    const char *text = "Hello, world!";
    size_t text_len = strlen(text) + 1;
    memcpy(&packet[index], text, text_len);
    index += text_len;

    packet[index++] = 0xAA;

    APP_DBG_MSG("Activelook: Sending Hello, world\n");
    FS_Activelook_Client_WriteWithoutResp(packet, index);
}
