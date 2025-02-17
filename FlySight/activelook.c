#include "main.h"
#include "activelook.h"
#include "activelook_client.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "stm32_seq.h"
#include <string.h>
#include <stdio.h>

/*----- State machine states -----*/
typedef enum
{
    AL_STATE_INIT = 0,       /* Not discovered yet */
    AL_STATE_CLEAR,          /* Need to send "clear" command */
    AL_STATE_READY,          /* Idle, waiting for GNSS data */
    AL_STATE_UPDATE_LINE_1,  /* GNSS arrived, show multiple lines */
    AL_STATE_UPDATE_LINE_2,
    AL_STATE_UPDATE_LINE_3,
    AL_STATE_UPDATE_LINE_4,
} FS_ActiveLook_State_t;

/*----- Module-level static variables -----*/
static FS_ActiveLook_State_t s_state = AL_STATE_INIT;
static FS_GNSS_Data_t        s_gnssDataCache;

/* Forward references */
static void FS_ActiveLook_Task(void);
static void OnActiveLookDiscoveryComplete(void);

/* Callback struct for the ActiveLook client library */
static const FS_ActiveLook_ClientCb_t s_alk_cb =
{
    .OnDiscoveryComplete = OnActiveLookDiscoveryComplete
};

/*******************************************************************************
 * Small helper to build and send a "txt" command to the ActiveLook display
 *  - x,y in 16-bit big-endian
 *  - rotation=4, font=2, color=15 (change as desired)
 *  - null-terminated text
 ******************************************************************************/
static void AL_SendTxtCmd(uint16_t x, uint16_t y, const char *text)
{
    uint8_t packet[64];
    uint8_t idx = 0;

    /* Start byte + Command ID for "txt" = 0x37 + no Query ID = 0x00 */
    packet[idx++] = 0xFF;       // start byte
    packet[idx++] = 0x37;       // "txt" command
    packet[idx++] = 0x00;       // format flags (0x00 means "no query ID")
    uint8_t lengthPos = idx++;  // We'll fill total packet length here at the end

    /* X coordinate (big endian) */
    packet[idx++] = (uint8_t)(x >> 8);
    packet[idx++] = (uint8_t)(x & 0xFF);

    /* Y coordinate (big endian) */
    packet[idx++] = (uint8_t)(y >> 8);
    packet[idx++] = (uint8_t)(y & 0xFF);

    /* rotation, font, color */
    packet[idx++] = 4;   // rotation
    packet[idx++] = 2;   // font
    packet[idx++] = 15;  // color

    /* Copy in the string + its null terminator */
    size_t textLen = strlen(text) + 1;
    memcpy(&packet[idx], text, textLen);
    idx += textLen;

    /* Footer byte */
    packet[idx++] = 0xAA;

    /* Fill total length */
    packet[lengthPos] = idx;

    /* Now send it with Write Without Response */
    FS_ActiveLook_Client_WriteWithoutResp(packet, idx);
}

/*******************************************************************************
 * Called by the ActiveLook client once the Rx characteristic is discovered.
 * We'll move the state to CLEAR, meaning "clear" next, and schedule the task.
 ******************************************************************************/
static void OnActiveLookDiscoveryComplete(void)
{
    APP_DBG_MSG("ActiveLook: Discovery complete\n");
    s_state = AL_STATE_CLEAR;
    UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
}

/*******************************************************************************
 * Called from your GNSS logic whenever new GNSS data is available.
 * If we're READY, copy data, set state=UPDATE_SETUP, and schedule the task.
 * Otherwise skip it.
 ******************************************************************************/
void FS_ActiveLook_GNSS_Update(const FS_GNSS_Data_t *current)
{
    /* Only queue an update if we are discovered & idle. */
    if (!FS_ActiveLook_Client_IsReady())
    {
        /* Possibly disconnected or not discovered. */
        return;
    }

    if (s_state == AL_STATE_READY)
    {
        /* Cache the data for later use */
        s_gnssDataCache = *current;

        /* We'll do multiple lines */
        s_state = AL_STATE_UPDATE_LINE_1;
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
    }
    else
    {
        APP_DBG_MSG("ActiveLook_GNSS_Update: State %d, ignoring.\n", s_state);
    }
}

/*******************************************************************************
 * Our main sequencer task. We handle the state machine, sending WWR commands
 * without blocking. Each line is sent in a separate state so that we don't
 * bombard the BLE stack with multiple WWR calls in a single pass.
 ******************************************************************************/
static void FS_ActiveLook_Task(void)
{
    char tmp[32];

    switch (s_state)
    {
    case AL_STATE_INIT:
        /* Not discovered yet, do nothing */
        break;

    case AL_STATE_CLEAR:
    {
        /* Example "clear" packet:
         * 0xFF 0x35 0x00 0x05  0xAA
         * Adjust if your firmware uses a different ID. */
        uint8_t packet[5];
        packet[0] = 0xFF;  // start
        packet[1] = 0x35;  // "clear" command ID
        packet[2] = 0x00;  // format
        packet[3] = 5;     // total length
        packet[4] = 0xAA;  // footer
        APP_DBG_MSG("ActiveLook: Clearing display...\n");
        FS_ActiveLook_Client_WriteWithoutResp(packet, sizeof(packet));

        /* Now go idle. Wait for FS_ActiveLook_GNSS_Update to set us to "update" */
        s_state = AL_STATE_READY;
        break;
    }

    case AL_STATE_READY:
        /* Idle: no action */
        break;

    case AL_STATE_UPDATE_LINE_1:
        /* Heading (s_gnssDataCache.heading), at y=93 */
        /* Example: "HDG: 123"  - you might scale heading if needed */
        snprintf(tmp, sizeof(tmp), "%ld", (long)s_gnssDataCache.heading);
        AL_SendTxtCmd(255, 208, tmp);

        /* Next line */
        s_state = AL_STATE_UPDATE_LINE_2;
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
        break;

    case AL_STATE_UPDATE_LINE_2:
        /* Horizontal speed (s_gnssDataCache.gSpeed/100) at y=128 */
        snprintf(tmp, sizeof(tmp), "%ld", (long)(s_gnssDataCache.gSpeed/100));
        AL_SendTxtCmd(255, 168, tmp);

        s_state = AL_STATE_UPDATE_LINE_3;
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
        break;

    case AL_STATE_UPDATE_LINE_3:
        /* Vertical speed (s_gnssDataCache.velD / 1000) at y=163 */
        snprintf(tmp, sizeof(tmp), "%ld", (long)(s_gnssDataCache.velD/1000));
        AL_SendTxtCmd(255, 128, tmp);

        s_state = AL_STATE_UPDATE_LINE_4;
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
        break;

    case AL_STATE_UPDATE_LINE_4:
        /* Elevation (s_gnssDataCache.hMSL / 1000) at y=198 */
        snprintf(tmp, sizeof(tmp), "%ld", (long)(s_gnssDataCache.hMSL/1000));
        AL_SendTxtCmd(255, 88, tmp);

        /* Done with this update. Return to READY. */
        s_state = AL_STATE_READY;
        break;
    }
}

/*******************************************************************************
 * Standard init/deinit for ActiveLook
 ******************************************************************************/
void FS_ActiveLook_Init(void)
{
    /* Register the callback for discovery */
    FS_ActiveLook_Client_RegisterCb(&s_alk_cb);

    /* Register our local "task" with the sequencer */
    UTIL_SEQ_RegTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, UTIL_SEQ_RFU, FS_ActiveLook_Task);

    /* Start in the INIT state */
    s_state = AL_STATE_INIT;

    /* If we want to automatically scan/connect: */
    UTIL_SEQ_SetTask(1 << CFG_TASK_START_SCAN_ID, CFG_SCH_PRIO_0);
}

void FS_ActiveLook_DeInit(void)
{
    /* Disconnect from the device if desired */
    UTIL_SEQ_SetTask(1 << CFG_TASK_DISCONN_DEV_1_ID, CFG_SCH_PRIO_0);
}
