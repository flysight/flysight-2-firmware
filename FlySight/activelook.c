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
    AL_STATE_CFG_WRITE,
    AL_STATE_SETUP_L1,
    AL_STATE_CFG_SET,
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
    s_state = AL_STATE_CFG_WRITE;
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
 * A helper to do a single WriteWithoutResp, building or storing a raw packet.
 * We do only one call per pass of the state machine to avoid spamming BLE.
 ******************************************************************************/
static void AL_SendRaw(const uint8_t *data, uint16_t length)
{
    FS_ActiveLook_Client_WriteWithoutResp(data, length);
}

/*******************************************************************************
 * Build a "layoutSave" command for one row, with or without an icon, etc.
 * We'll keep it simple: no icon here, just the normal text area. Adjust as needed.
 *
 * The snippet below demonstrates how you might build the layout data.
 * If you want an icon, add sub-commands for 'img', 'txt', etc. in "extra[]".
 *
 * Return the final size of the packet in 'outLen', so the caller can do:
 *    AL_SendRaw(buffer, outLen);
 ******************************************************************************/
static uint8_t AL_BuildLayout(
    uint8_t layoutId,
    uint8_t yOffset,
    uint8_t  *outBuf
)
{
    // Start building the buffer
    uint8_t idx = 0;
    outBuf[idx++] = 0xFF;  // Start
    outBuf[idx++] = 0x60;  // "layoutSave"
    outBuf[idx++] = 0x00;  // format => 1-byte length
    uint8_t lenPos = idx++; // We'll fill the total length later

    // layout ID
    outBuf[idx++] = layoutId;

    // We'll fill the additional commands size later
    uint8_t addCmdSizePos = idx++;

    // X (2 bytes, MSB first)
    // Suppose x=0 => 0x00, 0x00
    outBuf[idx++] = 0x00;  // x hi
    outBuf[idx++] = 0x00;  // x lo

    // Y (1 byte!)
    // Suppose y = 168 => 0xA8
    outBuf[idx++] = 0xA8;

    // Width (2 bytes, MSB first). E.g. 304 => 0x01,0x30
    outBuf[idx++] = 0x01;  // width hi
    outBuf[idx++] = 0x30;  // width lo

    // Height (1 byte!)
    // e.g. 40 => 0x28
    outBuf[idx++] = 0x28;

    // ForeColor, BackColor, Font
    outBuf[idx++] = 15; // foreColor
    outBuf[idx++] = 0;  // backColor
    outBuf[idx++] = 2;  // font

    // TextValid
    outBuf[idx++] = 1;  // 1 => use dynamic text argument

    // Text X (2 bytes, MSB first), say 255 => 0x00,0xFF
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0xFF;

    // Text Y (1 byte!), e.g. 40 => 0x28
    outBuf[idx++] = 0x28;

    // Text Rotation (1 byte)
    outBuf[idx++] = 4;

    // Text Opacity (1 byte)
    outBuf[idx++] = 1;

    //-------------------------------------------------------
    // Additional sub-commands for icon or units, if desired:
    //-------------------------------------------------------
    uint8_t extra[64];
    uint8_t e = 0;

    /* No subcommands yet */

    // Done building sub-commands
    outBuf[addCmdSizePos] = e;
    memcpy(&outBuf[idx], extra, e);
    idx += e;

    // Footer
    outBuf[idx++] = 0xAA;

    // Fill total length
    outBuf[lenPos] = idx;
    return idx;
}

/*******************************************************************************
 * Our main sequencer task. We handle the state machine, sending WWR commands
 * without blocking. Each line is sent in a separate state so that we don't
 * bombard the BLE stack with multiple WWR calls in a single pass.
 ******************************************************************************/
static void FS_ActiveLook_Task(void)
{
    char tmp[32];
    static uint8_t buf[128]; /* re-usable buffer for building commands */
    uint8_t length;

    switch (s_state)
    {
    case AL_STATE_INIT:
        /* Not discovered yet, do nothing */
        break;

    case AL_STATE_CFG_WRITE:
    {
        uint8_t packet[128];
        uint8_t idx = 0;

        packet[idx++] = 0xFF;  // start
        packet[idx++] = 0xD0;  // "cfgWrite" command ID
        packet[idx++] = 0x00;  // format
        uint8_t lenPos = idx++;

        snprintf(tmp, sizeof(tmp), "FLYSIGHT");
        size_t textLen = strlen(tmp);
        memcpy(&packet[idx], tmp, textLen);
        idx += textLen;

        while (textLen < 12)
        {
            packet[idx++] = 0x00; // padding
            textLen++;
        }

        packet[idx++] = 0x00;  // version
        packet[idx++] = 0x00;
        packet[idx++] = 0x00;
        packet[idx++] = 0x01;

        packet[idx++] = 0x01;  // password
        packet[idx++] = 0x02;
        packet[idx++] = 0x03;
        packet[idx++] = 0x04;

        /* Footer byte */
        packet[idx++] = 0xAA;

        /* Fill total length */
        packet[lenPos] = idx;

        /* Now send it with Write Without Response */
        FS_ActiveLook_Client_WriteWithoutResp(packet, idx);

        /* Next config step */
        s_state = AL_STATE_SETUP_L1;
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
        break;
    }

    case AL_STATE_SETUP_L1:
        length = AL_BuildLayout(10, "deg", buf);
        AL_SendRaw(buf, length);
        APP_DBG_MSG("Layout #1 defined.\n");

        /* Next config step */
        s_state = AL_STATE_CFG_SET;
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
        break;

    case AL_STATE_CFG_SET:
    {
        uint8_t packet[128];
        uint8_t idx = 0;

        packet[idx++] = 0xFF;  // start
        packet[idx++] = 0xD2;  // "cfgSet" command ID
        packet[idx++] = 0x00;  // format
        uint8_t lenPos = idx++;

        snprintf(tmp, sizeof(tmp), "FLYSIGHT");
        size_t textLen = strlen(tmp);
        memcpy(&packet[idx], tmp, textLen);
        idx += textLen;

        while (textLen < 12)
        {
            packet[idx++] = 0x00; // padding
            textLen++;
        }

        /* Footer byte */
        packet[idx++] = 0xAA;

        /* Fill total length */
        packet[lenPos] = idx;

        /* Now send it with Write Without Response */
        FS_ActiveLook_Client_WriteWithoutResp(packet, idx);

        /* Next config step */
        s_state = AL_STATE_CLEAR;
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
        break;
    }

    case AL_STATE_CLEAR:
    {
        /* Example "clear" packet:
         * 0xFF 0x35 0x00 0x05  0xAA
         * Adjust if your firmware uses a different ID. */
        uint8_t packet[5];
        packet[0] = 0xFF;  // start
        packet[1] = 0x01;  // "clear" command ID
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
    {
        uint8_t packet[128];
        uint8_t idx = 0;

        packet[idx++] = 0xFF;  // start
        packet[idx++] = 0x62;  // "layoutDisplay" command ID
        packet[idx++] = 0x00;  // format
        uint8_t lenPos = idx++;

        packet[idx++] = 10;    // layout ID

        /* Copy in the string + its null terminator */
//        snprintf(tmp, sizeof(tmp), "%ld", (long)s_gnssDataCache.heading);
        snprintf(tmp, sizeof(tmp), "TESTING");
        size_t textLen = strlen(tmp);
        memcpy(&packet[idx], tmp, textLen);
        idx += textLen;

        /* Footer byte */
        packet[idx++] = 0xAA;

        /* Fill total length */
        packet[lenPos] = idx;

        /* Now send it with Write Without Response */
        FS_ActiveLook_Client_WriteWithoutResp(packet, idx);

        /* Next line */
        s_state = AL_STATE_UPDATE_LINE_2;
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
        break;
    }

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
