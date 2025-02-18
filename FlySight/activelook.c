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
    AL_STATE_SETUP_L2,
    AL_STATE_SETUP_L3,
    AL_STATE_SETUP_L4,
    AL_STATE_SETUP_PAGE,
    AL_STATE_CFG_SET,
    AL_STATE_CLEAR,          /* Need to send "clear" command */
    AL_STATE_READY,          /* Idle, waiting for GNSS data */
    AL_STATE_UPDATE,         /* GNSS arrived, update page */
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

        /* Update the page */
        s_state = AL_STATE_UPDATE;
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
    const char *unitsText,
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
    outBuf[idx++] = 0x00;

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

    // Text X (2 bytes, MSB first), say 200 => 0x00,0xC8
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0xC8;

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

    // Example: color=15
    extra[e++] = 0x03;  // sub-cmd ID "color"
    extra[e++] = 15;    // color=15

    // Example: font=1
    extra[e++] = 0x04;
    extra[e++] = 1;     // smaller font

    // Example: txt => position= (240,0)
    extra[e++] = 0x09;  // "text"
    // x=80 => 2 bytes
    extra[e++] = 0x00;
    extra[e++] = 80;
    // y=0 => 2 bytes
    extra[e++] = 0x00;
    extra[e++] = 35;

    // Then one byte with length, then the string
    uint8_t eLenPos = e++; // We'll fill the extra length later
    size_t unitsLen = strlen(unitsText);
    memcpy(&extra[e], unitsText, unitsLen);
    e += unitsLen;
    extra[eLenPos] = (uint8_t)unitsLen;

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
 * Build a "pageSave" command with 4 sublayouts. We'll call it ID=10, referencing
 * layoutIds=1..4, each at offset x=0,y=0 inside the page.
 *
 * Return the final length so caller can AL_SendRaw() it.
 ******************************************************************************/
static uint8_t AL_BuildPage(uint8_t pageId, uint8_t *outBuf)
{
    /* pageSave => command=0x80.
       Format => 0x00 => 1-byte length.
       The structure is: pageId, then for each sublayout:
         layoutId(1B), x(2B?), y(2B?), ...
       The exact number of bytes for x,y depends on your firmware doc.
       We'll assume x=2 bytes, y=2 bytes for safety. Or if doc says otherwise, adapt.
    */
    uint8_t idx = 0;

    outBuf[idx++] = 0xFF;  // start
    outBuf[idx++] = 0x80;  // "pageSave"
    outBuf[idx++] = 0x00;  // format => 1B length
    uint8_t lenPos = idx++;

    outBuf[idx++] = pageId;

    // We have 4 sublayouts: layout #1..4
    // Each sublayout => layoutId(1B), x(2B), y(2B)
    // We'll set x=0,y=0 for each.

    // sublayout #1
    outBuf[idx++] = 10; // layoutId
    outBuf[idx++] = 0x00; // x hi
    outBuf[idx++] = 0x00; // x lo
    outBuf[idx++] = 168; // y lo

    // sublayout #2
    outBuf[idx++] = 11;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 128;

    // sublayout #3
    outBuf[idx++] = 12;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 88;

    // sublayout #4
    outBuf[idx++] = 13;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 48;

    // Footer
    outBuf[idx++] = 0xAA;

    // Fill length
    outBuf[lenPos] = idx;

    return idx;
}

/*******************************************************************************
 * Build a single "pageClearAndDisplay" command with 4 text arguments,
 * separated by '\0'. That is the entire update in one WWR call.
 ******************************************************************************/
static uint8_t AL_BuildPageUpdate(uint8_t pageId,
                                  const char *line1,
                                  const char *line2,
                                  const char *line3,
                                  const char *line4,
                                  uint8_t *outBuf)
{
    /* layoutClearAndDisplay or pageClearAndDisplay => 0x6A
       structure => [ pageId, line1\0 line2\0 line3\0 line4\0 ]
    */
    uint8_t idx = 0;

    outBuf[idx++] = 0xFF;
    outBuf[idx++] = 0x86; // "pageClearAndDisplay"
    outBuf[idx++] = 0x00; // format => 1B length
    uint8_t lenPos = idx++;

    outBuf[idx++] = pageId;

    // Copy line1 plus a null
    size_t l1 = strlen(line1);
    memcpy(&outBuf[idx], line1, l1);
    idx += l1;
    outBuf[idx++] = 0;

    // line2
    size_t l2 = strlen(line2);
    memcpy(&outBuf[idx], line2, l2);
    idx += l2;
    outBuf[idx++] = 0;

    // line3
    size_t l3 = strlen(line3);
    memcpy(&outBuf[idx], line3, l3);
    idx += l3;
    outBuf[idx++] = 0;

    // line4
    size_t l4 = strlen(line4);
    memcpy(&outBuf[idx], line4, l4);
    idx += l4;
    outBuf[idx++] = 0;

    // Footer
    outBuf[idx++] = 0xAA;

    // fill length
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
        s_state = AL_STATE_SETUP_L2;
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
        break;

    case AL_STATE_SETUP_L2:
        length = AL_BuildLayout(11, "m/s", buf);
        AL_SendRaw(buf, length);
        APP_DBG_MSG("Layout #2 defined.\n");
        s_state = AL_STATE_SETUP_L3;
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
        break;

    case AL_STATE_SETUP_L3:
        length = AL_BuildLayout(12, "m/s", buf);
        AL_SendRaw(buf, length);
        APP_DBG_MSG("Layout #3 defined.\n");
        s_state = AL_STATE_SETUP_L4;
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
        break;

    case AL_STATE_SETUP_L4:
        length = AL_BuildLayout(13, "m", buf);
        AL_SendRaw(buf, length);
        APP_DBG_MSG("Layout #4 defined.\n");
        s_state = AL_STATE_SETUP_PAGE;
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
        break;

    case AL_STATE_SETUP_PAGE:
        length = AL_BuildPage(10, buf);
        AL_SendRaw(buf, length);
        APP_DBG_MSG("Page #10 referencing layouts #10..#13 defined.\n");
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


    case AL_STATE_UPDATE:
    {
        /* Build 4 strings from our GNSS data. Then do a single "pageClearAndDisplay" */
        char line1[16], line2[16], line3[16], line4[16];
        snprintf(line1, sizeof(line1), "%ld",(long)(s_gnssDataCache.heading));
        snprintf(line2, sizeof(line2), "%ld",(long)(s_gnssDataCache.gSpeed/100));
        snprintf(line3, sizeof(line3), "%ld",(long)(s_gnssDataCache.velD/1000));
        snprintf(line4, sizeof(line4), "%ld",(long)(s_gnssDataCache.hMSL/1000));

        length = AL_BuildPageUpdate(10, line1, line2, line3, line4, buf);
        AL_SendRaw(buf, length);
        APP_DBG_MSG("ActiveLook: Page #10 updated with heading/speed/etc.\n");

        s_state = AL_STATE_READY;
    }
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
