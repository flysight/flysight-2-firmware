#include "main.h"
#include "activelook.h"
#include "activelook_client.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "stm32_seq.h"
#include <string.h>

/* 1) Define the states for our simple state machine */
typedef enum
{
    AL_STATE_INIT = 0,     /* Not discovered yet */
    AL_STATE_CLEAR,        /* Need to send "clear" command next */
    AL_STATE_READY,        /* Ready to send periodic updates */
    AL_STATE_UPDATE        /* Need to send a new GNSS update */
} FS_ActiveLook_State_t;

/* 2) Keep track of our current state and the last GNSS data */
static FS_ActiveLook_State_t s_state = AL_STATE_INIT;
static FS_GNSS_Data_t        s_gnssDataCache;

/* Forward references */
static void FS_ActiveLook_Task(void);
static void OnActiveLookDiscoveryComplete(void);

/* Callback struct for the ActiveLook_Client */
static const FS_ActiveLook_ClientCb_t s_alk_cb =
{
    .OnDiscoveryComplete = OnActiveLookDiscoveryComplete
};

/******************************************************************************
 * Initialization
 *****************************************************************************/
void FS_ActiveLook_Init(void)
{
    /* Register the callback for discovery */
    FS_ActiveLook_Client_RegisterCb(&s_alk_cb);

    /* Register our local "task" with the sequencer */
    UTIL_SEQ_RegTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, UTIL_SEQ_RFU, FS_ActiveLook_Task);

    /* Start in the INIT state */
    s_state = AL_STATE_INIT;

    /* If you want to automatically scan/connect to ActiveLook now: */
    UTIL_SEQ_SetTask(1 << CFG_TASK_START_SCAN_ID, CFG_SCH_PRIO_0);
}

/******************************************************************************
 * De-initialization
 *****************************************************************************/
void FS_ActiveLook_DeInit(void)
{
    /* If we want to disconnect from the device #1: */
    UTIL_SEQ_SetTask(1 << CFG_TASK_DISCONN_DEV_1_ID, CFG_SCH_PRIO_0);
}

/******************************************************************************
 * Called by the ActiveLook client once the Rx char is discovered.
 * We'll move the state to CLEAR, meaning "send Clear next" and schedule the task.
 *****************************************************************************/
static void OnActiveLookDiscoveryComplete(void)
{
    APP_DBG_MSG("ActiveLook: Discovery complete\n");
    s_state = AL_STATE_CLEAR;  /* We want to send a 'clear' next. */
    UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
}

/******************************************************************************
 * Called from your GNSS logic whenever new GNSS data is available.
 * If we're in READY state, copy the data, set state=UPDATE, schedule the task.
 * If not ready, we skip it.
 *****************************************************************************/
void FS_ActiveLook_GNSS_Update(const FS_GNSS_Data_t *current)
{
    /* We only want to queue an update if we're in AL_STATE_READY. */
    if (!FS_ActiveLook_Client_IsReady())
    {
        /* Not actually ready to send. It's discovered, but
         * maybe the BLE link has disconnected. Possibly do nothing here. */
        return;
    }

    if (s_state == AL_STATE_READY)
    {
        /* Copy the GNSS data for later use by FS_ActiveLook_Task() */
        s_gnssDataCache = *current;

        /* Now we want to send an update next time the task runs. */
        s_state = AL_STATE_UPDATE;
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
    }
    else
    {
        /* If we are still in CLEAR or UPDATE, or if not discovered, skip. */
        APP_DBG_MSG("ActiveLook_GNSS_Update: Not in READY, ignoring.\n");
    }
}

/******************************************************************************
 * This function is our "Task" in the sequencer, managing the state machine.
 * Each time we set s_state, we do UTIL_SEQ_SetTask(...). That eventually
 * calls here, which checks the current state and does the appropriate BLE write.
 *****************************************************************************/
static void FS_ActiveLook_Task(void)
{
    /* We'll build short packets on the stack. */
    uint8_t packet[64];
    uint8_t idx = 0;

    switch (s_state)
    {
    case AL_STATE_INIT:
        /* Haven't discovered the characteristic yet. Nothing to do. */
        break;

    case AL_STATE_CLEAR:
        /* Build a 'clear' command. (Check your doc for exact ID or format.)
         * Example: 0xFF 0x35 0x00 0x08  0x00 0x00 0x00 0xAA
         * We'll do a simple 5-byte version if your firmware supports it. */
        idx = 0;
        packet[idx++] = 0xFF;  // Start
        packet[idx++] = 0x35;  // Clear command ID (for example)
        packet[idx++] = 0x00;  // Format
        packet[idx++] = 5;     // Packet length
        packet[idx++] = 0xAA;  // Footer

        APP_DBG_MSG("ActiveLook: Sending CLEAR...\n");
        FS_ActiveLook_Client_WriteWithoutResp(packet, idx);

        /* Move to READY state. Next GNSS update will cause text to be drawn. */
        s_state = AL_STATE_READY;
        break;

    case AL_STATE_READY:
    default:
        /* No action needed. */
        break;

    case AL_STATE_UPDATE:
        /* Build a small 'txt' command using s_gnssDataCache. For example: */

        idx = 0;
        packet[idx++] = 0xFF;   // Start
        packet[idx++] = 0x37;   // "txt" command
        packet[idx++] = 0x00;   // Format: no query ID
        const uint8_t lengthPos = idx++; // We'll fill total length at the end

        /* For example, x=255, y=128, rotation=4, font=2, color=15 */
        packet[idx++] = 0x00; packet[idx++] = 255; // x=255 (big-endian might be 0x00, 0xFF)
        packet[idx++] = 0x00; packet[idx++] = 128; // y=128
        packet[idx++] = 4;    // rotation
        packet[idx++] = 2;    // font
        packet[idx++] = 15;   // color

        /* Build a short text from s_gnssDataCache. For instance: */
        char text[20];
        snprintf(text, sizeof(text), "iTOW:%lu", (unsigned long)(s_gnssDataCache.iTOW/1000));
        size_t textLen = strlen(text) + 1; /* +1 for '\0' terminator */
        memcpy(&packet[idx], text, textLen);
        idx += textLen;

        /* Footer */
        packet[idx++] = 0xAA;

        /* Fill in total length at 'lengthPos' */
        packet[lengthPos] = idx;

        APP_DBG_MSG("ActiveLook: Sending GNSS update...\n");
        FS_ActiveLook_Client_WriteWithoutResp(packet, idx);

        /* After sending it, return to READY. Next new GNSS data triggers another update. */
        s_state = AL_STATE_READY;
        break;
    }
}
