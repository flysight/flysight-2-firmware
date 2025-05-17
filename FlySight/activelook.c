#include "main.h"
#include "activelook.h"
#include "activelook_client.h"
#include "activelook_mode0.h"
#include "app_common.h"
#include "config.h"
#include "dbg_trace.h"
#include "stm32_seq.h"
#include <string.h>
#include <stdio.h>

#define ACTIVELOOK_UPDATE_MSEC    1000
#define ACTIVELOOK_UPDATE_RATE    (ACTIVELOOK_UPDATE_MSEC*1000/CFG_TS_TICK_VAL)

/*----- State machine states -----*/
typedef enum
{
    AL_STATE_INIT = 0,       /* Not discovered yet */
    AL_STATE_CFG_WRITE,
    AL_STATE_SETUP,
    AL_STATE_CFG_SET,
    AL_STATE_CLEAR,          /* Need to send "clear" command */
    AL_STATE_READY,          /* Idle, waiting for GNSS data */
    AL_STATE_UPDATE,         /* GNSS arrived, update page */
} FS_ActiveLook_State_t;

/*----- Module-level static variables -----*/
static FS_ActiveLook_State_t s_state = AL_STATE_INIT;

static uint8_t timer_id;

typedef struct {
    /**
     * Called exactly once when this mode is selected, before setup() is called.
     * This resets any static variables so we start from scratch.
     */
    void (*init)(void);

    /**
     * Called once (or multiple times) per pass of the main SM, until
     * it returns FS_AL_SETUP_DONE.
     */
    FS_ActiveLook_SetupStatus_t (*setup)(void);

    /**
     * Called whenever we want the mode to redraw or update data.
     */
    void (*update)(void);
} FS_ActiveLook_ModeOps;

static const FS_ActiveLook_ModeOps s_modeTable[] =
{
   { // mode 0
        FS_ActiveLook_Mode0_Init,
        FS_ActiveLook_Mode0_Setup,
        FS_ActiveLook_Mode0_Update
   },
   // etc
};

#define FS_ACTIVELOOK_NUM_MODES (sizeof(s_modeTable)/sizeof(s_modeTable[0]))
static const FS_ActiveLook_ModeOps *s_currentMode = NULL;

/* Forward references */
static void FS_ActiveLook_Timer(void);
static void FS_ActiveLook_Task(void);
static void OnActiveLookDiscoveryComplete(void);

/* Callback struct for the ActiveLook client library */
static const FS_ActiveLook_ClientCb_t s_alk_cb =
{
    .OnDiscoveryComplete = OnActiveLookDiscoveryComplete
};

void AL_SelectMode(uint8_t modeId)
{
    // Suppose you have s_modeTable[] with one entry per mode
    if (modeId < FS_ACTIVELOOK_NUM_MODES)
        s_currentMode = &s_modeTable[modeId];
    else
        s_currentMode = &s_modeTable[0];

    // Call the mode’s init if it exists
    if (s_currentMode->init)
        s_currentMode->init();
}

/*******************************************************************************
 * Called by the ActiveLook client once the Rx characteristic is discovered.
 * We'll move the state to CLEAR, meaning "clear" next, and schedule the task.
 ******************************************************************************/
static void OnActiveLookDiscoveryComplete(void)
{
    APP_DBG_MSG("ActiveLook: Discovery complete\n");

    // Initialize state
    s_state = AL_STATE_CFG_WRITE;

    // Begin updates
    UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);

    // Start Battery notifications
    FS_ActiveLook_Client_EnableBatteryNotifications();
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

        /* Get mode from config file */
        AL_SelectMode(FS_Config_Get()->al_mode - 1);

        /* Next config step */
        s_state = AL_STATE_SETUP;
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
        break;
    }

    case AL_STATE_SETUP:
    {
        if (s_currentMode && s_currentMode->setup)
        {
        	FS_ActiveLook_SetupStatus_t status = s_currentMode->setup();
            if (status == FS_AL_SETUP_DONE)
            {
                // Once the mode is fully set up, move on
                s_state = AL_STATE_CFG_SET;
            }
            else
            {
                // The mode still needs more calls
                // We'll come back here on next pass
            }
            UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
        }
        break;
    }

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

    	/* Start update timer */
    	HW_TS_Start(timer_id, ACTIVELOOK_UPDATE_RATE);
        break;
    }

    case AL_STATE_READY:
        /* Idle: no action */
        break;

    case AL_STATE_UPDATE:
        if (s_currentMode && s_currentMode->update)
            s_currentMode->update();
        s_state = AL_STATE_READY;
        break;
    }
}

static void FS_ActiveLook_Timer(void)
{
    if (s_state == AL_STATE_READY)
    {
		/* Update the page */
		s_state = AL_STATE_UPDATE;
		UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
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

	/* Initialize update timer */
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_Repeated, FS_ActiveLook_Timer);
}

void FS_ActiveLook_DeInit(void)
{
	/* Reset state */
	s_state = AL_STATE_INIT;

	/* Delete update timer */
	HW_TS_Delete(timer_id);

    /* Disconnect from the device if desired */
    UTIL_SEQ_SetTask(1 << CFG_TASK_DISCONN_DEV_1_ID, CFG_SCH_PRIO_0);
}
