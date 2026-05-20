#include "main.h"
#include "activelook.h"
#include "activelook_client.h"
#include "activelook_config_data.h"
#include "activelook_mode0.h"
#include "app_common.h"
#include "config.h"
#include "dbg_trace.h"
#include "log.h"
#include "stm32_seq.h"
#include <string.h>
#include <stdio.h>

typedef enum
{
    AL_STATE_INIT = 0,
    AL_STATE_CFG_READ,
    AL_STATE_CFG_CHECK,
    AL_STATE_CFG_WRITE,
    AL_STATE_CONFIG_UPLOAD,
    AL_STATE_SETUP,
    AL_STATE_CFG_SET,
    AL_STATE_CFG_ACTIVATE,
    AL_STATE_CLEAR,
    AL_STATE_READY,
    AL_STATE_UPDATE,
} FS_ActiveLook_State_t;

static FS_ActiveLook_State_t s_state = AL_STATE_INIT;
static uint16_t s_config_cmd_idx = 0;
static uint8_t timer_id;

typedef struct {
    void (*init)(void);
    FS_ActiveLook_SetupStatus_t (*setup)(void);
    void (*update)(void);
} FS_ActiveLook_ModeOps;

static const FS_ActiveLook_ModeOps s_modeTable[] =
{
   {
        FS_ActiveLook_Mode0_Init,
        FS_ActiveLook_Mode0_Setup,
        FS_ActiveLook_Mode0_Update
   },
};

#define FS_ACTIVELOOK_NUM_MODES (sizeof(s_modeTable)/sizeof(s_modeTable[0]))
static const FS_ActiveLook_ModeOps *s_currentMode = NULL;

static void FS_ActiveLook_Timer(void);
static void FS_ActiveLook_Task(void);
static void OnActiveLookDiscoveryComplete(void);
static void OnCfgReadComplete(uint8_t found, uint32_t version);

static const FS_ActiveLook_ClientCb_t s_alk_cb =
{
    .OnDiscoveryComplete = OnActiveLookDiscoveryComplete
};

static void AL_SelectMode(uint8_t modeId)
{
    if (modeId < FS_ACTIVELOOK_NUM_MODES)
        s_currentMode = &s_modeTable[modeId];
    else
        s_currentMode = &s_modeTable[0];

    if (s_currentMode->init)
        s_currentMode->init();
}

static uint8_t AL_BuildCfgWritePacket(uint8_t *packet, uint32_t version)
{
    uint8_t idx = 0;
    packet[idx++] = 0xFF;
    packet[idx++] = 0xD0;
    packet[idx++] = 0x00;
    uint8_t lenPos = idx++;

    const char *name = "FLYSIGHT";
    memcpy(&packet[idx], name, 8);
    idx += 8;
    for (uint8_t pad = 8; pad < 12; pad++)
        packet[idx++] = 0x00;

    packet[idx++] = (version >> 24) & 0xFF;
    packet[idx++] = (version >> 16) & 0xFF;
    packet[idx++] = (version >> 8) & 0xFF;
    packet[idx++] = version & 0xFF;

    packet[idx++] = 0x01;
    packet[idx++] = 0x02;
    packet[idx++] = 0x03;
    packet[idx++] = 0x04;

    packet[idx++] = 0xAA;
    packet[lenPos] = idx;
    return idx;
}

static uint8_t AL_BuildCfgSetPacket(uint8_t *packet)
{
    uint8_t idx = 0;
    packet[idx++] = 0xFF;
    packet[idx++] = 0xD2;
    packet[idx++] = 0x00;
    uint8_t lenPos = idx++;

    const char *name = "FLYSIGHT";
    memcpy(&packet[idx], name, 8);
    idx += 8;
    for (uint8_t pad = 8; pad < 12; pad++)
        packet[idx++] = 0x00;

    packet[idx++] = 0xAA;
    packet[lenPos] = idx;
    return idx;
}

static void OnActiveLookDiscoveryComplete(void)
{
    FS_Log_WriteEvent("AL discovery complete");
    s_state = AL_STATE_CFG_READ;
    UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
    FS_ActiveLook_Client_ReadBatteryLevel();
}

static void FS_ActiveLook_Task(void)
{
    switch (s_state)
    {
    case AL_STATE_INIT:
        break;

    case AL_STATE_CFG_READ:
    {
        FS_ActiveLook_Client_CfgRead(OnCfgReadComplete);
        s_state = AL_STATE_CFG_CHECK;
        HW_TS_Start(timer_id, 2000 * 1000 / CFG_TS_TICK_VAL);
        break;
    }

    case AL_STATE_CFG_CHECK:
        break;

    case AL_STATE_CFG_WRITE:
    {
        if (!FS_ActiveLook_Client_CanSend())
        {
            UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_1);
            break;
        }
        uint8_t packet[32];
        uint8_t len = AL_BuildCfgWritePacket(packet, 0);
        tBleStatus s = FS_ActiveLook_Client_WriteWithResp(packet, len);
        if (s == BLE_STATUS_SUCCESS)
        {
            FS_Log_WriteEvent("AL cfgWrite (open)");
            FS_ActiveLook_Client_SetUploadActive(1);
            s_config_cmd_idx = 0;
            s_state = AL_STATE_CONFIG_UPLOAD;
            HW_TS_Start(timer_id, 50 * 1000 / CFG_TS_TICK_VAL);
        }
        else
        {
            UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_1);
        }
        break;
    }

    case AL_STATE_CONFIG_UPLOAD:
    {
        if (s_config_cmd_idx < AL_CONFIG_CMD_COUNT)
        {
            if (!FS_ActiveLook_Client_CanSend())
            {
                HW_TS_Start(timer_id, 50 * 1000 / CFG_TS_TICK_VAL);
                break;
            }

            uint16_t offset = al_config_cmd_offsets[s_config_cmd_idx];
            uint16_t length = al_config_cmd_lengths[s_config_cmd_idx];
            tBleStatus s = FS_ActiveLook_Client_WriteWithResp(
                &al_config_cmd_data[offset], length);

            if (s == BLE_STATUS_SUCCESS)
            {
                s_config_cmd_idx++;
                HW_TS_Start(timer_id, 50 * 1000 / CFG_TS_TICK_VAL);
            }
            else
            {
                UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_1);
            }
        }
        else
        {
            FS_ActiveLook_Client_SetUploadActive(0);
            FS_Log_WriteEvent("AL upload done: %d chunks", s_config_cmd_idx);
            AL_SelectMode(FS_Config_Get()->al_mode - 1);
            s_state = AL_STATE_SETUP;
            UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
        }
        break;
    }

    case AL_STATE_SETUP:
    {
        if (!FS_ActiveLook_Client_CanSend())
        {
            UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_1);
            break;
        }
        if (s_currentMode && s_currentMode->setup)
        {
        	FS_ActiveLook_SetupStatus_t status = s_currentMode->setup();
            if (status == FS_AL_SETUP_DONE)
            {
                FS_Log_WriteEvent("AL setup done");
                s_state = AL_STATE_CFG_SET;
                UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
            }
        }
        break;
    }

    case AL_STATE_CFG_SET:
    {
        if (!FS_ActiveLook_Client_CanSend())
        {
            UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_1);
            break;
        }
        FS_ActiveLook_Client_SetUploadActive(0);
        uint8_t packet[32];
        uint8_t len = AL_BuildCfgWritePacket(packet, AL_CONFIG_VERSION);
        tBleStatus s = FS_ActiveLook_Client_WriteWithResp(packet, len);
        if (s == BLE_STATUS_SUCCESS)
        {
            FS_Log_WriteEvent("AL cfgWrite (v%d)", AL_CONFIG_VERSION);
            s_state = AL_STATE_CFG_ACTIVATE;
        }
        else
        {
            UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_1);
        }
        break;
    }

    case AL_STATE_CFG_ACTIVATE:
    {
        if (!FS_ActiveLook_Client_CanSend())
        {
            UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_1);
            break;
        }
        uint8_t packet[20];
        uint8_t len = AL_BuildCfgSetPacket(packet);
        tBleStatus s = FS_ActiveLook_Client_WriteWithResp(packet, len);
        if (s == BLE_STATUS_SUCCESS)
        {
            FS_Log_WriteEvent("AL cfgSet (activate)");
            s_state = AL_STATE_CLEAR;
        }
        else
        {
            UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_1);
        }
        break;
    }

    case AL_STATE_CLEAR:
    {
        if (!FS_ActiveLook_Client_CanSend())
        {
            UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_1);
            break;
        }
        uint8_t packet[] = {0xFF, 0x01, 0x00, 0x05, 0xAA};
        tBleStatus s = FS_ActiveLook_Client_WriteWithResp(packet, sizeof(packet));
        if (s == BLE_STATUS_SUCCESS)
        {
            s_state = AL_STATE_READY;
            HW_TS_Start(timer_id, FS_Config_Get()->al_rate * 1000 / CFG_TS_TICK_VAL);
        }
        else
        {
            UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_1);
        }
        break;
    }

    case AL_STATE_READY:
        break;

    case AL_STATE_UPDATE:
        if (!FS_ActiveLook_Client_CanSend())
        {
            s_state = AL_STATE_READY;
            break;
        }
        if (s_currentMode && s_currentMode->update)
            s_currentMode->update();
        s_state = AL_STATE_READY;
        break;
    }
}

static void OnCfgReadComplete(uint8_t found, uint32_t version)
{
    if (found && version == AL_CONFIG_VERSION)
    {
        FS_Log_WriteEvent("AL cfg v%lu matches, skip upload", (unsigned long)version);
        FS_ActiveLook_Client_SetUploadActive(0);
        AL_SelectMode(FS_Config_Get()->al_mode - 1);
        s_state = AL_STATE_SETUP;
    }
    else
    {
        FS_Log_WriteEvent("AL cfg %s v%lu, uploading",
                          found ? "outdated" : "not found", (unsigned long)version);
        s_state = AL_STATE_CFG_WRITE;
    }
    UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
}

static void FS_ActiveLook_Timer(void)
{
    if (s_state == AL_STATE_READY)
    {
		s_state = AL_STATE_UPDATE;
		UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
    }
    else if (s_state == AL_STATE_CFG_CHECK)
    {
		FS_Log_WriteEvent("AL cfgRead timeout, uploading");
		s_state = AL_STATE_CFG_WRITE;
		UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
    }
    else if (s_state == AL_STATE_CONFIG_UPLOAD)
    {
		UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
    }
}

void FS_ActiveLook_Init(void)
{
    FS_ActiveLook_Client_RegisterCb(&s_alk_cb);
    UTIL_SEQ_RegTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, UTIL_SEQ_RFU, FS_ActiveLook_Task);
    s_state = AL_STATE_INIT;
    UTIL_SEQ_SetTask(1 << CFG_TASK_START_SCAN_ID, CFG_SCH_PRIO_0);
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_Repeated, FS_ActiveLook_Timer);
}

void FS_ActiveLook_DeInit(void)
{
	s_state = AL_STATE_INIT;
	HW_TS_Delete(timer_id);
    UTIL_SEQ_SetTask(1 << CFG_TASK_DISCONN_DEV_1_ID, CFG_SCH_PRIO_0);
}

uint8_t FS_ActiveLook_IsActive(void)
{
    return (s_state != AL_STATE_INIT) ? 1 : 0;
}
