/*
 * state.c
 *
 *  Created on: Apr. 19, 2023
 *      Author: Michael
 */

#include "main.h"
#include "ff.h"
#include "state.h"
#include "version.h"

#define TIMEOUT_VALUE 100

static FS_State_Data_t state;
static FIL stateFile;

extern RNG_HandleTypeDef hrng;

static void FS_State_WriteHex_8(FIL *file, const uint8_t *data, uint32_t count)
{
	uint32_t i;

	for (i = 0; i < count; ++i)
	{
		f_printf(file, "%02x", data[i]);
	}
}

static void FS_State_WriteHex_32(FIL *file, const uint32_t *data, uint32_t count)
{
	uint32_t i;

	for (i = 0; i < count; ++i)
	{
		f_printf(file, "%08x", data[i]);
	}
}

static void FS_State_Read(void)
{
	char    buffer[100];
	size_t  len;

	char    *name;
	char    *result;

	if (f_open(&stateFile, "/flysight.txt", FA_READ) != FR_OK)
		return;

	while (!f_eof(&stateFile))
	{
		f_gets(buffer, sizeof(buffer), &stateFile);

		len = strcspn(buffer, ";");
		buffer[len] = '\0';

		name = strtok(buffer, " \r\n\t:");
		if (name == 0) continue ;

		result = strtok(0, " \r\n\t:");
		if (result == 0) continue ;

		if (!strcmp(name, "Config_File"))
		{
			result[12] = '\0';
			strncpy(state.config_filename, result, sizeof(state.config_filename));
		}

		if (!strcmp(name, "Temp_Folder"))
		{
			state.temp_folder = atol(result);
		}
	}

	f_close(&stateFile);
}

static void FS_State_Write(void)
{
	const uint8_t * const pubkey = (uint8_t *) 0x08018200;

	// Open FlySight info file
	if (f_open(&stateFile, "/flysight.txt", FA_WRITE|FA_CREATE_ALWAYS) != FR_OK)
	{
		Error_Handler();
	}

	// Write device info
	f_printf(&stateFile, "; FlySight - http://flysight.ca\n\n");

	f_printf(&stateFile, "; Device information\n\n");

    f_printf(&stateFile, "Firmware_Ver: %s\n", GIT_TAG);

    f_printf(&stateFile, "Device_ID:    ");
	FS_State_WriteHex_32(&stateFile, state.device_id, 3);
	f_printf(&stateFile, "\n");

	f_printf(&stateFile, "Session_ID:   ");
	FS_State_WriteHex_32(&stateFile, state.session_id, 3);
	f_printf(&stateFile, "\n\n");

	f_printf(&stateFile, "; Persistent state\n\n");

	f_printf(&stateFile, "Config_File:  %s\n", state.config_filename);
	f_printf(&stateFile, "Temp_Folder:  %04lu\n\n", state.temp_folder);

	f_printf(&stateFile, "; Bootloader public key\n\n");

	f_printf(&stateFile, "Pubkey_X:     ");
	FS_State_WriteHex_8(&stateFile, pubkey, 32);
	f_printf(&stateFile, "\n");

	f_printf(&stateFile, "Pubkey_Y:     ");
	FS_State_WriteHex_8(&stateFile, pubkey + 32, 32);
	f_printf(&stateFile, "\n");

	// Close FlySight info file
	f_close(&stateFile);
}

void FS_State_Init(void)
{
	HAL_StatusTypeDef res;
	uint32_t counter;
	uint32_t tickstart;

	/* Initialize persistent state */
	state.config_filename[0] = 0;
	state.temp_folder = -1;

	/* Read current state */
	FS_State_Read();

	/* Increment temporary folder number */
	++state.temp_folder;

	/* Get device ID */
	state.device_id[0] = HAL_GetUIDw0();
	state.device_id[1] = HAL_GetUIDw1();
	state.device_id[2] = HAL_GetUIDw2();

	/* Algorithm to use RNG on CPU1 comes from AN5289 Figure 8 */

	/* Poll Sem0 until granted */
	LL_HSEM_1StepLock(HSEM, CFG_HW_RNG_SEMID);

	/* Configure and switch on RNG clock*/
	MX_RNG_Init();

	/* Generate random session ID */
	for (counter = 0; counter < 3; ++counter)
	{
	    tickstart = HAL_GetTick();

	    res = HAL_ERROR;
		while ((res != HAL_OK) && (HAL_GetTick() - tickstart < TIMEOUT_VALUE))
		{
			res = HAL_RNG_GenerateRandomNumber(&hrng, &state.session_id[counter]);
		}

		if (res != HAL_OK)
		{
			Error_Handler();
		}
	}

	/* Switch off RNG IP and clock */
	HAL_RNG_DeInit(&hrng);

	/* Set RNGSEL = CLK48 */
    LL_RCC_SetRNGClockSource(RCC_RNGCLKSOURCE_CLK48);

	/* Release Sem0 */
	LL_HSEM_ReleaseLock(HSEM, CFG_HW_RNG_SEMID, 0);

	/* Write updated state */
	FS_State_Write();
}

const FS_State_Data_t *FS_State_Get(void)
{
	return &state;
}

void FS_State_SetConfigFilename(const char *filename)
{
	/* Update config filename */
	strncpy(state.config_filename, filename, sizeof(state.config_filename));

	/* Write updated state */
	FS_State_Write();
}
