/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2023 Bionic Avionics Inc.                                   **
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
#include "ff.h"
#include "shci.h"
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

static uint32_t hexCharToUint(char c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }
    else if (c >= 'a' && c <= 'f')
    {
        return 10 + (c - 'a');
    }
    else if (c >= 'A' && c <= 'F')
    {
        return 10 + (c - 'A');
    }
    return 0; // Optionally handle invalid character error
}

static void FS_State_ReadHex_32(const char *str, uint32_t *data, uint32_t count)
{
    uint32_t i, j, value;

    for (i = 0; i < count; ++i)
    {
        value = 0;
        for (j = 0; j < 8; ++j)
        {
            value = value << 4;
            value |= hexCharToUint(str[i * 8 + j]);
        }
        data[i] = value;
    }
}

static void FS_State_Read(void)
{
	char    buffer[100];
	size_t  len;

	char    *name;
	char    *result;
	int32_t val;

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

		val = atol(result);

		if (!strcmp(name, "Session_ID"))
		{
			FS_State_ReadHex_32(result, state.session_id, 3);
		}

		if (!strcmp(name, "Config_File"))
		{
			result[12] = '\0';
			strncpy(state.config_filename, result, sizeof(state.config_filename));
		}

		#define HANDLE_VALUE(s,w,r,t) \
			if ((t) && !strcmp(name, (s))) { (w) = (r); }

		HANDLE_VALUE("Temp_Folder", state.temp_folder,    val, val >= 0);
		HANDLE_VALUE("Charging",    state.charge_current, val, val >= 0 && val <= 3);

		#undef HANDLE_VALUE
	}

	f_close(&stateFile);
}

static void FS_State_Write(void)
{
	const uint8_t * const pubkey = (uint8_t *) 0x08018200;

	WirelessFwInfo_t WirelessInfo;

	// Read the firmware version of both the wireless stack and the FUS
	if (SHCI_GetWirelessFwInfo(&WirelessInfo) != SHCI_Success)
	{
		Error_Handler();
	}

	// Open FlySight info file
	if (f_open(&stateFile, "/flysight.txt", FA_WRITE|FA_CREATE_ALWAYS) != FR_OK)
	{
		Error_Handler();
	}

	// Write device info
	f_printf(&stateFile, "; FlySight - http://flysight.ca\n\n");

	f_printf(&stateFile, "; Firmware version\n\n");

	f_printf(&stateFile, "FUS_Ver:      %u.%u.%u\n",
			WirelessInfo.FusVersionMajor, WirelessInfo.FusVersionMinor, WirelessInfo.FusVersionSub);
	f_printf(&stateFile, "Stack_Ver:    %u.%u.%u\n",
			WirelessInfo.VersionMajor, WirelessInfo.VersionMinor, WirelessInfo.VersionSub);
	f_printf(&stateFile, "Firmware_Ver: %s\n\n", GIT_TAG);

	f_printf(&stateFile, "; Device information\n\n");

    f_printf(&stateFile, "Device_ID:    ");
	FS_State_WriteHex_32(&stateFile, state.device_id, 3);
	f_printf(&stateFile, "\n");

	f_printf(&stateFile, "Session_ID:   ");
	FS_State_WriteHex_32(&stateFile, state.session_id, 3);
	f_printf(&stateFile, "\n\n");

	f_printf(&stateFile, "; Persistent state\n\n");

	f_printf(&stateFile, "Config_File:  %s\n", state.config_filename);
	f_printf(&stateFile, "Temp_Folder:  %04lu\n\n", state.temp_folder);

	f_printf(&stateFile, "; System configuration\n\n");

	f_printf(&stateFile, "Charging:     %u ; 0 = No charging\n", state.charge_current);
	f_printf(&stateFile, "                ; 1 = 100 mA\n");
	f_printf(&stateFile, "                ; 2 = 200 mA (recommended)\n");
	f_printf(&stateFile, "                ; 3 = 300 mA\n\n");

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
	/* Initialize persistent state */
	state.config_filename[0] = 0;
	state.temp_folder = -1;
	state.charge_current = 2;

	/* Read current state */
	FS_State_Read();

	/* Get device ID */
	state.device_id[0] = HAL_GetUIDw0();
	state.device_id[1] = HAL_GetUIDw1();
	state.device_id[2] = HAL_GetUIDw2();
}

const FS_State_Data_t *FS_State_Get(void)
{
	return &state;
}

void FS_State_NextSession(void)
{
	HAL_StatusTypeDef res;
	uint32_t counter;
	uint32_t tickstart;

	/* Increment temporary folder number */
	state.temp_folder = (state.temp_folder + 1) % 10000;

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

void FS_State_SetConfigFilename(const char *filename)
{
	/* Update config filename */
	strncpy(state.config_filename, filename, sizeof(state.config_filename));

	/* Write updated state */
	FS_State_Write();
}
