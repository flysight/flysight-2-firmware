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

#include <stdbool.h>

#include "main.h"
#include "app_common.h"
#include "audio.h"
#include "ff.h"
#include "log.h"
#include "stm32_seq.h"

#define PLLSAI1_TIMEOUT_VALUE (2U) /* 2 ms */

#define MAX9850_ADDR 0x20

#define MAX9850_REG_STATUS_A      0x00
#define MAX9850_REG_STATUS_B      0x01
#define MAX9850_REG_VOLUME        0x02
#define MAX9850_REG_GENERAL       0x03
#define MAX9850_REG_INTERRUPT     0x04
#define MAX9850_REG_ENABLE        0x05
#define MAX9850_REG_CLOCK         0x06
#define MAX9850_REG_CHARGE_PUMP   0x07
#define MAX9850_REG_LRCLK_MSB     0x08
#define MAX9850_REG_LRCLK_LSB     0x09
#define MAX9850_REG_DIGITAL_AUDIO 0x0a
#define MAX9850_REG_RESERVED      0x0b

#define AUDIO_SAMPLE_RATE      24000
#define AUDIO_INDEX_BITS       7
#define AUDIO_INTERPOLATE_BITS 8

#define AUDIO_FRAME_LEN        2048

#define AUDIO_UPDATE_MSEC 40
#define AUDIO_UPDATE_RATE (AUDIO_UPDATE_MSEC*1000/CFG_TS_TICK_VAL)

static const int16_t sineTable[] =
{
		 0,   1607,   3211,   4807,   6392,   7961,   9511,  11038,
	 12539,  14009,  15446,  16845,  18204,  19519,  20787,  22004,
	 23169,  24278,  25329,  26318,  27244,  28105,  28897,  29621,
	 30272,  30851,  31356,  31785,  32137,  32412,  32609,  32727,
	 32767,  32727,  32609,  32412,  32137,  31785,  31356,  30851,
	 30272,  29621,  28897,  28105,  27244,  26318,  25329,  24278,
	 23169,  22004,  20787,  19519,  18204,  16845,  15446,  14009,
	 12539,  11038,   9511,   7961,   6392,   4807,   3211,   1607,
		 0,  -1608,  -3212,  -4808,  -6393,  -7962,  -9512, -11039,
	-12540, -14010, -15447, -16846, -18205, -19520, -20788, -22005,
	-23170, -24279, -25330, -26319, -27245, -28106, -28898, -29622,
	-30273, -30852, -31357, -31786, -32138, -32413, -32610, -32728,
	-32767, -32728, -32610, -32413, -32138, -31786, -31357, -30852,
	-30273, -29622, -28898, -28106, -27245, -26319, -25330, -24279,
	-23170, -22005, -20788, -19520, -18205, -16846, -15447, -14010,
	-12540, -11039,  -9512,  -7962,  -6393,  -4808,  -3212,  -1608
};

static uint32_t audioStep;
static uint32_t audioChirp;
static uint32_t audioLen = 0;

static int16_t audioBuffer[AUDIO_FRAME_LEN];

static volatile uint32_t frameCount;
static volatile uint32_t lastFrame;

static uint32_t readPos;
static uint32_t writePos;

static FIL audioFile;

static char audioList[AUDIO_LIST_LEN];
static char *audioListPtr;

static uint8_t audioVolume;

typedef enum
{
	AUDIO_IDLE,
	AUDIO_PLAY_TONE,
	AUDIO_PLAY_FILE,
	AUDIO_PLAY_LIST,
} FS_Audio_State_t;
static FS_Audio_State_t audioState = AUDIO_IDLE;

static uint8_t timer_id;

static uint32_t updateCount;
static uint32_t updateTotalTime;
static uint32_t updateMaxTime;
static uint32_t updateLastCall;
static uint32_t updateMaxInterval;
static uint32_t bufferUsed;

extern I2C_HandleTypeDef hi2c1;
extern SAI_HandleTypeDef hsai_BlockA1;

void FS_Audio_Stop(void);
static void FS_Audio_Timer(void);
static void FS_Audio_Update(void);

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	if ((++frameCount) != lastFrame)
	{
		// Begin DMA transfer
		HAL_SAI_Transmit_DMA(hsai, (uint8_t*) audioBuffer, AUDIO_FRAME_LEN);
	}
	else
	{
		// Stop audio update timer
		HW_TS_Stop(timer_id);

		// Call update task
		UTIL_SEQ_SetTask(1<<CFG_TASK_FS_AUDIO_UPDATE_ID, CFG_SCH_PRIO_0);
	}
}

static void FS_Audio_SetVolume(uint8_t volume)
{
	if (audioVolume != volume)
	{
		uint8_t buf[1];

		/* Set volume */
		buf[0] = volume;
		if (HAL_I2C_Mem_Write(&hi2c1, MAX9850_ADDR, MAX9850_REG_VOLUME, 1, buf, 1, HAL_MAX_DELAY) != HAL_OK)
		{
			Error_Handler();
		}

		audioVolume = volume;
	}
}

void FS_Audio_Init(void)
{
	uint32_t tickstart;

	uint8_t buf[1];

	// Reset state
	updateCount = 0;
	updateTotalTime = 0;
	updateMaxTime = 0;
	updateMaxInterval = 0;
	bufferUsed = 0;

	/* Initialize I2C1 */
	MX_I2C1_Init();

	/* Mute audio outputs */
	buf[0] = (0x3f << 0);
	if (HAL_I2C_Mem_Write(&hi2c1, MAX9850_ADDR, MAX9850_REG_VOLUME, 1, buf, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}

	audioVolume = 0x3f;

	/* Set stereo mode */
	buf[0] = 0;
	if (HAL_I2C_Mem_Write(&hi2c1, MAX9850_ADDR, MAX9850_REG_GENERAL, 1, buf, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}

	/* Set IC = 0x0 (SF = 1) */
	buf[0] = (0x0 << 2);
	if (HAL_I2C_Mem_Write(&hi2c1, MAX9850_ADDR, MAX9850_REG_CLOCK, 1, buf, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}

	/* Set CP = 0x09 */
	buf[0] = (0x09 << 0);
	if (HAL_I2C_Mem_Write(&hi2c1, MAX9850_ADDR, MAX9850_REG_CHARGE_PUMP, 1, buf, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}

	/* Set INT = 1 */
	buf[0] = (1 << 7);
	if (HAL_I2C_Mem_Write(&hi2c1, MAX9850_ADDR, MAX9850_REG_LRCLK_MSB, 1, buf, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}

	/* Set LSB = 32 */
	buf[0] = 32;
	if (HAL_I2C_Mem_Write(&hi2c1, MAX9850_ADDR, MAX9850_REG_LRCLK_LSB, 1, buf, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}

	/* Set slave mode, I2S, 16 bits */
	buf[0] = (0 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (1 << 3) | (0 << 2) | (0 << 0);
	if (HAL_I2C_Mem_Write(&hi2c1, MAX9850_ADDR, MAX9850_REG_DIGITAL_AUDIO, 1, buf, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}

	/* Enable MCLK, charge pump, headphone output and DAC */
	buf[0] = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 0);
	if (HAL_I2C_Mem_Write(&hi2c1, MAX9850_ADDR, MAX9850_REG_ENABLE, 1, buf, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}

	/* Enable PLLSAI1 */
	__HAL_RCC_PLLSAI1_ENABLE();

	/* Get start tick */
	tickstart = HAL_GetTick();

	/* Wait until PLLSAI1 is ready */
	while (LL_RCC_PLLSAI1_IsReady() != 1U)
	{
		if ((HAL_GetTick() - tickstart) > PLLSAI1_TIMEOUT_VALUE)
		{
			// TODO: Handle timeout
			Error_Handler();
		}
	}

	/* Initialize SAI1 */
	MX_SAI1_Init();

	/* Enable SAI to generate clock used by audio driver */
	__HAL_SAI_ENABLE(&hsai_BlockA1);

	// Initialize audio task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_AUDIO_UPDATE_ID, UTIL_SEQ_RFU, FS_Audio_Update);

	// Initialize audio update timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_Repeated, FS_Audio_Timer);
}

void FS_Audio_DeInit(void)
{
	uint8_t buf[1];

	// Stop audio output
	FS_Audio_Stop();

	/* Disable MCLK, charge pump, headphone output and DAC */
	buf[0] = 0;
	if (HAL_I2C_Mem_Write(&hi2c1, MAX9850_ADDR, MAX9850_REG_ENABLE, 1, buf, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}

	/* Disable I2C1 */
	HAL_I2C_DeInit(&hi2c1);

	/* Disable SAI1 */
	if (HAL_SAI_DeInit(&hsai_BlockA1) != HAL_OK)
	{
		Error_Handler();
	}

	// Delete audio update timer
	HW_TS_Delete(timer_id);

	// Add event log entries for timing info
	FS_Log_WriteEvent("----------");
	FS_Log_WriteEvent("%lu/%lu slots used in audio buffer", bufferUsed, AUDIO_FRAME_LEN);
	FS_Log_WriteEvent("%lu ms average time spent in audio update task", updateTotalTime / updateCount);
	FS_Log_WriteEvent("%lu ms maximum time spent in audio update task", updateMaxTime);
	FS_Log_WriteEvent("%lu ms maximum time between calls to audio update task", updateMaxInterval);
}

static void FS_Audio_LoadTone(void)
{
	static uint32_t phase = 0;

	uint32_t size, i;

	size = MIN(readPos + AUDIO_FRAME_LEN - writePos, audioLen);
	size = MIN(AUDIO_FRAME_LEN, size);

	for (i = 0; i < size; ++i, phase += audioStep, audioStep += audioChirp)
	{
		// Get sample value
		const uint32_t index = phase >> (32 - AUDIO_INDEX_BITS);
		const uint32_t a1 = (phase << AUDIO_INDEX_BITS) >> (32 - AUDIO_INTERPOLATE_BITS);
		const uint32_t a2 = (1 << AUDIO_INTERPOLATE_BITS) - a1;
		const int16_t val1 = sineTable[index];
		const int16_t val2 = sineTable[(index + 1) % (1 << AUDIO_INDEX_BITS)];
		const int16_t val = (val1 * a2 + val2 * a1) / (1 << AUDIO_INTERPOLATE_BITS);

		// Copy sample into audioBuffer
		audioBuffer[writePos % AUDIO_FRAME_LEN] = val;

		++writePos;
	}

	audioLen -= size;
}

static void FS_Audio_LoadFile(void)
{
	uint32_t size, s1, s2;
	UINT br;

	size = MIN(readPos + AUDIO_FRAME_LEN - writePos, audioLen);
	size = MIN(AUDIO_FRAME_LEN, size);

	s1 = MIN(AUDIO_FRAME_LEN - (writePos % AUDIO_FRAME_LEN), size);
	s2 = size - s1;

	// Read a block of data
	f_read(&audioFile, &audioBuffer[writePos % AUDIO_FRAME_LEN], s1 * sizeof(audioBuffer[0]), &br);
	f_read(&audioFile, &audioBuffer[0], s2 * sizeof(audioBuffer[0]), &br);

	writePos += size;
	audioLen -= size;
}

static void FS_Audio_Load(void)
{
	switch (audioState)
	{
	case AUDIO_IDLE:
		break;
	case AUDIO_PLAY_TONE:
		FS_Audio_LoadTone();
		break;
	case AUDIO_PLAY_FILE:
	case AUDIO_PLAY_LIST:
		FS_Audio_LoadFile();
		break;
	}
}

static void FS_Audio_InitTransfer(
		uint32_t bytes)
{
	// Initialize state
	frameCount = 0;
	lastFrame = (bytes + AUDIO_FRAME_LEN - 1) / AUDIO_FRAME_LEN;
	readPos = writePos = lastFrame * AUDIO_FRAME_LEN - bytes;

	// Initialize buffer
	FS_Audio_Load();

	// Begin DMA transfer
	HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*) (audioBuffer + readPos), AUDIO_FRAME_LEN - readPos);
}

void FS_Audio_Beep(
		uint32_t startFrequency,
		uint32_t endFrequency,
		uint32_t duration,
		uint8_t volume)
{
	if (audioState != AUDIO_IDLE)
	{
		FS_Audio_Stop();
	}

	audioState = AUDIO_PLAY_TONE;

	// Set volume
	FS_Audio_SetVolume(volume);

	// Remember frequency and duration
	audioStep = ((uint64_t) startFrequency << 32) / AUDIO_SAMPLE_RATE;
	audioLen = (duration * AUDIO_SAMPLE_RATE) / 1000;

	// Calculate chirp
	const uint32_t end_step = ((uint64_t) endFrequency << 32) / AUDIO_SAMPLE_RATE;
	if (end_step > audioStep) audioChirp = (end_step - audioStep) / audioLen;
	else                      audioChirp = 0 - (audioStep - end_step) / audioLen;

	// Initialize audio buffer
	FS_Audio_InitTransfer(audioLen);

	// Reset last update call time
	updateLastCall = 0;

	// Start audio update timer
	HW_TS_Start(timer_id, AUDIO_UPDATE_RATE);
}

static bool FS_Audio_PlayFile(
		const char *filename)
{
	f_chdir("/audio");

	if (f_open(&audioFile, filename, FA_READ) != FR_OK)
		return false;

	// TODO: Change this to parse WAV file properly
	f_lseek(&audioFile, 0x28);

	// Remember duration
	UINT br;
	uint32_t len;
	f_read(&audioFile, &len, 4, &br);
	audioLen = len / sizeof(audioBuffer[0]);

	// TODO: Change this to parse WAV file properly
	f_lseek(&audioFile, 0x2c);

	// Initialize audio buffer
	FS_Audio_InitTransfer(audioLen);

	// Reset last update call time
	updateLastCall = 0;

	// Start audio update timer
	HW_TS_Start(timer_id, AUDIO_UPDATE_RATE);

	return true;
}

void FS_Audio_Play(
		const char *filename,
		uint8_t volume)
{
	if (audioState != AUDIO_IDLE)
	{
		FS_Audio_Stop();
	}

	audioState = AUDIO_PLAY_FILE;

	// Set volume
	FS_Audio_SetVolume(volume);

	if (!FS_Audio_PlayFile(filename))
	{
		audioState = AUDIO_IDLE;
	}
}

static bool FS_Audio_LoadList(void)
{
	char filename[20];
	char ch;

	while ((ch = *(audioListPtr++)))
	{
		if ('0' <= ch && ch <= '9')
		{
			sprintf(filename, "%c.wav", ch);
			if (FS_Audio_PlayFile(filename))
				return true;
		}
		else if (ch == '-')
		{
			if (FS_Audio_PlayFile("minus.wav"))
				return true;
		}
		else if (ch == '.')
		{
			if (FS_Audio_PlayFile("dot.wav"))
				return true;
		}
	}
	return false;
}

void FS_Audio_PlayList(
		const char *list,
		uint8_t volume)
{
	if (audioState != AUDIO_IDLE)
	{
		FS_Audio_Stop();
	}

	audioState = AUDIO_PLAY_LIST;

	// Keep a copy of the list
	strncpy(audioList, list, sizeof(audioList));
	audioListPtr = audioList;

	// Set volume
	FS_Audio_SetVolume(volume);

	// Initialize audio buffer
	if (!FS_Audio_LoadList())
	{
		audioState = AUDIO_IDLE;
	}
}

static void FS_Audio_Idle(void)
{
	switch (audioState) {
	case AUDIO_IDLE:
		return;
	case AUDIO_PLAY_FILE:
	case AUDIO_PLAY_LIST:
		f_close(&audioFile);
		break;
	default:
		break;
	}

	audioState = AUDIO_IDLE;
}

bool FS_Audio_IsIdle(void)
{
	return audioState == AUDIO_IDLE;
}

void FS_Audio_Stop(void)
{
	uint32_t primask_bit;

	/* Enter critical section */
	primask_bit = __get_PRIMASK();
	__disable_irq();

	// Stop DMA transfer
	HAL_SAI_DMAStop(&hsai_BlockA1);

	// Stop audio update timer
	HW_TS_Stop(timer_id);

	/* Exit critical section */
	__set_PRIMASK(primask_bit);

	// Go to idle state
	FS_Audio_Idle();
}

static void FS_Audio_Timer(void)
{
	// Call update task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_AUDIO_UPDATE_ID, CFG_SCH_PRIO_0);
}

static void FS_Audio_Update(void)
{
	uint32_t msStart, msEnd;
	uint32_t primask_bit;
	uint32_t cndtr, count;

	if (audioState == AUDIO_IDLE) return;

	msStart = HAL_GetTick();

	if (updateLastCall != 0)
	{
		updateMaxInterval = MAX(updateMaxInterval, msStart - updateLastCall);
	}
	updateLastCall = msStart;

	/* Enter critical section */
	primask_bit = __get_PRIMASK();
	__disable_irq();

	count = frameCount;
	cndtr = hsai_BlockA1.hdmatx->Instance->CNDTR;

	/* Exit critical section */
	__set_PRIMASK(primask_bit);

	if (count == lastFrame)
	{
		uint32_t prevState = audioState;

		// Last frame complete
		FS_Audio_Idle();

		if (prevState == AUDIO_PLAY_LIST)
		{
			audioState = AUDIO_PLAY_LIST;

			// Initialize audio buffer
			if (!FS_Audio_LoadList())
			{
				audioState = AUDIO_IDLE;
			}
		}
	}
	else
	{
		// Get read position
		readPos = (count + 1) * AUDIO_FRAME_LEN - cndtr;

		// Update buffer statistics
		if (writePos != lastFrame * AUDIO_FRAME_LEN)
		{
			if (writePos > readPos)
			{
				bufferUsed = MAX(bufferUsed, readPos + AUDIO_FRAME_LEN - writePos);
			}
			else
			{
				bufferUsed = AUDIO_FRAME_LEN;
			}
		}

		// Update buffer
		FS_Audio_Load();
	}

	++updateCount;

	msEnd = HAL_GetTick();
	updateTotalTime += msEnd - msStart;
	updateMaxTime = MAX(updateMaxTime, msEnd - msStart);
}
