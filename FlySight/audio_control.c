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

#include <math.h>
#include <stdbool.h>

#include "main.h"
#include "app_common.h"
#include "audio.h"
#include "audio_control.h"
#include "common.h"
#include "config.h"
#include "stm32_seq.h"

#define CONSUMER_TIMER_MSEC    10
#define CONSUMER_TIMER_TICKS   (CONSUMER_TIMER_MSEC*1000/CFG_TS_TICK_VAL)

#define ABS(a) (((a) < 0) ? -(a) : (a))

#define INVALID_VALUE   INT32_MAX

#define ALT_MIN         1500L // Minimum announced altitude (m)

#define FLAG_HAS_FIX         0x01
#define FLAG_FIRST_FIX       0x02
#define FLAG_BEEP_DONE       0x04
#define FLAG_SAY_ALTITUDE    0x08
#define FLAG_VERTICAL_ACC    0x10

#define TONE_MIN_PITCH 220
#define TONE_MAX_PITCH 1760

static const uint16_t sas_table[] =
{
	1024, 1077, 1135, 1197,
	1265, 1338, 1418, 1505,
	1600, 1704, 1818, 1944
};

static uint8_t timer_id;

static uint8_t cur_speech;

static uint16_t sp_counter;

static uint8_t flags;
static uint8_t prev_flags;

static int32_t prevHMSL;

static uint8_t g_suppress_tone;

static char speech_buf[16];
static char *speech_ptr;

static volatile uint32_t tonePitch;
static volatile int32_t  toneChirp;
static volatile uint16_t toneRate;
static volatile uint8_t  toneHold;

static void setRate(uint16_t rate)
{
	toneRate = rate;
}

static void setPitch(uint16_t pitch)
{
	tonePitch = pitch;
}

static void setChirp(uint32_t chirp)
{
	toneChirp = chirp;
}

static void setTone(
	FS_Config_Data_t *config,
	int32_t val_1,
	int32_t min_1,
	int32_t max_1,
	int32_t val_2,
	int32_t min_2,
	int32_t max_2)
{
	#define UNDER(val,min,max) ((min < max) ? (val <= min) : (val >= min))
	#define OVER(val,min,max)  ((min < max) ? (val >= max) : (val <= max))

	if (val_1 != INVALID_VALUE &&
	    val_2 != INVALID_VALUE)
	{
		if (UNDER(val_2, min_2, max_2))
		{
			if (config->flatline)
			{
				setRate(FS_CONFIG_RATE_FLATLINE);
			}
			else
			{
				setRate(config->min_rate);
			}
		}
		else if (OVER(val_2, min_2, max_2))
		{
			setRate(config->max_rate - 1);
		}
		else
		{
			setRate(config->min_rate + (config->max_rate - config->min_rate) * (val_2 - min_2) / (max_2 - min_2));
		}

		if (UNDER(val_1, min_1, max_1))
		{
			if (config->limits == 0)
			{
				setRate(0);
			}
			else if (config->limits == 1)
			{
				setPitch(TONE_MIN_PITCH);
				setChirp(0);
			}
			else if (config->limits == 2)
			{
				setPitch(TONE_MIN_PITCH);
				setChirp(TONE_MAX_PITCH - TONE_MIN_PITCH);
			}
			else
			{
				setPitch(TONE_MAX_PITCH);
				setChirp(TONE_MIN_PITCH - TONE_MAX_PITCH);
			}
		}
		else if (OVER(val_1, min_1, max_1))
		{
			if (config->limits == 0)
			{
				setRate(0);
			}
			else if (config->limits == 1)
			{
				setPitch(TONE_MAX_PITCH);
				setChirp(0);
			}
			else if (config->limits == 2)
			{
				setPitch(TONE_MAX_PITCH);
				setChirp(TONE_MIN_PITCH - TONE_MAX_PITCH);
			}
			else
			{
				setPitch(TONE_MIN_PITCH);
				setChirp(TONE_MAX_PITCH - TONE_MIN_PITCH);
			}
		}
		else
		{
			setPitch(TONE_MIN_PITCH + (TONE_MAX_PITCH - TONE_MIN_PITCH) * (val_1 - min_1) / (max_1 - min_1));
			setChirp(0);
		}
	}
	else
	{
		setRate(0);
	}

	#undef OVER
	#undef UNDER
}

static void getValues(
	FS_GNSS_Data_t *current,
	FS_Config_Data_t *config,
	uint8_t mode,
	int32_t *val,
	int32_t *min,
	int32_t *max)
{
	uint16_t speed_mul = 1024;

	if (config->use_sas)
	{
		if (current->hMSL < 0)
		{
			speed_mul = sas_table[0];
		}
		else if (current->hMSL >= 11534336L)
		{
			speed_mul = sas_table[11];
		}
		else
		{
			int32_t h = current->hMSL / 1024;
			uint16_t i = h / 1024;
			uint16_t j = h % 1024;
			uint16_t y1 = sas_table[i];
			uint16_t y2 = sas_table[i + 1];
			speed_mul = y1 + ((y2 - y1) * j) / 1024;
		}
	}

	switch (mode)
	{
	case 0: // Horizontal speed
		*val = (current->gSpeed * 1024) / speed_mul;
		break;
	case 1: // Vertical speed
		*val = (current->velD * 1024) / speed_mul;
		break;
	case 2: // Glide ratio
		if (current->velD != 0)
		{
			*val = 10000 * (int32_t) current->gSpeed / current->velD;
			*min *= 100;
			*max *= 100;
		}
		break;
	case 3: // Inverse glide ratio
		if (current->gSpeed != 0)
		{
			*val = 10000 * current->velD / (int32_t) current->gSpeed;
			*min *= 100;
			*max *= 100;
		}
		break;
	case 4: // Total speed
		*val = (current->speed * 1024) / speed_mul;
		break;
	case 11: // Dive angle
		*val = atan2(current->velD, current->gSpeed) / M_PI * 180;
		break;
	}
}

static char *numberToSpeech(
	int32_t number,
	char *ptr)
{
	// Adapted from https://stackoverflow.com/questions/2729752/converting-numbers-in-to-words-c-sharp

    if (number == 0)
	{
		*(ptr++) = '0';
		return ptr;
	}

    if (number < 0)
	{
		*(ptr++) = '-';
        return numberToSpeech(-number, ptr);
	}

    if ((number / 1000) > 0)
    {
        ptr = numberToSpeech(number / 1000, ptr);
		*(ptr++) = 'k';
        number %= 1000;
    }

    if ((number / 100) > 0)
    {
        ptr = numberToSpeech(number / 100, ptr);
		*(ptr++) = 'h';
        number %= 100;
    }

    if (number > 0)
    {
		if (number < 10)
		{
			*(ptr++) = '0' + number;
		}
		else if (number < 20)
		{
			*(ptr++) = 't';
			*(ptr++) = '0' + (number - 10);
		}
        else
        {
			*(ptr++) = 'x';
			*(ptr++) = '0' + (number / 10);

            if ((number % 10) > 0)
				*(ptr++) = '0' + (number % 10);
        }
    }

    return ptr;
}

static void speakValue(
	FS_Config_Data_t *config,
	FS_GNSS_Data_t *current)
{
	uint16_t speed_mul = 1024;
	int32_t step_size, step;

	char *end_ptr;

	if (config->use_sas)
	{
		if (current->hMSL < 0)
		{
			speed_mul = sas_table[0];
		}
		else if (current->hMSL >= 11534336L)
		{
			speed_mul = sas_table[11];
		}
		else
		{
			int32_t h = current->hMSL / 1024;
			uint16_t i = h / 1024;
			uint16_t j = h % 1024;
			uint16_t y1 = sas_table[i];
			uint16_t y2 = sas_table[i + 1];
			speed_mul = y1 + ((y2 - y1) * j) / 1024;
		}
	}

	switch (config->speech[cur_speech].units)
	{
	case FS_CONFIG_UNITS_KMH:
		speed_mul = (uint16_t) (((uint32_t) speed_mul * 18204) / 65536);
		break;
	case FS_CONFIG_UNITS_MPH:
		speed_mul = (uint16_t) (((uint32_t) speed_mul * 29297) / 65536);
		break;
	}

	// Step 0: Initialize speech pointers, leaving room at the end for one unit character

	speech_ptr = speech_buf + sizeof(speech_buf) - 1;
	end_ptr = speech_ptr;

	// Step 1: Get speech value with 2 decimal places

	switch (config->speech[cur_speech].mode)
	{
	case 0: // Horizontal speed
		speech_ptr = writeInt32ToBuf(speech_ptr, (current->gSpeed * 1024) / speed_mul, 2, 1, 0);
		break;
	case 1: // Vertical speed
		speech_ptr = writeInt32ToBuf(speech_ptr, (current->velD * 1024) / speed_mul, 2, 1, 0);
		break;
	case 2: // Glide ratio
		if (current->velD != 0)
		{
			speech_ptr = writeInt32ToBuf(speech_ptr, 100 * (int32_t) current->gSpeed / current->velD, 2, 1, 0);
		}
		else
		{
			*(--speech_ptr) = '\0';
		}
		break;
	case 3: // Inverse glide ratio
		if (current->gSpeed != 0)
		{
			speech_ptr = writeInt32ToBuf(speech_ptr, 100 * (int32_t) current->velD / current->gSpeed, 2, 1, 0);
		}
		else
		{
			*(--speech_ptr) = '\0';
		}
		break;
	case 4: // Total speed
		speech_ptr = writeInt32ToBuf(speech_ptr, (current->speed * 1024) / speed_mul, 2, 1, 0);
		break;
	case 11: // Dive angle
		speech_ptr = writeInt32ToBuf(speech_ptr, 100 * atan2(current->velD, current->gSpeed) / M_PI * 180, 2, 1, 0);
		break;
	case 12: // Altitude
		if (config->speech[cur_speech].units == FS_CONFIG_UNITS_KMH)
		{
			step_size = 10000 * config->speech[cur_speech].decimals;
		}
		else
		{
			step_size = 3048 * config->speech[cur_speech].decimals;
		}
		step = ((current->hMSL - config->dz_elev) * 10 + step_size / 2) / step_size;
		speech_ptr = speech_buf + 2;
		speech_ptr = numberToSpeech(step * config->speech[cur_speech].decimals, speech_ptr);
		end_ptr = speech_ptr;
		speech_ptr = speech_buf + 2;
		break;
	}

	// Step 1.5: Include label
	if (config->num_speech > 1)
	{
		*(--speech_ptr) = config->speech[cur_speech].mode + 1;
		*(--speech_ptr) = '>';
	}

	// Step 2: Truncate to the desired number of decimal places

	if (config->speech[cur_speech].mode != 5)
	{
		if (config->speech[cur_speech].decimals == 0) end_ptr -= 4;
		else end_ptr -= 3 - config->speech[cur_speech].decimals;
	}

	// Step 3: Add units if needed, e.g., *(end_ptr++) = 'k';

	switch (config->speech[cur_speech].mode)
	{
	case 0: // Horizontal speed
	case 1: // Vertical speed
	case 2: // Glide ratio
	case 3: // Inverse glide ratio
	case 4: // Total speed
	case 11: // Dive angle
		break;
	case 12: // Altitude
		*(end_ptr++) = (config->speech[cur_speech].units == FS_CONFIG_UNITS_KMH) ? 'm' : 'f';
		break;
	}

	// Step 4: Terminate with a null

	*(end_ptr++) = '\0';
}

static void updateAlarms(
	FS_Config_Data_t *config,
	FS_GNSS_Data_t *current)
{
	uint8_t i, suppress_tone, suppress_alt;
	int32_t step_size, step, step_elev;

	char filename[13];

	suppress_tone = 0;
	suppress_alt = 0;

	for (i = 0; i < config->num_alarms; ++i)
	{
		const int32_t alarm_elev = config->alarms[i].elev + config->dz_elev;

		if ((current->hMSL <= alarm_elev + config->alarm_window_above) &&
		    (current->hMSL >= alarm_elev - config->alarm_window_below))
		{
			suppress_tone = 1;
			break;
		}
	}

	for (i = 0; i < config->num_windows; ++i)
	{
		if ((config->windows[i].bottom + config->dz_elev <= current->hMSL) &&
		    (config->windows[i].top + config->dz_elev >= current->hMSL))
		{
			suppress_tone = 1;
			suppress_alt = 1;
			break;
		}
	}

	if (config->alt_step > 0)
	{
		if (config->alt_units == FS_CONFIG_UNITS_METERS)
		{
			step_size = 10000 * config->alt_step;
		}
		else
		{
			step_size = 3048 * config->alt_step;
		}

		step = ((current->hMSL - config->dz_elev) * 10 + step_size / 2) / step_size;
		step_elev = step * step_size / 10 + config->dz_elev;

		if ((current->hMSL <= step_elev + config->alarm_window_above) &&
		    (current->hMSL >= step_elev - config->alarm_window_below) &&
		    (current->hMSL - config->dz_elev >= ALT_MIN * 1000))
		{
			suppress_tone = 1;
		}
	}

	if (suppress_tone && !g_suppress_tone)
	{
		*speech_ptr = '\0';
		setRate(0);
		FS_Audio_Stop();
	}

	g_suppress_tone = suppress_tone;

	if (prev_flags & FLAG_HAS_FIX)
	{
		int32_t min = MIN(prevHMSL, current->hMSL);
		int32_t max = MAX(prevHMSL, current->hMSL);

		for (i = 0; i < config->num_alarms; ++i)
		{
			const int32_t alarm_elev = config->alarms[i].elev + config->dz_elev;

			if (alarm_elev >= min && alarm_elev < max)
			{
				switch (config->alarms[i].type)
				{
				case 1:	// beep
					FS_Audio_Beep(TONE_MAX_PITCH, TONE_MAX_PITCH, 125, config->volume * 5);
					break ;
				case 2:	// chirp up
					FS_Audio_Beep(TONE_MIN_PITCH, TONE_MAX_PITCH, 125, config->volume * 5);
					break ;
				case 3:	// chirp down
					FS_Audio_Beep(TONE_MAX_PITCH, TONE_MIN_PITCH, 125, config->volume * 5);
					break ;
				case 4:	// play file
					filename[0] = '\0';
					strncat(filename, config->alarms[i].filename, sizeof(filename) - 1);
					strncat(filename, ".wav", sizeof(filename) - 1);
					FS_Audio_Play(filename, config->sp_volume * 5);
					break;
				}

				*speech_ptr = '\0';
				break;
			}
		}

		if ((config->alt_step > 0) &&
		    (i == config->num_alarms) &&
		    (prevHMSL - config->dz_elev >= ALT_MIN * 1000) &&
		    (*speech_ptr == 0) &&
		    !(flags & FLAG_SAY_ALTITUDE) &&
		    !suppress_alt)
		{
			if ((step_elev >= min && step_elev < max) &&
			    ABS(current->velD) >= config->threshold &&
			    current->gSpeed >= config->hThreshold)
			{
				speech_ptr = speech_buf;
				speech_ptr = numberToSpeech(step * config->alt_step, speech_ptr);
				*(speech_ptr++) = (config->alt_units == FS_CONFIG_UNITS_METERS) ? 'm' : 'f';
				*(speech_ptr++) = '\0';
				speech_ptr = speech_buf;
			}
		}
	}
}

static void updateTones(
	FS_Config_Data_t *config,
	FS_GNSS_Data_t *current)
{
	static int32_t x0 = INVALID_VALUE, x1, x2;

	int32_t val_1 = INVALID_VALUE, min_1 = config->min, max_1 = config->max;
	int32_t val_2 = INVALID_VALUE, min_2 = config->min_2, max_2 = config->max_2;

	uint8_t i;

	getValues(current, config, config->mode, &val_1, &min_1, &max_1);

	if (config->mode_2 == 8)
	{
		getValues(current, config, config->mode, &val_2, &min_2, &max_2);
		if (val_2 != INVALID_VALUE)
		{
			val_2 = ABS(val_2);
		}
	}
	else if (config->mode_2 == 9)
	{
		x2 = x1;
		x1 = x0;
		x0 = val_1;

		if (x0 != INVALID_VALUE &&
			x1 != INVALID_VALUE &&
			x2 != INVALID_VALUE &&
			max_1 != min_1)
		{
			val_2 = (int32_t) 1000 * (x2 - x0) / (int32_t) (2 * config->rate);
			val_2 = (int32_t) 10000 * ABS(val_2) / ABS(max_1 - min_1);
		}
	}
	else
	{
		getValues(current, config, config->mode_2, &val_2, &min_2, &max_2);
	}

	if (!g_suppress_tone)
	{
		if (ABS(current->velD) >= config->threshold &&
			current->gSpeed >= config->hThreshold)
		{
			setTone(config, val_1, min_1, max_1, val_2, min_2, max_2);

			if (config->sp_rate != 0 &&
			    config->num_speech != 0 &&
			    sp_counter >= config->sp_rate &&
				(*speech_ptr == 0) &&
				!(flags & FLAG_SAY_ALTITUDE))
			{
				for (i = 0; i < config->num_speech; ++i)
				{
					if ((config->speech[cur_speech].mode != 5) ||
						(current->hMSL - config->dz_elev >= ALT_MIN * 1000))
					{
						speakValue(config, current);
						cur_speech = (cur_speech + 1) % config->num_speech;
						break;
					}
					else
					{
						cur_speech = (cur_speech + 1) % config->num_speech;
					}
				}

				sp_counter = 0;
			}
		}
		else
		{
			setRate(0);
		}
	}

	if (sp_counter < config->sp_rate)
	{
		sp_counter += config->rate;
	}
}

static void producerTask(void)
{
	FS_Config_Data_t config;
	FS_GNSS_Data_t current;

	// Copy to local variable
	memcpy(&config, FS_Config_Get(), sizeof(FS_Config_Data_t));
	memcpy(&current, FS_GNSS_GetData(), sizeof(FS_GNSS_Data_t));

	if (current.gpsFix == 3)
	{
		flags |= FLAG_HAS_FIX;

		updateAlarms(&config, &current);
		updateTones(&config, &current);

		if (!(flags & FLAG_BEEP_DONE))
		{
			flags |= FLAG_FIRST_FIX;
		}
	}
	else
	{
		flags &= ~FLAG_HAS_FIX;
		setRate(0);
	}

	if (current.vAcc < 10000)
	{
		flags |= FLAG_VERTICAL_ACC;
	}
	else
	{
		flags &= ~FLAG_VERTICAL_ACC;
	}

	prev_flags = flags;
	prevHMSL = current.hMSL;
}

static void consumerTimer(void)
{
	static uint16_t tone_timer = 0;
	const FS_Config_Data_t *config = FS_Config_Get();

	if (FS_Audio_IsIdle() && !toneHold && toneRate > 0 && 0x10000 - tone_timer <= toneRate)
	{
		FS_Audio_Beep(tonePitch, tonePitch + toneChirp, 125, config->volume * 5);
	}

	tone_timer += toneRate;

	// Call consumer task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_AUDIO_CONTROL_CONSUMER_ID, CFG_SCH_PRIO_0);
}

static void consumerTask(void)
{
	const FS_Config_Data_t *config = FS_Config_Get();

	if (*speech_ptr)
	{
		if (FS_Audio_IsIdle())
		{
			char filename[13];

			toneHold = 1;

			if (*speech_ptr == '-')
			{
				FS_Audio_Play("minus.wav", config->sp_volume * 5);
			}
			else if (*speech_ptr == '.')
			{
				FS_Audio_Play("dot.wav", config->sp_volume * 5);
			}
			else if (*speech_ptr == 'h')
			{
				FS_Audio_Play("00.wav", config->sp_volume * 5);
			}
			else if (*speech_ptr == 'k')
			{
				FS_Audio_Play("000.wav", config->sp_volume * 5);
			}
			else if (*speech_ptr == 'm')
			{
				FS_Audio_Play("meters.wav", config->sp_volume * 5);
			}
			else if (*speech_ptr == 'f')
			{
				FS_Audio_Play("feet.wav", config->sp_volume * 5);
			}
			else if (*speech_ptr == 't')
			{
				++speech_ptr;
				filename[1] = *speech_ptr;
				filename[0] = '1';
				filename[2] = '.';
				filename[3] = 'w';
				filename[4] = 'a';
				filename[5] = 'v';
				filename[6] = '\0';

				FS_Audio_Play(filename, config->sp_volume * 5);
			}
			else if (*speech_ptr == 'x')
			{
				++speech_ptr;
				filename[0] = *speech_ptr;
				filename[1] = '0';
				filename[2] = '.';
				filename[3] = 'w';
				filename[4] = 'a';
				filename[5] = 'v';
				filename[6] = '\0';

				FS_Audio_Play(filename, config->sp_volume * 5);
			}
			else if (*speech_ptr == '>')
			{
				++speech_ptr;
				switch ((*speech_ptr) - 1)
				{
					case 0:
						FS_Audio_Play("horz.wav", config->sp_volume * 5);
						break;
					case 1:
						FS_Audio_Play("vert.wav", config->sp_volume * 5);
						break;
					case 2:
						FS_Audio_Play("glide.wav", config->sp_volume * 5);
						break;
					case 3:
						FS_Audio_Play("iglide.wav", config->sp_volume * 5);
						break;
					case 4:
						FS_Audio_Play("speed.wav", config->sp_volume * 5);
						break;
					case 11:
						FS_Audio_Play("dive.wav", config->sp_volume * 5);
						break;
					case 12:
						FS_Audio_Play("alt.wav", config->sp_volume * 5);
						break;
				}
			}
			else
			{
				filename[0] = *speech_ptr;
				filename[1] = '.';
				filename[2] = 'w';
				filename[3] = 'a';
				filename[4] = 'v';
				filename[5] = '\0';

				FS_Audio_Play(filename, config->sp_volume * 5);
			}

			++speech_ptr;
		}
	}
	else
	{
		const FS_Config_Data_t *config = FS_Config_Get();

		toneHold = 0;

		if ((flags & FLAG_FIRST_FIX) && FS_Audio_IsIdle())
		{
			flags &= ~FLAG_FIRST_FIX;
			FS_Audio_Beep(TONE_MAX_PITCH, TONE_MAX_PITCH, 125, config->volume * 5);
			flags |= FLAG_BEEP_DONE;
		}

		if ((flags & FLAG_SAY_ALTITUDE) &&
			(flags & FLAG_HAS_FIX) &&
		    (flags & FLAG_VERTICAL_ACC) &&
			FS_Audio_IsIdle())
		{
			flags &= ~FLAG_SAY_ALTITUDE;
			speech_ptr = speech_buf;

			if (config->alt_units == FS_CONFIG_UNITS_METERS)
			{
				speech_ptr = numberToSpeech((prevHMSL - config->dz_elev) / 1000, speech_ptr);
				*(speech_ptr++) = 'm';
			}
			else
			{
				speech_ptr = numberToSpeech((prevHMSL - config->dz_elev) * 10 / 3048, speech_ptr);
				*(speech_ptr++) = 'f';
			}

			*(speech_ptr++) = '\0';
			speech_ptr = speech_buf;
		}
	}
}

void FS_AudioControl_Init(void)
{
	const FS_Config_Data_t *config = FS_Config_Get();
	uint8_t i;
	char filename[13];

	// Initialize state
	cur_speech = 0;
	sp_counter = 0;
	flags = 0;
	prev_flags = 0;
	g_suppress_tone = 0;
	speech_buf[0] = '\0';
	speech_ptr = speech_buf;
	tonePitch = 0;
	toneChirp = 0;
	toneRate = 0;
	toneHold = 0;

	// Initialize producer task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_AUDIO_CONTROL_PRODUCER_ID, UTIL_SEQ_RFU, producerTask);

	// Initialize consumer task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_AUDIO_CONTROL_CONSUMER_ID, UTIL_SEQ_RFU, consumerTask);

	// Initialize consumer timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_Repeated, consumerTimer);
	HW_TS_Start(timer_id, CONSUMER_TIMER_TICKS);

	if (config->alt_step > 0)
	{
		flags |= FLAG_SAY_ALTITUDE;
	}

	for (i = 0; i < config->num_speech; ++i)
	{
		if (config->speech[i].mode == 5)
		{
			flags |= FLAG_SAY_ALTITUDE;
		}
	}

	if (config->init_mode == 1)
	{
		strncpy(speech_buf, "0123456789.-", sizeof(speech_buf));
	}
	else if (config->init_mode == 2)
	{
		filename[0] = '\0';
		strncat(filename, config->init_filename, sizeof(filename) - 1);
		strncat(filename, ".wav", sizeof(filename) - 1);
		FS_Audio_Play(filename, config->sp_volume * 5);
	}
}

void FS_AudioControl_DeInit(void)
{
	// Delete update timer
	HW_TS_Delete(timer_id);
}

void FS_AudioControl_UpdateGNSS(const FS_GNSS_Data_t *current)
{
	// Call update task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_AUDIO_CONTROL_PRODUCER_ID, CFG_SCH_PRIO_0);
}
