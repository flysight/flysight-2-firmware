/*
 * audio.h
 *
 *  Created on: May 17, 2020
 *      Author: Michael
 */

#ifndef AUDIO_H_
#define AUDIO_H_

#include <stdbool.h>

#define AUDIO_LIST_LEN 10

void FS_Audio_Init(void);
void FS_Audio_DeInit(void);

void FS_Audio_Beep(uint32_t startFrequency, uint32_t endFrequency, uint32_t duration, uint8_t volume);
void FS_Audio_Play(const char *filename, uint8_t volume);
void FS_Audio_PlayList(const char *list, uint8_t volume);
void FS_Audio_Stop(void);
bool FS_Audio_IsIdle(void);

#endif /* AUDIO_H_ */
