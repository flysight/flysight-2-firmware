/*
 * audio_control.h
 *
 *  Created on: Sep. 30, 2020
 *      Author: Michael
 */

#ifndef AUDIO_CONTROL_H_
#define AUDIO_CONTROL_H_

#include "gnss.h"

void FS_AudioControl_Init(void);
void FS_AudioControl_DeInit(void);
void FS_AudioControl_UpdateGNSS(const FS_GNSS_Data_t *current);

#endif /* AUDIO_CONTROL_H_ */
