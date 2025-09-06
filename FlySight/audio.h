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

#ifndef AUDIO_H_
#define AUDIO_H_

#include <stdbool.h>

#define AUDIO_LIST_LEN 10

HAL_StatusTypeDef FS_Audio_Init(void);
void FS_Audio_DeInit(void);

void FS_Audio_Beep(uint32_t startFrequency, uint32_t endFrequency, uint32_t duration, uint8_t volume);
void FS_Audio_Play(const char *filename, uint8_t volume);
void FS_Audio_PlayList(const char *list, uint8_t volume);
void FS_Audio_Stop(void);
bool FS_Audio_IsIdle(void);

#endif /* AUDIO_H_ */
