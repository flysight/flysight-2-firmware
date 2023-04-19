/*
 * config_mode.h
 *
 *  Created on: Apr 17, 2023
 *      Author: Michael
 */

#ifndef CONFIG_MODE_H_
#define CONFIG_MODE_H_

void FS_ConfigMode_Init(void);
void FS_ConfigMode_DeInit(void);
const char *FS_ConfigMode_GetConfigFilename(void);

#endif /* CONFIG_MODE_H_ */
