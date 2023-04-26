/*
 * state.h
 *
 *  Created on: Apr. 19, 2023
 *      Author: Michael
 */

#ifndef STATE_H_
#define STATE_H_

typedef struct
{
	uint32_t device_id[3];
	uint32_t session_id[3];
	char     config_filename[13];
	uint32_t temp_folder;
} FS_State_Data_t;

void FS_State_Init(void);
const FS_State_Data_t *FS_State_Get(void);
void FS_State_SetConfigFilename(const char *filename);

#endif /* STATE_H_ */
