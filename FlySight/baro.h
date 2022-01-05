/*
 * baro.h
 *
 *  Created on: Jun 10, 2020
 *      Author: Michael
 */

#ifndef BARO_H_
#define BARO_H_

typedef struct
{
	uint32_t time;			// ms
	int32_t pressure;		// Pa * 100
	int16_t temperature;	// degrees C * 100
} FS_Baro_Data_t;

void FS_Baro_Init(void);
void FS_Baro_Start(void);
void FS_Baro_Stop(void);
void FS_Baro_Read(void);
const FS_Baro_Data_t *FS_Baro_GetData(void);
void FS_Baro_DataReady_Callback(void);

#endif /* BARO_H_ */
