/*
 * mag.h
 *
 *  Created on: Jun 10, 2020
 *      Author: Michael
 */

#ifndef MAG_H_
#define MAG_H_

typedef struct
{
	uint32_t time;			// ms
	int16_t x;				// gauss * 1000
	int16_t y;				// gauss * 1000
	int16_t z;				// gauss * 1000
	int16_t temperature;	// degrees C * 10
} FS_Mag_Data_t;

void FS_Mag_Init(void);
void FS_Mag_Start(void);
void FS_Mag_Stop(void);
void FS_Mag_Read(void);
const FS_Mag_Data_t *FS_Mag_GetData(void);
void FS_Mag_DataReady_Callback(void);

#endif /* MAG_H_ */
