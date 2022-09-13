/*
 * vbat.h
 *
 *  Created on: Sep. 13, 2022
 *      Author: Michael
 */

#ifndef VBAT_H_
#define VBAT_H_

typedef struct
{
	uint32_t time;		// ms
	uint16_t voltage;   // Battery voltage
} FS_VBAT_Data_t;

void FS_VBAT_Init(void);
void FS_VBAT_DeInit(void);

void FS_VBAT_ConversionComplete(void);

const FS_VBAT_Data_t *FS_VBAT_GetData(void);
void FS_VBAT_ValueReady_Callback(void);

#endif /* VBAT_H_ */
