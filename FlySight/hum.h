/*
 * hum.h
 *
 *  Created on: Jun 10, 2020
 *      Author: Michael
 */

#ifndef HUM_H_
#define HUM_H_

typedef enum
{
  FS_HUM_OK       = 0x00,
  FS_HUM_ERROR    = 0x01
} FS_Hum_Result_t;

typedef struct
{
	uint32_t time;			// ms
	uint16_t humidity;		// rH % * 10
	uint16_t temperature;	// degrees C * 10
} FS_Hum_Data_t;

void FS_Hum_Init(void);
void FS_Hum_Start(void);
void FS_Hum_Stop(void);
void FS_Hum_Read(void);
const FS_Hum_Data_t *FS_Hum_GetData(void);
void FS_Hum_DataReady_Callback(void);

#endif /* HUM_H_ */
