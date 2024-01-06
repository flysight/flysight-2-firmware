/*
 * hum.c
 *
 *  Created on: Jun 10, 2020
 *      Author: Michael
 */

#include "main.h"
#include "app_common.h"
#include "hts221.h"
#include "hum.h"

typedef struct
{
	void (*Start)(void);
	void (*Stop)(void);
	void (*Read)(FS_Hum_Data_t *data);
} FS_Hum_Interface_t;

static FS_Hum_Interface_t humInterface;
static FS_Hum_Data_t humData;

void FS_Hum_Init(void)
{
	if (FS_HTS221_Init() == FS_HUM_OK)
	{
		humInterface.Start = &FS_HTS221_Start;
		humInterface.Stop = &FS_HTS221_Stop;
		humInterface.Read = &FS_HTS221_Read;
	}
	else
	{
		Error_Handler();	// Should never be called
	}
}

void FS_Hum_Start(void)
{
	(*humInterface.Start)();
}

void FS_Hum_Stop(void)
{
	(*humInterface.Stop)();
}

void FS_Hum_Read(void)
{
	(*humInterface.Read)(&humData);
}

const FS_Hum_Data_t *FS_Hum_GetData(void)
{
	return &humData;
}

__weak void FS_Hum_DataReady_Callback(void)
{
  /* NOTE: This function should not be modified, when the callback is needed,
           the FS_Hum_DataReady_Callback could be implemented in the user file
   */
}
