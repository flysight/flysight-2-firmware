/*
 * sensor.c
 *
 *  Created on: Aug. 1, 2020
 *      Author: Michael
 */

#include <stdbool.h>

#include "main.h"
#include "app_common.h"
#include "stm32_seq.h"

#define HANDLER_COUNT 5

typedef enum
{
	Operation_Read,
	Operation_Write
} Operation_t;

typedef struct
{
	Operation_t op;
	uint8_t addr;
	uint16_t reg;
	uint8_t *pData;
	uint16_t size;
	void (*Callback)(HAL_StatusTypeDef);
} Handler_t;

static          Handler_t handlerBuf[HANDLER_COUNT];	// handler buffer
static volatile uint32_t  handlerRdI = 0;				// read index
static volatile uint32_t  handlerWrI = 0;				// write index

static volatile bool      handleRead = false;
static volatile bool      sensorBusy = false;

extern I2C_HandleTypeDef hi2c3;

static void BeginRead(void);
static void NextHandler(HAL_StatusTypeDef result);

void FS_Sensor_TransferComplete(void)
{
	NextHandler(HAL_OK);
}

void FS_Sensor_TransferError(void)
{
	NextHandler(HAL_ERROR);
}

void FS_Sensor_Start(void)
{
	uint32_t primask_bit;

	// Enable asyncrhonous reads
	primask_bit = __get_PRIMASK();
	__disable_irq();

	handleRead = true;
	bool begin = (handlerRdI != handlerWrI);

	__set_PRIMASK(primask_bit);

	if (begin)
	{
		BeginRead();
	}
}

void FS_Sensor_Stop(void)
{
	// Disable asynchronous reads
	handleRead = false;

	while (sensorBusy);
}

static void BeginRead(void)
{
	Handler_t *h = &handlerBuf[handlerRdI % HANDLER_COUNT];
	HAL_StatusTypeDef result;

	sensorBusy = true;
	if (h->op == Operation_Read)
	{
		result = HAL_I2C_Mem_Read_DMA(&hi2c3, h->addr, h->reg, 1, h->pData, h->size);
	}
	else
	{
		result = HAL_I2C_Mem_Write_DMA(&hi2c3, h->addr, h->reg, 1, h->pData, h->size);
	}

	if (result != HAL_OK)
	{
		sensorBusy = false;
		if (h->Callback)
			h->Callback(result);
	}
}

static void NextHandler(HAL_StatusTypeDef result)
{
	Handler_t *h = &handlerBuf[handlerRdI % HANDLER_COUNT];
	uint32_t primask_bit;

	// Handler callback
	sensorBusy = false;
	if (h->Callback)
		h->Callback(result);

	// Begin next transfer
	primask_bit = __get_PRIMASK();
	__disable_irq();

	++handlerRdI;
	bool begin = (handlerRdI != handlerWrI);

	__set_PRIMASK(primask_bit);

	if (handleRead && begin)
	{
		BeginRead();
	}
}

void FS_Sensor_WriteAsync(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size, void (*Callback)(HAL_StatusTypeDef))
{
	Handler_t *h = &handlerBuf[handlerWrI % HANDLER_COUNT];
	uint32_t primask_bit;

	// Initialize handler
	h->op = Operation_Write;
	h->addr = addr;
	h->reg = reg;
	h->pData = pData;
	h->size = size;
	h->Callback = Callback;

	// Begin transfer
	primask_bit = __get_PRIMASK();
	__disable_irq();

	bool begin = (handlerRdI == handlerWrI);
	++handlerWrI;

	__set_PRIMASK(primask_bit);

	if (handleRead && begin)
	{
		BeginRead();
	}
}

void FS_Sensor_ReadAsync(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size, void (*Callback)(HAL_StatusTypeDef))
{
	Handler_t *h = &handlerBuf[handlerWrI % HANDLER_COUNT];
	uint32_t primask_bit;

	// Initialize handler
	h->op = Operation_Read;
	h->addr = addr;
	h->reg = reg;
	h->pData = pData;
	h->size = size;
	h->Callback = Callback;

	// Begin transfer
	primask_bit = __get_PRIMASK();
	__disable_irq();

	bool begin = (handlerRdI == handlerWrI);
	++handlerWrI;

	__set_PRIMASK(primask_bit);

	if (handleRead && begin)
	{
		BeginRead();
	}
}

HAL_StatusTypeDef FS_Sensor_Write(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size)
{
	HAL_StatusTypeDef ret;
	bool active = handleRead;

	if (active) FS_Sensor_Stop();
	ret = HAL_I2C_Mem_Write(&hi2c3, addr, reg, 1, pData, size, HAL_MAX_DELAY);
	if (active) FS_Sensor_Start();

	return ret;
}

HAL_StatusTypeDef FS_Sensor_Read(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size)
{
	HAL_StatusTypeDef ret;
	bool active = handleRead;

	if (active) FS_Sensor_Stop();
	ret = HAL_I2C_Mem_Read(&hi2c3, addr, reg, 1, pData, size, HAL_MAX_DELAY);
	if (active) FS_Sensor_Start();

	return ret;
}
