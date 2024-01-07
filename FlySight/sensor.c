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

#define HANDLER_COUNT 4
#define TIMEOUT       100

typedef enum
{
	MODE_INACTIVE,
	MODE_ACTIVE
} Mode_t;

static volatile Mode_t mode = MODE_INACTIVE;
static volatile bool busy = false;

typedef enum
{
	Operation_Read,
	Operation_Write,
	Operation_Recieve,
	Operation_Transmit
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
	mode = MODE_ACTIVE;

	if (handlerRdI != handlerWrI)
	{
		BeginRead();
	}
}

void FS_Sensor_Stop(void)
{
	mode = MODE_INACTIVE;
	while (busy);
}

static void BeginRead(void)
{
	Handler_t *h = &handlerBuf[handlerRdI % HANDLER_COUNT];
	HAL_StatusTypeDef result;

	busy = true;

	if (h->op == Operation_Read)
	{
		result = HAL_I2C_Mem_Read_DMA(&hi2c3, h->addr, h->reg, 1, h->pData, h->size);
	}
	else if (h->op == Operation_Write)
	{
		result = HAL_I2C_Mem_Write_DMA(&hi2c3, h->addr, h->reg, 1, h->pData, h->size);
	}
	else if (h->op == Operation_Recieve)
	{
		result = HAL_I2C_Master_Receive_DMA(&hi2c3, h->addr, h->pData, h->size);
	}
	else if (h->op == Operation_Transmit)
	{
		result = HAL_I2C_Master_Transmit_DMA(&hi2c3, h->addr, h->pData, h->size);
	}
	else
	{
		Error_Handler(); // Should never be called
	}

	if (result != HAL_OK)
	{
		NextHandler(result);
	}
}

static void NextHandler(HAL_StatusTypeDef result)
{
	Handler_t *h = &handlerBuf[handlerRdI % HANDLER_COUNT];
	uint32_t primask_bit;

	if (h->Callback)
	{
		h->Callback(result);
	}

	primask_bit = __get_PRIMASK();
	__disable_irq();

	++handlerRdI;
	busy = (mode == MODE_ACTIVE) && (handlerRdI != handlerWrI);

	__set_PRIMASK(primask_bit);

	if (busy)
	{
		BeginRead();
	}
}

void FS_Sensor_TransmitAsync(uint8_t addr, uint8_t *pData, uint16_t size, void (*Callback)(HAL_StatusTypeDef))
{
	Handler_t *h = &handlerBuf[handlerWrI % HANDLER_COUNT];
	uint32_t primask_bit;
	bool begin;

	// Initialize handler
	h->op = Operation_Transmit;
	h->addr = addr;
	h->pData = pData;
	h->size = size;
	h->Callback = Callback;

	primask_bit = __get_PRIMASK();
	__disable_irq();

	++handlerWrI;
	begin = (mode == MODE_ACTIVE) && !busy;

	__set_PRIMASK(primask_bit);

	if (begin)
	{
		BeginRead();
	}
}

void FS_Sensor_ReceiveAsync(uint8_t addr, uint8_t *pData, uint16_t size, void (*Callback)(HAL_StatusTypeDef))
{
	Handler_t *h = &handlerBuf[handlerWrI % HANDLER_COUNT];
	uint32_t primask_bit;
	bool begin;

	// Initialize handler
	h->op = Operation_Recieve;
	h->addr = addr;
	h->pData = pData;
	h->size = size;
	h->Callback = Callback;

	primask_bit = __get_PRIMASK();
	__disable_irq();

	++handlerWrI;
	begin = (mode == MODE_ACTIVE) && !busy;

	__set_PRIMASK(primask_bit);

	if (begin)
	{
		BeginRead();
	}
}

void FS_Sensor_WriteAsync(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size, void (*Callback)(HAL_StatusTypeDef))
{
	Handler_t *h = &handlerBuf[handlerWrI % HANDLER_COUNT];
	uint32_t primask_bit;
	bool begin;

	// Initialize handler
	h->op = Operation_Write;
	h->addr = addr;
	h->reg = reg;
	h->pData = pData;
	h->size = size;
	h->Callback = Callback;

	primask_bit = __get_PRIMASK();
	__disable_irq();

	++handlerWrI;
	begin = (mode == MODE_ACTIVE) && !busy;

	__set_PRIMASK(primask_bit);

	if (begin)
	{
		BeginRead();
	}
}

void FS_Sensor_ReadAsync(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size, void (*Callback)(HAL_StatusTypeDef))
{
	Handler_t *h = &handlerBuf[handlerWrI % HANDLER_COUNT];
	uint32_t primask_bit;
	bool begin;

	// Initialize handler
	h->op = Operation_Read;
	h->addr = addr;
	h->reg = reg;
	h->pData = pData;
	h->size = size;
	h->Callback = Callback;

	primask_bit = __get_PRIMASK();
	__disable_irq();

	++handlerWrI;
	begin = (mode == MODE_ACTIVE) && !busy;

	__set_PRIMASK(primask_bit);

	if (begin)
	{
		BeginRead();
	}
}

HAL_StatusTypeDef FS_Sensor_Transmit(uint8_t addr, uint8_t *pData, uint16_t size)
{
	HAL_StatusTypeDef result;
	uint32_t ms;

	if (mode == MODE_ACTIVE)
	{
		return HAL_ERROR;
	}
	else
	{
		ms = HAL_GetTick();
		do
		{
			if (HAL_GetTick() - ms > TIMEOUT)
			{
				Error_Handler();
			}

			result = HAL_I2C_Master_Transmit(&hi2c3, addr, pData, size, TIMEOUT);
		}
		while (result != HAL_OK);

		return result;
	}
}

HAL_StatusTypeDef FS_Sensor_Receive(uint8_t addr, uint8_t *pData, uint16_t size)
{
	HAL_StatusTypeDef result;
	uint32_t ms;

	if (mode == MODE_ACTIVE)
	{
		return HAL_ERROR;
	}
	else
	{
		ms = HAL_GetTick();
		do
		{
			if (HAL_GetTick() - ms > TIMEOUT)
			{
				Error_Handler();
			}

			result = HAL_I2C_Master_Receive(&hi2c3, addr, pData, size, TIMEOUT);
		}
		while (result != HAL_OK);

		return result;
	}
}

HAL_StatusTypeDef FS_Sensor_Write(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size)
{
	HAL_StatusTypeDef result;
	uint32_t ms;

	if (mode == MODE_ACTIVE)
	{
		return HAL_ERROR;
	}
	else
	{
		ms = HAL_GetTick();
		do
		{
			if (HAL_GetTick() - ms > TIMEOUT)
			{
				Error_Handler();
			}

			result = HAL_I2C_Mem_Write(&hi2c3, addr, reg, 1, pData, size, TIMEOUT);
		}
		while (result != HAL_OK);

		return result;
	}
}

HAL_StatusTypeDef FS_Sensor_Read(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size)
{
	HAL_StatusTypeDef result;
	uint32_t ms;

	if (mode == MODE_ACTIVE)
	{
		return HAL_ERROR;
	}
	else
	{
		ms = HAL_GetTick();
		do
		{
			if (HAL_GetTick() - ms > TIMEOUT)
			{
				Error_Handler();
			}

			result = HAL_I2C_Mem_Read(&hi2c3, addr, reg, 1, pData, size, TIMEOUT);
		}
		while (result != HAL_OK);

		return result;
	}
}
