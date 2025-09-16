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

#include <stdbool.h>

#include "main.h"
#include "app_common.h"
#include "log.h"
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

static volatile uint32_t overrun_count;

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
	overrun_count = 0;
	mode = MODE_ACTIVE;

	if (handlerRdI != handlerWrI)
	{
		BeginRead();
	}
}

void FS_Sensor_Stop(void)
{
	if (overrun_count > 0)
	{
		FS_Log_WriteEvent("Sensor data overruns: %lu", overrun_count);
	}

	mode = MODE_INACTIVE;
	while (busy);
}

static void BeginRead(void)
{
	Handler_t *h = &handlerBuf[handlerRdI % HANDLER_COUNT];
	HAL_StatusTypeDef result = HAL_ERROR;

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

	// Check if the handler queue is full
	if (handlerWrI - handlerRdI >= HANDLER_COUNT)
	{
		++overrun_count;
		return;
	}

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

	// Check if the handler queue is full
	if (handlerWrI - handlerRdI >= HANDLER_COUNT)
	{
		++overrun_count;
		return;
	}

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

	// Check if the handler queue is full
	if (handlerWrI - handlerRdI >= HANDLER_COUNT)
	{
		++overrun_count;
		return;
	}

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

	// Check if the handler queue is full
	if (handlerWrI - handlerRdI >= HANDLER_COUNT)
	{
		++overrun_count;
		return;
	}

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
	HAL_StatusTypeDef result = HAL_ERROR;
	uint32_t ms;

	if (mode == MODE_INACTIVE)
	{
		ms = HAL_GetTick();
		do
		{
			if (HAL_GetTick() - ms > TIMEOUT)
			{
				return HAL_TIMEOUT;
			}

			result = HAL_I2C_Master_Transmit(&hi2c3, addr, pData, size, TIMEOUT);
		}
		while (result != HAL_OK);
	}

	return result;
}

HAL_StatusTypeDef FS_Sensor_Receive(uint8_t addr, uint8_t *pData, uint16_t size)
{
	HAL_StatusTypeDef result = HAL_ERROR;
	uint32_t ms;

	if (mode == MODE_INACTIVE)
	{
		ms = HAL_GetTick();
		do
		{
			if (HAL_GetTick() - ms > TIMEOUT)
			{
				return HAL_TIMEOUT;
			}

			result = HAL_I2C_Master_Receive(&hi2c3, addr, pData, size, TIMEOUT);
		}
		while (result != HAL_OK);
	}

	return result;
}

HAL_StatusTypeDef FS_Sensor_Write(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size)
{
	HAL_StatusTypeDef result = HAL_ERROR;
	uint32_t ms;

	if (mode == MODE_INACTIVE)
	{
		ms = HAL_GetTick();
		do
		{
			if (HAL_GetTick() - ms > TIMEOUT)
			{
				return HAL_TIMEOUT;
			}

			result = HAL_I2C_Mem_Write(&hi2c3, addr, reg, 1, pData, size, TIMEOUT);
		}
		while (result != HAL_OK);
	}

	return result;
}

HAL_StatusTypeDef FS_Sensor_Read(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size)
{
	HAL_StatusTypeDef result = HAL_ERROR;
	uint32_t ms;

	if (mode == MODE_INACTIVE)
	{
		ms = HAL_GetTick();
		do
		{
			if (HAL_GetTick() - ms > TIMEOUT)
			{
				return HAL_TIMEOUT;
			}

			result = HAL_I2C_Mem_Read(&hi2c3, addr, reg, 1, pData, size, TIMEOUT);
		}
		while (result != HAL_OK);
	}

	return result;
}
