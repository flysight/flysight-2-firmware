/*
 * sensor.h
 *
 *  Created on: Aug. 1, 2020
 *      Author: Michael
 */

#ifndef SENSOR_H_
#define SENSOR_H_

void FS_Sensor_Start(void);
void FS_Sensor_Stop(void);

void FS_Sensor_WriteAsync(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size, void (*Callback)(HAL_StatusTypeDef));
void FS_Sensor_ReadAsync(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size, void (*Callback)(HAL_StatusTypeDef));

void FS_Sensor_TransferComplete(void);
void FS_Sensor_TransferError(void);

HAL_StatusTypeDef FS_Sensor_Write(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size);
HAL_StatusTypeDef FS_Sensor_Read(uint8_t addr, uint16_t reg, uint8_t *pData, uint16_t size);

#endif /* SENSOR_H_ */
