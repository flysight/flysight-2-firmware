/*
 * log.h
 *
 *  Created on: Sep. 29, 2020
 *      Author: Michael
 */

#ifndef LOG_H_
#define LOG_H_

#include "gnss.h"
#include "imu.h"
#include "led.h"

void FS_Log_Init(uint32_t sessionId);
void FS_Log_DeInit(uint32_t sessionId);

void FS_Log_WriteGNSSData(const FS_GNSS_Data_t *current);
void FS_Log_WriteGNSSTime(const FS_GNSS_Time_t *current);
void FS_Log_WriteIMUData(const FS_IMU_Data_t *current);

#endif /* LOG_H_ */
