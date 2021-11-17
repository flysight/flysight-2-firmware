/*
 * imu.h
 *
 *  Created on: Jun. 9, 2020
 *      Author: Michael
 */

#ifndef IMU_H_
#define IMU_H_

typedef struct
{
	uint32_t time;			// ms
	int32_t wx;				// deg/s * 1000
	int32_t wy;				// deg/s * 1000
	int32_t wz;				// deg/s * 1000
	int32_t ax;				// g * 100000
	int32_t ay;				// g * 100000
	int32_t az;				// g * 100000
	int16_t temperature;	// degrees C * 100
} FS_IMU_Data_t;

void FS_IMU_TransferComplete(void);
void FS_IMU_TransferError(void);

void FS_IMU_Init(void);
void FS_IMU_Start(void);
void FS_IMU_Stop(void);
void FS_IMU_Read(void);
const FS_IMU_Data_t *FS_IMU_GetData(void);
void FS_IMU_DataReady_Callback(void);

#endif /* IMU_H_ */
