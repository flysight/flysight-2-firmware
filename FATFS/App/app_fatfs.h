/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_FATFS_H
#define __APP_FATFS_H

/* Includes ------------------------------------------------------------------*/
#include "ff.h"
#include "ff_gen_drv.h"
#include "user_diskio.h" /* defines USER_Driver as external */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
int32_t MX_FATFS_Init(void);
int32_t MX_FATFS_Process(void);
/* USER CODE BEGIN EFP */
int32_t MX_FATFS_DeInit(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define APP_OK                      0
#define APP_ERROR                  -1
#define APP_SD_UNPLUGGED           -2
/* USER CODE END Private defines */

extern FATFS USERFatFs;    /* File system object for USER logical drive */
extern FIL USERFile;       /* File  object for USER */
extern char USERPath[4];   /* USER logical drive path */

#endif /*__APP_FATFS_H */
