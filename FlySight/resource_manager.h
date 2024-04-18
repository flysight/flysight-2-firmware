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

#ifndef RESOURCE_MANAGER_H_
#define RESOURCE_MANAGER_H_

typedef enum
{
	FS_RESOURCE_VCC,
	FS_RESOURCE_MICROSD,
	FS_RESOURCE_FATFS,

	// Number of resources
	FS_RESOURCE_COUNT
} FS_Resource_t;

typedef enum
{
	FS_RESOURCE_MANAGER_SUCCESS,
	FS_RESOURCE_MANAGER_FAILURE
} FS_ResourceManager_Result_t;

void FS_ResourceManager_Init(void);
FS_ResourceManager_Result_t FS_ResourceManager_RequestResource(FS_Resource_t resource);
void FS_ResourceManager_ReleaseResource(FS_Resource_t resource);

#endif /* RESOURCE_MANAGER_H_ */
