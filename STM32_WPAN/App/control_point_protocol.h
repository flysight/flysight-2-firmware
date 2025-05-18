/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2025 Bionic Avionics Inc.                                   **
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

#ifndef APP_CONTROL_POINT_PROTOCOL_H_
#define APP_CONTROL_POINT_PROTOCOL_H_

#include <stdint.h>

#define CP_RESPONSE_ID (0xF0) // Identifier for a response packet

// Common Status Codes for Responses
#define CP_STATUS_SUCCESS                   (0x01)
#define CP_STATUS_CMD_NOT_SUPPORTED         (0x02)
#define CP_STATUS_INVALID_PARAMETER         (0x03)
#define CP_STATUS_OPERATION_FAILED          (0x04)
#define CP_STATUS_OPERATION_NOT_PERMITTED   (0x05)
#define CP_STATUS_BUSY                      (0x06)
#define CP_STATUS_ERROR_UNKNOWN             (0x07)

#define MAX_CP_OPTIONAL_RESPONSE_DATA_LEN 17 // Max optional data in a response

#endif /* APP_CONTROL_POINT_PROTOCOL_H_ */
