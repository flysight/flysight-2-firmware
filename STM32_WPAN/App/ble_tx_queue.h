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

#ifndef APP_BLE_TX_QUEUE_H_
#define APP_BLE_TX_QUEUE_H_

#include "custom_stm.h"

void BLE_TX_Queue_Init(void);
void BLE_TX_Queue_TxPoolAvailableNotification(void);

uint8_t *BLE_TX_Queue_GetNextTxPacket(void);
void BLE_TX_Queue_SendNextTxPacket(
		Custom_STM_Char_Opcode_t opcode, uint8_t length);

#endif /* APP_BLE_TX_QUEUE_H_ */
