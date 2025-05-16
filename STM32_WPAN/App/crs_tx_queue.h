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

#ifndef APP_CRS_TX_QUEUE_H_
#define APP_CRS_TX_QUEUE_H_

typedef struct
{
  uint8_t data[244];
  uint8_t length;
} CRS_TX_Queue_Packet_t;

void CRS_TX_Queue_Init(void);
void CRS_TX_Queue_TxPoolAvailableNotification(void);

CRS_TX_Queue_Packet_t *CRS_TX_Queue_GetNextTxPacket(void);
void CRS_TX_Queue_SendNextTxPacket(void);

#endif /* APP_CRS_TX_QUEUE_H_ */
