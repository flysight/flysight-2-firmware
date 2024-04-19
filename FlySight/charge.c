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

#include "main.h"
#include "app_common.h"
#include "charge.h"

FS_Charge_State_t FS_Charge_GetState(void)
{
	if (HAL_GPIO_ReadPin(CHG_STAT_GPIO_Port, CHG_STAT_Pin))
	{
		return FS_CHARGE_COMPLETE;
	}
	else
	{
		return FS_CHARGE_ACTIVE;
	}
}

void FS_Charge_SetCurrent(FS_Charge_Current_t charge_current)
{
	GPIO_PinState chg_en_lo, chg_en_hi;

	chg_en_lo = ((~charge_current) >> 0) & 1;
	chg_en_hi = ((~charge_current) >> 1) & 1;

	HAL_GPIO_WritePin(CHG_EN_LO_GPIO_Port, CHG_EN_LO_Pin, chg_en_lo);
	HAL_GPIO_WritePin(CHG_EN_HI_GPIO_Port, CHG_EN_HI_Pin, chg_en_hi);
}
