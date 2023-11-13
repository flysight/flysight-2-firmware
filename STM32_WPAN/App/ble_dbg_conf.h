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
#ifndef __BLE_DBG_CONF_H
#define __BLE_DBG_CONF_H

/**
 * Enable or Disable traces from BLE
 */

#define BLE_DBG_APP_EN             0
#define BLE_DBG_DIS_EN             0
#define BLE_DBG_HRS_EN             0
#define BLE_DBG_SVCCTL_EN          0
#define BLE_DBG_BLS_EN             0
#define BLE_DBG_HTS_EN             0
#define BLE_DBG_P2P_STM_EN         0

/**
 * Macro definition
 */
#if ( BLE_DBG_APP_EN != 0 )
#define BLE_DBG_APP_MSG             PRINT_MESG_DBG
#else
#define BLE_DBG_APP_MSG             PRINT_NO_MESG
#endif

#if ( BLE_DBG_DIS_EN != 0 )
#define BLE_DBG_DIS_MSG             PRINT_MESG_DBG
#else
#define BLE_DBG_DIS_MSG             PRINT_NO_MESG
#endif

#if ( BLE_DBG_HRS_EN != 0 )
#define BLE_DBG_HRS_MSG             PRINT_MESG_DBG
#else
#define BLE_DBG_HRS_MSG             PRINT_NO_MESG
#endif

#if ( BLE_DBG_P2P_STM_EN != 0 )
#define BLE_DBG_P2P_STM_MSG         PRINT_MESG_DBG
#else
#define BLE_DBG_P2P_STM_MSG         PRINT_NO_MESG
#endif

#if ( BLE_DBG_TEMPLATE_STM_EN != 0 )
#define BLE_DBG_TEMPLATE_STM_MSG         PRINT_MESG_DBG
#else
#define BLE_DBG_TEMPLATE_STM_MSG         PRINT_NO_MESG
#endif

#if ( BLE_DBG_EDS_STM_EN != 0 )
#define BLE_DBG_EDS_STM_MSG         PRINT_MESG_DBG
#else
#define BLE_DBG_EDS_STM_MSG         PRINT_NO_MESG
#endif

#if ( BLE_DBG_LBS_STM_EN != 0 )
#define BLE_DBG_LBS_STM_MSG         PRINT_MESG_DBG
#else
#define BLE_DBG_LBS_STM_MSG         PRINT_NO_MESG
#endif

#if ( BLE_DBG_SVCCTL_EN != 0 )
#define BLE_DBG_SVCCTL_MSG          PRINT_MESG_DBG
#else
#define BLE_DBG_SVCCTL_MSG          PRINT_NO_MESG
#endif

#if (BLE_DBG_CTS_EN != 0)
#define BLE_DBG_CTS_MSG             PRINT_MESG_DBG
#else
#define BLE_DBG_CTS_MSG             PRINT_NO_MESG
#endif

#if (BLE_DBG_HIDS_EN != 0)
#define BLE_DBG_HIDS_MSG            PRINT_MESG_DBG
#else
#define BLE_DBG_HIDS_MSG            PRINT_NO_MESG
#endif

#if (BLE_DBG_PASS_EN != 0)
#define BLE_DBG_PASS_MSG            PRINT_MESG_DBG
#else
#define BLE_DBG_PASS_MSG            PRINT_NO_MESG
#endif

#if (BLE_DBG_BLS_EN != 0)
#define BLE_DBG_BLS_MSG             PRINT_MESG_DBG
#else
#define BLE_DBG_BLS_MSG             PRINT_NO_MESG
#endif

#if (BLE_DBG_HTS_EN != 0)
#define BLE_DBG_HTS_MSG             PRINT_MESG_DBG
#else
#define BLE_DBG_HTS_MSG             PRINT_NO_MESG
#endif

#if (BLE_DBG_ANS_EN != 0)
#define BLE_DBG_ANS_MSG             PRINT_MESG_DBG
#else
#define BLE_DBG_ANS_MSG             PRINT_NO_MESG
#endif

#if (BLE_DBG_ESS_EN != 0)
#define BLE_DBG_ESS_MSG             PRINT_MESG_DBG
#else
#define BLE_DBG_ESS_MSG             PRINT_NO_MESG
#endif

#if (BLE_DBG_GLS_EN != 0)
#define BLE_DBG_GLS_MSG             PRINT_MESG_DBG
#else
#define BLE_DBG_GLS_MSG             PRINT_NO_MESG
#endif

#if (BLE_DBG_BAS_EN != 0)
#define BLE_DBG_BAS_MSG             PRINT_MESG_DBG
#else
#define BLE_DBG_BAS_MSG             PRINT_NO_MESG
#endif

#if (BLE_DBG_RTUS_EN != 0)
#define BLE_DBG_RTUS_MSG            PRINT_MESG_DBG
#else
#define BLE_DBG_RTUS_MSG            PRINT_NO_MESG
#endif

#if (BLE_DBG_HPS_EN != 0)
#define BLE_DBG_HPS_MSG             PRINT_MESG_DBG
#else
#define BLE_DBG_HPS_MSG             PRINT_NO_MESG
#endif

#if (BLE_DBG_TPS_EN != 0)
#define BLE_DBG_TPS_MSG             PRINT_MESG_DBG
#else
#define BLE_DBG_TPS_MSG             PRINT_NO_MESG
#endif

#if (BLE_DBG_LLS_EN != 0)
#define BLE_DBG_LLS_MSG             PRINT_MESG_DBG
#else
#define BLE_DBG_LLS_MSG             PRINT_NO_MESG
#endif

#if (BLE_DBG_IAS_EN != 0)
#define BLE_DBG_IAS_MSG             PRINT_MESG_DBG
#else
#define BLE_DBG_IAS_MSG             PRINT_NO_MESG
#endif

#if (BLE_DBG_WSS_EN != 0)
#define BLE_DBG_WSS_MSG             PRINT_MESG_DBG
#else
#define BLE_DBG_WSS_MSG             PRINT_NO_MESG
#endif

#if (BLE_DBG_LNS_EN != 0)
#define BLE_DBG_LNS_MSG             PRINT_MESG_DBG
#else
#define BLE_DBG_LNS_MSG             PRINT_NO_MESG
#endif

#if (BLE_DBG_SCPS_EN != 0)
#define BLE_DBG_SCPS_MSG            PRINT_MESG_DBG
#else
#define BLE_DBG_SCPS_MSG            PRINT_NO_MESG
#endif

#if (BLE_DBG_DTS_EN != 0)
#define BLE_DBG_DTS_MSG             PRINT_MESG_DBG
#define BLE_DBG_DTS_BUF             PRINT_LOG_BUFF_DBG
#else
#define BLE_DBG_DTS_MSG             PRINT_NO_MESG
#define BLE_DBG_DTS_BUF             PRINT_NO_MESG
#endif

#endif /*__BLE_DBG_CONF_H */
