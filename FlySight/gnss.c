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
#include "config.h"
#include "gnss.h"
#include "log.h"
#include "state.h"
#include "stm32_seq.h"

#define GNSS_RATE           921600	// Baud rate
#define GNSS_TIMEOUT        100		// ACK/NAK timeout (ms)

#define GNSS_UPDATE_MSEC    40
#define GNSS_UPDATE_RATE    (GNSS_UPDATE_MSEC*1000/CFG_TS_TICK_VAL)

#define UBX_NUM_CHANNELS	72		// For MAX-M8
#define UBX_PAYLOAD_LEN		(8+12*UBX_NUM_CHANNELS) // Payload for single UBX message

#define UBX_NUM_CFG_BYTES   36

#define UBX_SYNC_1			0xb5	// UBX sync bytes
#define UBX_SYNC_2			0x62

#define UBX_NAV             0x01
#define UBX_NAV_POSLLH      0x02
#define UBX_NAV_STATUS      0x03
#define UBX_NAV_SOL         0x06
#define UBX_NAV_PVT         0x07
#define UBX_NAV_VELNED      0x12
#define UBX_NAV_TIMEUTC     0x21
#define UBX_NAV_SAT         0x35

#define UBX_RXM             0x02
#define UBX_RXM_PMREQ       0x41

#define UBX_ACK             0x05
#define UBX_ACK_NAK         0x00
#define UBX_ACK_ACK         0x01

#define UBX_CFG             0x06
#define UBX_CFG_PRT         0x00
#define UBX_CFG_MSG         0x01
#define UBX_CFG_RST         0x04
#define UBX_CFG_RATE        0x08
#define UBX_CFG_RXM         0x11
#define UBX_CFG_NAV5        0x24
#define UBX_CFG_TP5         0x31
#define UBX_CFG_PM2         0x3b
#define UBX_CFG_VALSET      0x8a

#define UBX_MON             0x0a
#define UBX_MON_TXBUF       0x08
#define UBX_MON_HW          0x09
#define UBX_MON_SPAN        0x31

#define UBX_TIM             0x0d
#define UBX_TIM_TP          0x01
#define UBX_TIM_TM2         0x03

#define UBX_NMEA            0xf0
#define UBX_NMEA_GPGGA      0x00
#define UBX_NMEA_GPGLL      0x01
#define UBX_NMEA_GPGSA      0x02
#define UBX_NMEA_GPGSV      0x03
#define UBX_NMEA_GPRMC      0x04
#define UBX_NMEA_GPVTG      0x05

#define UBX_SEC             0x27
#define UBX_SEC_ECSIGN      0x04

#define UBX_MSG_PVT         0x01
#define UBX_MSG_VELNED      0x02
#define UBX_MSG_ALL         (UBX_MSG_PVT | UBX_MSG_VELNED)

#define UBX_CFG_SEC_ECCFGSESSIONID0 0x06, 0x00, 0xf6, 0x50
#define UBX_CFG_SEC_ECCFGSESSIONID1 0x07, 0x00, 0xf6, 0x50
#define UBX_CFG_SEC_ECCFGSESSIONID2 0x08, 0x00, 0xf6, 0x50

typedef struct
{
	uint8_t msgClass;  // Message class
	uint8_t msgID;     // Message identifier
	uint8_t rate;      // Send rate
}
ubxCfgMsg_t;

typedef struct
{
	uint8_t  portID;       // Port identifier number
	uint8_t  reserved0;    // Reserved
	uint16_t txReady;      // TX ready pin configuration
	uint32_t mode;         // UART mode
	uint32_t baudRate;     // Baud rate (bits/sec)
	uint16_t inProtoMask;  // Input protocols
	uint16_t outProtoMask; // Output protocols
	uint16_t flags;        // Flags
	uint16_t reserved5;    // Always set to zero
}
ubxCfgPrt_t;

typedef struct
{
	uint16_t measRate; // Measurement rate             (ms)
	uint16_t navRate;  // Navigation rate, in number
	                   //   of measurement cycles
	uint16_t timeRef;  // Alignment to reference time:
	                   //   0 = UTC time; 1 = GPS time
}
ubxCfgRate_t;

typedef struct
{
	uint16_t navBbrMask; // BBR sections to clear
	uint8_t  resetMode;  // Reset type
	uint8_t  res;        // Reserved
}
ubxCfgRst_t;

typedef struct
{
	uint16_t mask;             // Only masked parameters will be applied
	uint8_t  dynModel;         // Dynamic platform model
	uint8_t  fixMode;          // Position fixing mode
	int32_t  fixedAlt;         // Fixed altitude (MSL) for 2D mode       (m)
	uint32_t fixedAltVar;      // Fixed altitude variance for 2D mode    (m^2)
	int8_t   minElev;          // Minimum elevation for satellite        (deg)
	uint8_t  drLimit;          // Maximum time to perform dead reckoning (s)
	uint16_t pDop;             // Position DOP mask
	uint16_t tDop;             // Time DOP mask
	uint16_t pAcc;             // Position accuracy mask                 (m)
	uint16_t tAcc;             // Time accuracy mask                     (m)
	uint8_t  staticHoldThresh; // Static hold threshold                  (cm/s)
	uint8_t  res1;             // Reserved, set to 0
	uint32_t res2;             // Reserved, set to 0
	uint32_t res3;             // Reserved, set to 0
	uint32_t res4;             // Reserved, set to 0
}
ubxCfgNav5_t;

typedef struct
{
	uint8_t  tpIdx;             // Time pulse selection (0 = TIMEPULSE, 1 = TIMEPULSE2)
	uint8_t  version;           // Version, 0 for this message
	uint16_t reserved1;         // Reserved
	int16_t  antCableDelay;     // Antenna cable delay (ns)
	int16_t  rfGroupDelay;      // RF group delay (ns)
	uint32_t freqPeriod;        // Frequency or period time
	uint32_t freqPeriodLock;    // Frequency or period time when locked to GPS time
	uint32_t pulseLenRatio;     // Pulse length or duty cycle
	uint32_t pulseLenRatioLock; // Pulse length or duty cycle when locked to GPS time
	int32_t  userConfigDelay;   // User configurable time pulse delay
	uint32_t flags;             // Configuration flags
}
ubxCfgTp5_t;

typedef struct
{
	uint8_t version;
	uint8_t layers;
	uint8_t reserved0[2];
	uint8_t cfgData[UBX_NUM_CFG_BYTES];
}
ubxCfgValset_t;

typedef struct
{
	uint32_t iTOW;     // GPS time of week             (ms)
	int32_t  lon;      // Longitude                    (deg)
	int32_t  lat;      // Latitude                     (deg)
	int32_t  height;   // Height above ellipsoid       (mm)
	int32_t  hMSL;     // Height above mean sea level  (mm)
	uint32_t hAcc;     // Horizontal accuracy estimate (mm)
	uint32_t vAcc;     // Vertical accuracy estimate   (mm)
}
ubxNavPosLlh_t;        // 28 bytes total

typedef struct
{
	uint32_t iTOW;     // GPS time of week             (ms)
	int32_t  fTOW;     // Fractional nanoseconds       (ns)
	int16_t  week;     // GPS week
	uint8_t  gpsFix;   // GPS fix type
	uint8_t  flags;    // Fix status flags
	int32_t  ecefX;    // ECEF X coordinate            (cm)
	int32_t  ecefY;    // ECEF Y coordinate            (cm)
	int32_t  ecefZ;    // ECEF Z coordinate            (cm)
	uint32_t pAcc;     // 3D position accuracy         (cm)
	int32_t  ecefVX;   // ECEF X velocity              (cm/s)
	int32_t  ecefVY;   // ECEF Y velocity              (cm/s)
	int32_t  ecefVZ;   // ECEF Z velocity              (cm/s)
	uint32_t sAcc;     // Speed accuracy               (cm/s)
	uint16_t pDOP;     // Position DOP
	uint8_t  res1;     // Reserved
	uint8_t  numSV;    // Number of SVs in solution
	uint32_t res2;     // Reserved
}
ubxNavSol_t;           // 52 bytes total

typedef struct
{
	uint32_t iTOW;         // GPS time of week              (ms)
	uint16_t year;         // Year                          (1999..2099)
	uint8_t  month;        // Month                         (1..12)
	uint8_t  day;          // Day of month                  (1..31)
	uint8_t  hour;         // Hour of day                   (0..23)
	uint8_t  min;          // Minute of hour                (0..59)
	uint8_t  sec;          // Second of minute              (0..59)
	uint8_t  valid;        // Validity flags
	uint32_t tAcc;         // Time accuracy estimate        (ns)
	int32_t  nano;         // Nanoseconds of second         (ns)
	uint8_t  gpsFix;       // GPS fix type
	uint8_t  flags;        // Fix status flags
	uint8_t  flags2;       // Additional flags
	uint8_t  numSV;        // Number of SVs in solution
	int32_t  lon;          // Longitude                     (deg)
	int32_t  lat;          // Latitude                      (deg)
	int32_t  height;       // Height above ellipsoid        (mm)
	int32_t  hMSL;         // Height above mean sea level   (mm)
	uint32_t hAcc;         // Horizontal accuracy estimate  (mm)
	uint32_t vAcc;         // Vertical accuracy estimate    (mm)
	int32_t  velN;         // North velocity                (mm/s)
	int32_t  velE;         // East velocity                 (mm/s)
	int32_t  velD;         // Down velocity                 (mm/s)
	int32_t  gSpeed;       // Ground speed                  (mm/s)
	int32_t  headMot;      // 2D heading of motion          (deg)
	uint32_t sAcc;         // Speed accuracy estimate       (mm/s)
	uint32_t headAcc;      // Heading accuracy estimate     (deg)
	uint16_t pDOP;         // Position DOP
	uint8_t  flags3;       // Additional flags
	uint8_t  reserved0[5]; // Reserved
	int32_t  headVeh;      // 2D heading of vehicle         (deg)
	int16_t  magDec;       // Magnetic declination          (deg)
	uint16_t magAcc;       // Magnetic declination accuracy (deg)
}
ubxNavPvt_t;               // 92 bytes total

typedef struct
{
	uint32_t iTOW;     // GPS time of week             (ms)
	int32_t  velN;     // North velocity               (cm/s)
	int32_t  velE;     // East velocity                (cm/s)
	int32_t  velD;     // Down velocity                (cm/s)
	uint32_t speed;    // 3D speed                     (cm/s)
	uint32_t gSpeed;   // Ground speed                 (cm/s)
	int32_t  heading;  // 2D heading                   (deg)
	uint32_t sAcc;     // Speed accuracy estimate      (cm/s)
	uint32_t cAcc;     // Heading accuracy estimate    (deg)
}
ubxNavVelNed_t;        // 36 bytes total

typedef struct
{
	uint32_t iTOW;     // GPS time of week             (ms)
	uint32_t tAcc;     // Time accuracy estimate       (ns)
	int32_t  nano;     // Nanoseconds of second        (ns)
	uint16_t year;     // Year                         (1999..2099)
	uint8_t  month;    // Month                        (1..12)
	uint8_t  day;      // Day of month                 (1..31)
	uint8_t  hour;     // Hour of day                  (0..23)
	uint8_t  min;      // Minute of hour               (0..59)
	uint8_t  sec;      // Second of minute             (0..59)
	uint8_t  valid;    // Validity flags
}
ubxNavTimeUtc_t;       // 20 bytes total

typedef struct
{
	uint8_t  gnssId;   // GNSS identifier
	uint8_t  svId;     // Satellite identifier
	uint8_t  cno;      // Carrier to noise ratio       (dbHz)
	int8_t   elev;     // Elevation                    (deg)
	int16_t  azim;     // Azimuth                      (deg)
	int16_t  prRes;    // Pseudo range residual        (m)
	uint32_t flags;    // Bitmask
}
ubxNavSatSv_t;         // 12 bytes total

typedef struct
{
	uint32_t iTOW;     // GPS time of week             (ms)
	uint8_t  version;  // Message version
	uint8_t  numSvs;   // Number of satellites
	uint16_t res1;
	ubxNavSatSv_t sv[UBX_NUM_CHANNELS];
}
ubxNavSat_t;           // 8 + 12 * numSvs bytes total

typedef struct
{
	uint32_t towMS;     // Time pulse time of week      (ms)
	uint32_t towSubMS;  // Submillisecond part of towMS (ms * 2^32)
	int32_t  qErr;      // Quantization error           (ps)
	uint16_t week;      // Time pulse week number
	uint8_t  flags;     // Bitmask
	uint8_t  refInfo;   // Time reference information
}
ubxTimTp_t;             // 16 bytes total

typedef struct
{
	uint8_t  ch;        // Channel
	uint8_t  flags;     // Bitmask
	uint16_t count;     // Rising edge counter
	uint16_t wnR;       // Week number of last rising edge
	uint16_t wnF;       // Week number of last falling edge
	uint32_t towMsR;    // TOW of rising edge            (ms)
	uint32_t towSubMsR; // Submillisecond part of towMsR (ns)
	uint32_t towMsF;    // TOW of falling edge           (ms)
	uint32_t towSubMsF; // Submillisecond part of towMsR (ns)
	uint32_t accEst;    // Accuracy estimate             (ns)
}
ubxTimTm2_t;            // 28 bytes total

typedef struct
{
	uint16_t pending[6];   // Bytes pending in transmit buffer
	uint8_t  usage[6];     // Maximum usage in last period each target (%)
	uint8_t  peakUsage[6]; // Maximum usage each target (%)
	uint8_t  tUsage;       // Maximum usage last period all targets (%)
	uint8_t  tPeakUsage;   // Maximum usage all targets (%)
	uint8_t  errors;       // Error bitmask
	uint8_t  reserved;     // Reserved
}
ubxMonTxbuf_t;             // 28 bytes total

typedef struct
{
	uint8_t clsID;     // Class ID of acknowledged message
	uint8_t msgID;     // Message ID of acknowledged message
}
ubxAckAck_t;

typedef struct
{
	uint8_t clsID;     // Class ID of not-acknowledged message
	uint8_t msgID;     // Message ID of not-acknowledged message
}
ubxAckNak_t;

union
{
	uint8_t       whole[GNSS_RX_BUF_LEN];	// data buffer
	FS_GNSS_Raw_t split[2];					// raw output buffers
} gnssRxData;

uint32_t gnssRxIndex = 0;				// read index
uint8_t  gnssRawIndex = 0;				// raw output index

// Current UBX message
static uint8_t   gnssMsgClass;
static uint8_t   gnssMsgId;
static uint16_t  gnssPayloadLen;
static union
{
	uint8_t buf[UBX_PAYLOAD_LEN];
	ubxAckAck_t     ackAck;
	ubxAckNak_t     ackNak;
	ubxNavPvt_t     navPvt;
	ubxNavPosLlh_t  navPosLlh;
	ubxNavVelNed_t  navVelNed;
	ubxNavTimeUtc_t navTimeUtc;
	ubxNavSat_t     navSat;
	ubxTimTp_t      timTp;
	ubxTimTm2_t     timTm2;
} gnssPayload;

// Saved GNSS messages
static uint32_t gnssTimeOfWeek;
static uint8_t  gnssMsgReceived;
static bool validTime;

static FS_GNSS_Data_t gnssData;
static FS_GNSS_Time_t gnssTime;
static FS_GNSS_Int_t  gnssInt;

static uint8_t timer_id;

static enum
{
	st_sync_1,
	st_sync_2,
	st_class,
	st_id,
	st_length_1,
	st_length_2,
	st_payload,
	st_ck_a,
	st_ck_b
} gnssState;

static uint32_t updateCount;
static uint32_t updateTotalTime;
static uint32_t updateMaxTime;
static uint32_t updateLastCall;
static uint32_t updateMaxInterval;
static uint32_t bufferUsed;

// Error logging
static volatile bool gnss_is_initializing = 0;
static volatile uint32_t uart_error_code;

// UART handle
extern UART_HandleTypeDef huart1;

static void FS_GNSS_Timer(void);
static void FS_GNSS_Update(void);

static void (*data_ready_callback)(void) = NULL;
static void (*time_ready_callback)(bool validTime) = NULL;
static void (*raw_ready_callback)(void) = NULL;
static void (*int_ready_callback)(void) = NULL;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	uart_error_code = huart->ErrorCode;

	// During initialization, a framing error is expected due to the baud rate switch.
	// We should ignore it and continue.
	if (gnss_is_initializing && (uart_error_code & HAL_UART_ERROR_FE))
	{
		return;
	}

	// Check for common, recoverable data errors (Overrun, Noise, Framing)
	if (uart_error_code & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE))
	{
		// These are non-fatal. We will try to recover.
		// Immediately restart the DMA transfer to continue receiving data.
		if (HAL_UART_Receive_DMA(&huart1, gnssRxData.whole, GNSS_RX_BUF_LEN) == HAL_OK)
		{
			// If restart was successful, schedule a task to log the error message later.
			UTIL_SEQ_SetTask(1 << CFG_TASK_FS_GNSS_LOG_ERROR_ID, CFG_SCH_PRIO_1);
		}
		else
		{
			// The recovery mechanism itself has failed. This is a critical, unrecoverable fault.
			// The UART is now in an unknown state and cannot receive data.
			Error_Handler();
		}
	}
	else
	{
		// Any other error (especially HAL_UART_ERROR_DMA) is considered critical and unrecoverable.
		// Halt the system. The error code can be inspected in a debug session.
		Error_Handler();
	}
}

static void FS_GNSS_LogError(void)
{
	FS_Log_WriteEvent("GNSS UART fatal error: 0x%lX", uart_error_code);
}

static uint8_t FS_GNSS_GetChar(void)
{
	uint8_t ch = gnssRxData.whole[gnssRxIndex];
	gnssRxIndex = (gnssRxIndex + 1) % GNSS_RX_BUF_LEN;
	return ch;
}

static void FS_GNSS_PutChar(uint8_t ch)
{
	HAL_UART_Transmit(&huart1, &ch, 1, HAL_MAX_DELAY);
}

static uint8_t FS_GNSS_HandleByte(unsigned char ch)
{
	uint8_t ret = 0;

	static uint8_t ck_a, ck_b;
	static uint16_t index;

	switch (gnssState)
	{
	case st_sync_1:
		if (ch == UBX_SYNC_1)
		{
			gnssState = st_sync_2;
		}
		break;
	case st_sync_2:
		if (ch == UBX_SYNC_2)
		{
			gnssState = st_class;
		}
		else
		{
			gnssState = st_sync_1;
		}
		break;
	case st_class:
		gnssMsgClass = ch;
		ck_a = ck_b = ch;
		gnssState = st_id;
		break;
	case st_id:
		gnssMsgId = ch;
		ck_a += ch;
		ck_b += ck_a;
		gnssState = st_length_1;
		break;
	case st_length_1:
		gnssPayloadLen = ch;
		ck_a += ch;
		ck_b += ck_a;
		gnssState = st_length_2;
		break;
	case st_length_2:
		gnssPayloadLen += ch << 8;
		ck_a += ch;
		ck_b += ck_a;
		if (gnssPayloadLen == 0)
		{
			gnssState = st_ck_a;
		}
		else if (gnssPayloadLen <= UBX_PAYLOAD_LEN)
		{
			gnssState = st_payload;
			index = 0;
		}
		else
		{
			gnssState = st_sync_1;
		}
		break;
	case st_payload:
		gnssPayload.buf[index++] = ch;
		ck_a += ch;
		ck_b += ck_a;
		if (index == gnssPayloadLen)
		{
			gnssState = st_ck_a;
		}
		break;
	case st_ck_a:
		if (ck_a == ch)
		{
			gnssState = st_ck_b;
		}
		else
		{
			gnssState = st_sync_1;
		}
		break;
	case st_ck_b:
		if (ck_b == ch)
		{
			ret = 1;
		}
		gnssState = st_sync_1;
		break;
	}

	return ret;
}

static uint8_t FS_GNSS_WaitForAck(uint8_t msg_class, uint8_t msg_id, uint16_t timeout)
{
	const uint32_t ms = HAL_GetTick();

	while (HAL_GetTick() < ms + timeout)
	{
		if (gnssRxIndex != GNSS_RX_BUF_LEN - huart1.hdmarx->Instance->CNDTR)
		{
			if (FS_GNSS_HandleByte(FS_GNSS_GetChar()))
			{
				if (gnssMsgClass == UBX_ACK &&
				    gnssMsgId == UBX_ACK_ACK)
				{
					if (gnssPayload.ackAck.clsID == msg_class &&
					    gnssPayload.ackAck.msgID == msg_id)
					{
						return 1; // ACK
					}
				}
				else if (gnssMsgClass == UBX_ACK &&
				         gnssMsgId == UBX_ACK_NAK)
				{
					if (gnssPayload.ackNak.clsID == msg_class &&
					    gnssPayload.ackNak.msgID == msg_id)
					{
						return 0; // NAK
					}
				}
			}
		}
	}

	return 0;
}

static void FS_GNSS_SendMessage(uint8_t msgClass, uint8_t msgId, uint16_t size, const void *data)
{
	uint16_t i;
	const uint8_t *bytes = (const uint8_t *) data;
	uint8_t ckA = 0, ckB = 0;

	FS_GNSS_PutChar(UBX_SYNC_1);
	FS_GNSS_PutChar(UBX_SYNC_2);

	#define SEND_BYTE(a) {		\
		FS_GNSS_PutChar(a);			\
		ckA += a; ckB += ckA; }

	SEND_BYTE(msgClass);
	SEND_BYTE(msgId);

	SEND_BYTE(size & 0xff);
	SEND_BYTE((size >> 8) & 0xff);

	for (i = 0; i < size; ++i)
	{
		SEND_BYTE(bytes[i]);
	}

	#undef SEND_BYTE

	FS_GNSS_PutChar(ckA);
	FS_GNSS_PutChar(ckB);
}

static void FS_GNSS_ReceiveMessage(uint8_t msgReceived, uint32_t timeOfWeek)
{
	if (timeOfWeek != gnssTimeOfWeek)
	{
		gnssTimeOfWeek = timeOfWeek;
		gnssMsgReceived = 0;
	}

	gnssMsgReceived |= msgReceived;

	if (gnssMsgReceived == UBX_MSG_ALL)
	{
		gnssData.iTOW = timeOfWeek;
		if (data_ready_callback)
		{
			data_ready_callback();
		}
		gnssMsgReceived = 0;
	}
}

static void FS_GNSS_HandlePvt(void)
{
	gnssData.year = gnssPayload.navPvt.year;
	gnssData.month = gnssPayload.navPvt.month;
	gnssData.day = gnssPayload.navPvt.day;
	gnssData.hour = gnssPayload.navPvt.hour;
	gnssData.min = gnssPayload.navPvt.min;
	gnssData.sec = gnssPayload.navPvt.sec;
	gnssData.tAcc = gnssPayload.navPvt.tAcc;
	gnssData.nano = gnssPayload.navPvt.nano;
	gnssData.gpsFix = gnssPayload.navPvt.gpsFix;
	gnssData.numSV = gnssPayload.navPvt.numSV;
	gnssData.lon = gnssPayload.navPvt.lon;
	gnssData.lat = gnssPayload.navPvt.lat;
	gnssData.hMSL = gnssPayload.navPvt.hMSL;
	gnssData.hAcc = gnssPayload.navPvt.hAcc;
	gnssData.vAcc = gnssPayload.navPvt.vAcc;
	gnssData.velN = gnssPayload.navPvt.velN;
	gnssData.velE = gnssPayload.navPvt.velE;
	gnssData.velD = gnssPayload.navPvt.velD;
	gnssData.sAcc = gnssPayload.navPvt.sAcc;

	FS_GNSS_ReceiveMessage(UBX_MSG_PVT, gnssPayload.navPvt.iTOW);
}

static void FS_GNSS_HandleVelocity(void)
{
	gnssData.speed = gnssPayload.navVelNed.speed;
	gnssData.gSpeed = gnssPayload.navVelNed.gSpeed;
	gnssData.heading = gnssPayload.navVelNed.heading;

	FS_GNSS_ReceiveMessage(UBX_MSG_VELNED, gnssPayload.navVelNed.iTOW);
}

static void FS_GNSS_HandleTp(void)
{
	gnssTime.towMS = gnssPayload.timTp.towMS;
	gnssTime.week = gnssPayload.timTp.week;
	validTime = true;
}

static void FS_GNSS_HandleTm2(void)
{
	gnssInt.towMS = gnssPayload.timTm2.towMsR;
	gnssInt.week = gnssPayload.timTm2.wnR;

	if (int_ready_callback)
	{
		int_ready_callback();
	}
}

static void FS_GNSS_HandleMessage(void)
{
	switch (gnssMsgClass)
	{
	case UBX_NAV:
		switch (gnssMsgId)
		{
		case UBX_NAV_PVT:
			FS_GNSS_HandlePvt();
			break;
		case UBX_NAV_VELNED:
			FS_GNSS_HandleVelocity();
			break;
		}
		break;
	case UBX_TIM:
		switch (gnssMsgId)
		{
		case UBX_TIM_TP:
			FS_GNSS_HandleTp();
			break;
		case UBX_TIM_TM2:
			FS_GNSS_HandleTm2();
			break;
		}
		break;
	}
}

static void FS_GNSS_InitMessages(void)
{
	const FS_Config_Data_t *config = FS_Config_Get();
	const FS_State_Data_t *state = FS_State_Get();

	const ubxCfgMsg_t cfgMsg[] =
	{
		{UBX_NMEA, UBX_NMEA_GPGGA,  0},
		{UBX_NMEA, UBX_NMEA_GPGLL,  0},
		{UBX_NMEA, UBX_NMEA_GPGSA,  0},
		{UBX_NMEA, UBX_NMEA_GPGSV,  0},
		{UBX_NMEA, UBX_NMEA_GPRMC,  0},
		{UBX_NMEA, UBX_NMEA_GPVTG,  0},
		{UBX_NAV,  UBX_NAV_VELNED,  1},
		{UBX_NAV,  UBX_NAV_PVT,     1},
		{UBX_TIM,  UBX_TIM_TP,      1},
		{UBX_TIM,  UBX_TIM_TM2,     1},
		{UBX_SEC,  UBX_SEC_ECSIGN,  10}
	};

	const ubxCfgMsg_t cfgMsgRaw[] =
	{
		{UBX_MON,  UBX_MON_TXBUF,   1},
		{UBX_MON,  UBX_MON_SPAN,    1},
		{UBX_NAV,  UBX_NAV_SAT,     MAX(1, 1000 / config->rate)}
	};

	size_t i, n;

	const ubxCfgRate_t cfgRate =
	{
		.measRate   = config->rate, // Measurement rate (ms)
		.navRate    = 1,            // Navigation rate (cycles)
		.timeRef    = 0             // UTC time
	};

	const ubxCfgNav5_t cfgNav5 =
	{
		.mask       = 0x0001,   // Apply dynamic model settings
		.dynModel   = config->model
	};

	const ubxCfgTp5_t cfgTp5 =
	{
		.tpIdx             = 0,       // Time pulse selection (0 = TIMEPULSE, 1 = TIMEPULSE2)
		.version           = 1,       // Version
		.reserved1         = 0,       // Reserved
		.antCableDelay     = 50,      // Antenna cable delay (ns)
		.rfGroupDelay      = 0,       // RF group delay (ns)
		.freqPeriod        = 1000000, // Frequency or period time
		.pulseLenRatio     = 100000,  // Pulse length or duty cycle
		.userConfigDelay   = 0,       // User configurable time pulse delay
		.flags             = 0x73     // Configuration flags
	};

	ubxCfgValset_t cfgValset =
	{
		.version = 0x00,
		.layers = 0x01,
		.cfgData =
		{
			UBX_CFG_SEC_ECCFGSESSIONID0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			UBX_CFG_SEC_ECCFGSESSIONID1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			UBX_CFG_SEC_ECCFGSESSIONID2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
		}
	};

	#define SEND_MESSAGE(c,m,d)							\
		do {											\
			FS_GNSS_SendMessage(c,m,sizeof(d),&d);			\
		} while (!FS_GNSS_WaitForAck(c,m,GNSS_TIMEOUT));

	n = sizeof(cfgMsg) / sizeof(ubxCfgMsg_t);
	for (i = 0; i < n; ++i)
	{
		SEND_MESSAGE(UBX_CFG, UBX_CFG_MSG, cfgMsg[i]);
	}

	if (FS_Config_Get()->enable_raw)
	{
		n = sizeof(cfgMsgRaw) / sizeof(ubxCfgMsg_t);
		for (i = 0; i < n; ++i)
		{
			SEND_MESSAGE(UBX_CFG, UBX_CFG_MSG, cfgMsgRaw[i]);
		}
	}

	SEND_MESSAGE(UBX_CFG, UBX_CFG_RATE, cfgRate);
	SEND_MESSAGE(UBX_CFG, UBX_CFG_NAV5, cfgNav5);
	SEND_MESSAGE(UBX_CFG, UBX_CFG_TP5,  cfgTp5);

	// Set session ID
	for (i = 0; i < 4; ++i)
	{
		cfgValset.cfgData[8 + i]  = (state->device_id[0]  >> (8 * i)) & 0xff;
		cfgValset.cfgData[4 + i]  = (state->device_id[1]  >> (8 * i)) & 0xff;
		cfgValset.cfgData[20 + i] = (state->device_id[2]  >> (8 * i)) & 0xff;
		cfgValset.cfgData[16 + i] = (state->session_id[0] >> (8 * i)) & 0xff;
		cfgValset.cfgData[32 + i] = (state->session_id[1] >> (8 * i)) & 0xff;
		cfgValset.cfgData[28 + i] = (state->session_id[2] >> (8 * i)) & 0xff;
	}

	SEND_MESSAGE(UBX_CFG, UBX_CFG_VALSET, cfgValset);

	#undef SEND_MESSAGE
}

void FS_GNSS_Init(void)
{
	const ubxCfgPrt_t cfgPrt =
	{
		.portID       = 1,         // UART 1
		.reserved0    = 0,         // Reserved
		.txReady      = 0,         // no TX ready
		.mode         = 0x08d0,    // 8N1
		.baudRate     = GNSS_RATE, // Baud rate in bits/second
		.inProtoMask  = 0x0001,    // UBX protocol
		.outProtoMask = 0x0001,    // UBX protocol
		.flags        = 0,         // Flags bit mask
		.reserved5    = 0          // Reserved, set to 0
	};

	// Reset state
	gnssTimeOfWeek = 0;
	gnssMsgReceived = 0;
	validTime = false;

	updateCount = 0;
	updateTotalTime = 0;
	updateMaxTime = 0;
	updateLastCall = 0;
	updateMaxInterval = 0;
	bufferUsed = 0;

	// Set initialization flag
	gnss_is_initializing = true;

	do
	{
		while (huart1.gState == HAL_UART_STATE_BUSY_TX);

		// Stop DMA transfer
		if (HAL_UART_Abort(&huart1) != HAL_OK)
		{
			Error_Handler();
		}

		// Configure port baud rate
		huart1.Init.BaudRate = 38400;
		if (HAL_UART_Init(&huart1) != HAL_OK)
		{
			Error_Handler();
		}

		// Configure UBX baud rate
		FS_GNSS_SendMessage(UBX_CFG, UBX_CFG_PRT, sizeof(cfgPrt), &cfgPrt);
		while (huart1.gState == HAL_UART_STATE_BUSY_TX);

		// Configure port baud rate
		huart1.Init.BaudRate = GNSS_RATE;
		if (HAL_UART_Init(&huart1) != HAL_OK)
		{
			Error_Handler();
		}

		// Begin DMA transfer
		if (HAL_UART_Receive_DMA(&huart1, gnssRxData.whole, GNSS_RX_BUF_LEN) != HAL_OK)
		{
			Error_Handler();
		}

		// Reset state machine
		gnssRxIndex = 0;
		gnssState = st_sync_1;

		// Configure UBX baud rate
		FS_GNSS_SendMessage(UBX_CFG, UBX_CFG_PRT, sizeof(cfgPrt), &cfgPrt);
	}
	while (!FS_GNSS_WaitForAck(UBX_CFG, UBX_CFG_PRT, GNSS_TIMEOUT));

	// Configure UBX messages
	FS_GNSS_InitMessages();

	// Clear initialization flag
	gnss_is_initializing = false;

	// Initialize GNSS tasks
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_GNSS_UPDATE_ID, UTIL_SEQ_RFU, FS_GNSS_Update);
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_GNSS_LOG_ERROR_ID, UTIL_SEQ_RFU, FS_GNSS_LogError);

	// Initialize GNSS update timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_Repeated, FS_GNSS_Timer);
	HW_TS_Start(timer_id, GNSS_UPDATE_RATE);
}

void FS_GNSS_DeInit(void)
{
	// Stop DMA transfer
	HAL_UART_DMAStop(&huart1);

	// Delete GNSS update timer
	HW_TS_Delete(timer_id);

	// Add event log entries for timing info
	FS_Log_WriteEvent("----------");
	FS_Log_WriteEvent("%lu/%lu slots used in GNSS buffer", bufferUsed, GNSS_RX_BUF_LEN);
	FS_Log_WriteEvent("%lu ms average time spent in GNSS update task",
			(updateCount > 0) ? (updateTotalTime / updateCount) : 0);
	FS_Log_WriteEvent("%lu ms maximum time spent in GNSS update task", updateMaxTime);
	FS_Log_WriteEvent("%lu ms maximum time between calls to GNSS update task", updateMaxInterval);
}

void FS_GNSS_Start(void)
{
	// Enable EXTI pin
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_3);

	if (FS_Config_Get()->cold_start)
	{
		const ubxCfgRst_t cfgRst1 =
		{
			.navBbrMask = 0xffff,   // Cold start
			.resetMode  = 0x02      // Controlled software reset (GNSS only)
		};

		FS_GNSS_SendMessage(UBX_CFG, UBX_CFG_RST, sizeof(cfgRst1), &cfgRst1);
	}

	const ubxCfgRst_t cfgRst2 =
	{
		.navBbrMask = 0x0000,   // Hot start
		.resetMode  = 0x09      // Controlled GPS start
	};

	FS_GNSS_SendMessage(UBX_CFG, UBX_CFG_RST, sizeof(cfgRst2), &cfgRst2);
}

void FS_GNSS_Stop(void)
{
	const ubxCfgRst_t cfgRst =
	{
		.navBbrMask = 0x0000,   // Hot start
		.resetMode  = 0x08      // Controlled GPS stop
	};

	FS_GNSS_SendMessage(UBX_CFG, UBX_CFG_RST, sizeof(cfgRst), &cfgRst);
}

static void FS_GNSS_Timer(void)
{
	// Call update task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_GNSS_UPDATE_ID, CFG_SCH_PRIO_1);
}

static void FS_GNSS_Update(void)
{
	uint32_t msStart, msEnd;
	uint32_t writeIndex = GNSS_RX_BUF_LEN - huart1.hdmarx->Instance->CNDTR;

	msStart = HAL_GetTick();

	if (updateLastCall != 0)
	{
		updateMaxInterval = MAX(updateMaxInterval, msStart - updateLastCall);
	}
	updateLastCall = msStart;

	// Update buffer statistics
	bufferUsed = MAX(bufferUsed, (writeIndex - gnssRxIndex) % GNSS_RX_BUF_LEN);

	while (gnssRxIndex != writeIndex)
	{
		if (FS_GNSS_HandleByte(FS_GNSS_GetChar()))
		{
			FS_GNSS_HandleMessage();
		}
	}

	if (FS_Config_Get()->enable_raw)
	{
		while (writeIndex / GNSS_RAW_BUF_LEN != gnssRawIndex)
		{
			if (raw_ready_callback)
			{
				raw_ready_callback();
			}
			gnssRawIndex = (gnssRawIndex + 1) % (GNSS_RX_BUF_LEN / GNSS_RAW_BUF_LEN);
		}
	}

	++updateCount;

	msEnd = HAL_GetTick();
	updateTotalTime += msEnd - msStart;
	updateMaxTime = MAX(updateMaxTime, msEnd - msStart);
}

const FS_GNSS_Data_t *FS_GNSS_GetData(void)
{
	return &gnssData;
}

void FS_GNSS_Timepulse(void)
{
	gnssTime.time = HAL_GetTick();

	if (time_ready_callback)
	{
		time_ready_callback(validTime);
	}

	validTime = false;
}

const FS_GNSS_Time_t *FS_GNSS_GetTime(void)
{
	return &gnssTime;
}

const FS_GNSS_Raw_t *FS_GNSS_GetRaw(void)
{
	return &gnssRxData.split[gnssRawIndex];
}

const FS_GNSS_Int_t *FS_GNSS_GetInt(void)
{
	return &gnssInt;
}

void FS_GNSS_DataReady_SetCallback(void (*callback)(void))
{
	data_ready_callback = callback;
}

void FS_GNSS_TimeReady_SetCallback(void (*callback)(bool))
{
	time_ready_callback = callback;
}

void FS_GNSS_RawReady_SetCallback(void (*callback)(void))
{
	raw_ready_callback = callback;
}

void FS_GNSS_IntReady_SetCallback(void (*callback)(void))
{
	int_ready_callback = callback;
}
