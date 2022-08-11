/*
 * gnss.c
 *
 *  Created on: Mar 3, 2020
 *      Author: Michael Cooper
 */

#include <stdbool.h>

#include "main.h"
#include "app_common.h"
#include "config.h"
#include "gnss.h"
#include "stm32_seq.h"

#define GNSS_RATE           230400	// Baud rate
#define GNSS_TIMEOUT        100		// ACK/NAK timeout (ms)

#define GNSS_UPDATE_MSEC    10
#define GNSS_UPDATE_RATE    (GNSS_UPDATE_MSEC*1000/CFG_TS_TICK_VAL)

#define GNSS_SAVED_LEN      8		// Buffer for stored GNSS messages

#define UBX_NUM_CHANNELS	72		// For MAX-M8
#define UBX_PAYLOAD_LEN		(8+12*UBX_NUM_CHANNELS) // Payload for single UBX message

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

#define UBX_MON             0x0a
#define UBX_MON_HW          0x09
#define UBX_MON_SPAN        0x31

#define UBX_TIM             0x0d
#define UBX_TIM_TP          0x01

#define UBX_NMEA            0xf0
#define UBX_NMEA_GPGGA      0x00
#define UBX_NMEA_GPGLL      0x01
#define UBX_NMEA_GPGSA      0x02
#define UBX_NMEA_GPGSV      0x03
#define UBX_NMEA_GPRMC      0x04
#define UBX_NMEA_GPVTG      0x05

#define UBX_MSG_POSLLH      0x01
#define UBX_MSG_PVT         0x02
#define UBX_MSG_VELNED      0x04
#define UBX_MSG_TIMEUTC     0x08
#define UBX_MSG_ALL         (UBX_MSG_POSLLH | UBX_MSG_PVT | UBX_MSG_VELNED | UBX_MSG_TIMEUTC)

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
} gnssPayload;

// Saved GNSS messages
static uint32_t gnssTimeOfWeek;
static uint8_t  gnssMsgReceived;
static bool validTime;

static FS_GNSS_Data_t gnssData;
static FS_GNSS_Time_t gnssTime;

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


// UART handle
extern UART_HandleTypeDef huart1;

static void FS_GNSS_Timer(void);
static void FS_GNSS_Update(void);

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->ErrorCode & HAL_UART_ERROR_ORE)
	{
		Error_Handler();
	}
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
		FS_GNSS_DataReady_Callback();
		gnssMsgReceived = 0;
	}
}

static void FS_GNSS_HandlePvt(void)
{
	gnssData.gpsFix = gnssPayload.navPvt.gpsFix;
	gnssData.numSV = gnssPayload.navPvt.numSV;

	FS_GNSS_ReceiveMessage(UBX_MSG_PVT, gnssPayload.navPvt.iTOW);
}

static void FS_GNSS_HandlePosition(void)
{
	gnssData.lon = gnssPayload.navPosLlh.lon;
	gnssData.lat = gnssPayload.navPosLlh.lat;
	gnssData.hMSL = gnssPayload.navPosLlh.hMSL;
	gnssData.hAcc = gnssPayload.navPosLlh.hAcc;
	gnssData.vAcc = gnssPayload.navPosLlh.vAcc;

	FS_GNSS_ReceiveMessage(UBX_MSG_POSLLH, gnssPayload.navPosLlh.iTOW);
}

static void FS_GNSS_HandleVelocity(void)
{
	gnssData.velN = gnssPayload.navVelNed.velN;
	gnssData.velE = gnssPayload.navVelNed.velE;
	gnssData.velD = gnssPayload.navVelNed.velD;
	gnssData.speed = gnssPayload.navVelNed.speed;
	gnssData.gSpeed = gnssPayload.navVelNed.gSpeed;
	gnssData.sAcc = gnssPayload.navVelNed.sAcc;

	FS_GNSS_ReceiveMessage(UBX_MSG_VELNED, gnssPayload.navVelNed.iTOW);
}

static void FS_GNSS_HandleTimeUTC(void)
{
	gnssData.tAcc = gnssPayload.navTimeUtc.tAcc;
	gnssData.nano = gnssPayload.navTimeUtc.nano;
	gnssData.year = gnssPayload.navTimeUtc.year;
	gnssData.month = gnssPayload.navTimeUtc.month;
	gnssData.day = gnssPayload.navTimeUtc.day;
	gnssData.hour = gnssPayload.navTimeUtc.hour;
	gnssData.min = gnssPayload.navTimeUtc.min;
	gnssData.sec = gnssPayload.navTimeUtc.sec;

	FS_GNSS_ReceiveMessage(UBX_MSG_TIMEUTC, gnssPayload.navTimeUtc.iTOW);
}

static void FS_GNSS_HandleTp(void)
{
	gnssTime.towMS = gnssPayload.timTp.towMS;
	gnssTime.week = gnssPayload.timTp.week;
	validTime = true;
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
		case UBX_NAV_POSLLH:
			FS_GNSS_HandlePosition();
			break;
		case UBX_NAV_VELNED:
			FS_GNSS_HandleVelocity();
			break;
		case UBX_NAV_TIMEUTC:
			FS_GNSS_HandleTimeUTC();
			break;
		}
		break;
	case UBX_TIM:
		switch (gnssMsgId)
		{
		case UBX_TIM_TP:
			FS_GNSS_HandleTp();
			break;
		}
		break;
	}
}

static void FS_GNSS_InitMessages(void)
{
	const FS_Config_Data_t *config = FS_Config_Get();

	const ubxCfgMsg_t cfgMsg[] =
	{
		{UBX_NMEA, UBX_NMEA_GPGGA,  0},
		{UBX_NMEA, UBX_NMEA_GPGLL,  0},
		{UBX_NMEA, UBX_NMEA_GPGSA,  0},
		{UBX_NMEA, UBX_NMEA_GPGSV,  0},
		{UBX_NMEA, UBX_NMEA_GPRMC,  0},
		{UBX_NMEA, UBX_NMEA_GPVTG,  0},
		{UBX_NAV,  UBX_NAV_POSLLH,  1},
		{UBX_NAV,  UBX_NAV_VELNED,  1},
		{UBX_NAV,  UBX_NAV_PVT,     1},
		{UBX_NAV,  UBX_NAV_TIMEUTC, 1},
		{UBX_TIM,  UBX_TIM_TP ,     1}
	};

	const ubxCfgMsg_t cfgMsgRaw[] =
	{
		{UBX_MON,  UBX_MON_HW,      1000 / config->rate},
		{UBX_MON,  UBX_MON_SPAN,    1000 / config->rate},
		{UBX_NAV,  UBX_NAV_SAT,     1000 / config->rate},
		{UBX_NAV,  UBX_NAV_STATUS,  1000 / config->rate}
	};

	size_t i, n;

	const ubxCfgRate_t cfgRate =
	{
		.measRate   = config->rate, // Measurement rate (ms)
		.navRate    = 1,        // Navigation rate (cycles)
		.timeRef    = 0         // UTC time
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
		.freqPeriodLock    = 1000000, // Frequency or period time when locked to GPS time
		.pulseLenRatio     = 0,       // Pulse length or duty cycle
		.pulseLenRatioLock = 100000,  // Pulse length or duty cycle when locked to GPS time
		.userConfigDelay   = 0,       // User configurable time pulse delay
		.flags             = 0x77     // Configuration flags
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

	// Initialize GNSS task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_GNSS_UPDATE_ID, UTIL_SEQ_RFU, FS_GNSS_Update);

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
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_GNSS_UPDATE_ID, CFG_SCH_PRIO_0);
}

static void FS_GNSS_Update(void)
{
	uint32_t writeIndex = GNSS_RX_BUF_LEN - huart1.hdmarx->Instance->CNDTR;

	while (gnssRxIndex != writeIndex)
	{
		if (FS_GNSS_HandleByte(FS_GNSS_GetChar()))
		{
			FS_GNSS_HandleMessage();
		}
	}

	if (FS_Config_Get()->enable_raw &&
			(writeIndex / GNSS_RAW_BUF_LEN != gnssRawIndex))
	{
		FS_GNSS_RawReady_Callback();
		gnssRawIndex = writeIndex / GNSS_RAW_BUF_LEN;
	}
}

const FS_GNSS_Data_t *FS_GNSS_GetData(void)
{
	return &gnssData;
}

__weak void FS_GNSS_DataReady_Callback(void)
{
  /* NOTE: This function should not be modified, when the callback is needed,
           the FS_GNSS_DataReady_Callback could be implemented in the user file
   */
}

void FS_GNSS_Timepulse(void)
{
	if (validTime)
	{
		gnssTime.time = HAL_GetTick();

		FS_GNSS_TimeReady_Callback();

		validTime = false;
	}
}

const FS_GNSS_Time_t *FS_GNSS_GetTime(void)
{
	return &gnssTime;
}

__weak void FS_GNSS_TimeReady_Callback(void)
{
  /* NOTE: This function should not be modified, when the callback is needed,
           the FS_GNSS_TimeReady_Callback could be implemented in the user file
   */
}

const FS_GNSS_Raw_t *FS_GNSS_GetRaw(void)
{
	return &gnssRxData.split[gnssRawIndex];
}

__weak void FS_GNSS_RawReady_Callback(void)
{
  /* NOTE: This function should not be modified, when the callback is needed,
           the FS_GNSS_RawReady_Callback could be implemented in the user file
   */
}
