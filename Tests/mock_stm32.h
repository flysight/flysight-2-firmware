// tests/mock_stm32.h
#ifndef MOCK_STM32_H
#define MOCK_STM32_H

#include <stdint.h>
#include <stdlib.h>

// Mock your GNSS structure (copy from your actual header)
typedef struct {
    uint32_t iTOW;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    int32_t  nano;
    int32_t  lon;
    int32_t  lat;
    int32_t  hMSL;
    int32_t  velN;
    int32_t  velE;
    int32_t  velD;
    int32_t  speed;
    int32_t  gSpeed;
    int32_t  heading;
    uint32_t tAcc;
    uint32_t hAcc;
    uint32_t vAcc;
    uint32_t sAcc;
    uint8_t  gpsFix;
    uint8_t  numSV;
} FS_GNSS_Data_t;

#endif
