/*
 * time.h
 *
 *  Created on: Sep. 2, 2023
 *      Author: Michael
 */

#ifndef TIME_H_
#define TIME_H_

#include <stdint.h>

uint32_t mk_gmtime(uint16_t year, uint8_t mon, uint8_t mday, uint8_t hour, uint8_t min, uint8_t sec);
void gmtime_r(const uint32_t timer, uint16_t *year, uint8_t *mon, uint8_t *mday, uint8_t *hour, uint8_t *min, uint8_t *sec);

#endif /* TIME_H_ */
