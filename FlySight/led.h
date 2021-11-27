/*
 * led.h
 *
 *  Created on: Aug. 8, 2020
 *      Author: Michael
 */

#ifndef LED_H_
#define LED_H_

typedef enum
{
	FS_LED_RED,
	FS_LED_GREEN
} FS_LED_Colour_t;

void FS_LED_On(void);
void FS_LED_Off(void);
void FS_LED_SetColour(FS_LED_Colour_t colour);

#endif /* LED_H_ */
