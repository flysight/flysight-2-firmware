/*
 * mode.h
 *
 *  Created on: Jan 23, 2020
 *      Author: Michael Cooper
 */

#ifndef MODE_H_
#define MODE_H_

typedef enum
{
	FS_MODE_EVENT_BUTTON_PRESSED,
	FS_MODE_EVENT_BUTTON_RELEASED,
	FS_MODE_EVENT_TIMER,
	FS_MODE_EVENT_VBUS_HIGH,
	FS_MODE_EVENT_VBUS_LOW
} FS_Mode_Event_t;

typedef enum
{
	FS_MODE_STATE_SLEEP,
	FS_MODE_STATE_ACTIVE,
	FS_MODE_STATE_CONFIG,
	FS_MODE_STATE_USB,

	// Number of modes
	FS_MODE_STATE_COUNT
} FS_Mode_State_t;

void FS_Mode_Init(void);
void FS_Mode_PushQueue(FS_Mode_Event_t event);
FS_Mode_State_t FS_Mode_State(void);

#endif /* MODE_H_ */
