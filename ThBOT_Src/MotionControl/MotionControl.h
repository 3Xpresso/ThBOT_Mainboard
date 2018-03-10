/*
 * MotionControl.h
 *
 *  Created on: 10 mars 2018
 *      Author: jpb
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#ifndef MOTIONCONTROL_MOTIONCONTROL_H_
#define MOTIONCONTROL_MOTIONCONTROL_H_

class MotionControl {
public:
	MotionControl();
	virtual ~MotionControl();

	void task(void);

	uint32_t   Param1;

protected:

	osThreadId motionTaskHandle;
	uint32_t   Param2;
};

#endif /* MOTIONCONTROL_MOTIONCONTROL_H_ */
