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

#include "DcMotor.h"

#ifndef MOTIONCONTROL_MOTIONCONTROL_H_
#define MOTIONCONTROL_MOTIONCONTROL_H_

enum
{
	MOTION_MOTOR_LEFT = 0,
	MOTION_MOTOR_RIGHT,
	MOTION_MOTOR_MAX,
};

class MotionControl {
public:
	MotionControl();
	virtual ~MotionControl();

	void task(void);

protected:

	osThreadId motionTaskHandle;

	DcMotor motionMotor[MOTION_MOTOR_MAX];
};

#endif /* MOTIONCONTROL_MOTIONCONTROL_H_ */
