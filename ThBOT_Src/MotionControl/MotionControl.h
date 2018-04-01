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

#include "bsp/DcMotor.h"

#ifndef MOTIONCONTROL_H_
#define MOTIONCONTROL_H_

enum
{
	MOTION_MOTOR_LEFT = 0,
	MOTION_MOTOR_RIGHT,
	MOTION_MOTOR_MAX,
};

class RobotCore;

class MotionControl {
public:
	MotionControl(RobotCore * Rob);
	virtual ~MotionControl();

	void task(void);

protected:

	RobotCore * Robocore;

	osThreadId motionTaskHandle;

	DcMotor * motionMotorLeft;
	DcMotor * motionMotorRight;
};

#endif /* MOTIONCONTROL_H_ */
