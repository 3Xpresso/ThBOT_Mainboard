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

	void SetMotionMotor(uint32_t Id, uint32_t Direction, uint32_t Percentage){
		switch(Id) {
		case MOTION_MOTOR_LEFT:
			motionMotorLeft->SetPercentPower(Percentage);
			motionMotorLeft->SetDirection(Direction);
		break;
		case MOTION_MOTOR_RIGHT:
			motionMotorRight->SetPercentPower(Percentage);
			motionMotorRight->SetDirection(Direction);
		break;
		}
	}

	void task(void);

protected:

	RobotCore * Robocore;

	osThreadId motionTaskHandle;

	DcMotor * motionMotorLeft;
	DcMotor * motionMotorRight;
};

#endif /* MOTIONCONTROL_H_ */
