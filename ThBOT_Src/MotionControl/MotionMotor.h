/*
 * MotionMotor.h
 *
 *  Created on: 5 oct. 2018
 *      Author: jpb
 */

#ifndef MOTIONCONTROL_MOTIONMOTOR_H_
#define MOTIONCONTROL_MOTIONMOTOR_H_

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "bsp/DcMotor.h"
#include <MotionControl/Pid.h>

class RobotCore;

class MotionMotor {
public:
	MotionMotor(RobotCore * Rob, uint32_t id, double Te);
	virtual ~MotionMotor();

	void SetPercentPower(float Percentage);
	void SetDirection(uint32_t Direction);
	void SetSpeed(double expectedSpeed);
	double UpdateSpeed(double currentSpeed);
	void PidReload(void);

protected:
	RobotCore * Robocore;
	DcMotor * dcMotor;
	Pid * speedPid;
	double targetSpeed;
	uint32_t Direction;
	uint32_t MotionErrorDetector;
};

#endif /* MOTIONCONTROL_MOTIONMOTOR_H_ */
