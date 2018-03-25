/*
 * RobotCore.h
 *
 *  Created on: 20 mars 2018
 *      Author: jpb
 */

#ifndef ROBOTCORE_H_
#define ROBOTCORE_H_

#include "Odometry/Odometry.h"
#include "MotionControl/MotionControl.h"
#include "tests/Test.h"

#include "thb-bsp.h"

class RobotCore {
public:
	RobotCore();
	virtual ~RobotCore();

	void Init(void);
	void Task(void);

	uint32_t GetPercentPower(){
		return thb_param_GetPercentPower();
	}

protected:
	Odometry *      odom;
	MotionControl * motionCtrl;
	Test *          test;
};


#endif /* ROBOTCORE_H_ */
