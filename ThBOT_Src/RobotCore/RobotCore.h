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

class RobotCore {
public:
	RobotCore();
	virtual ~RobotCore();

	void Init(void);
	void Task(void);

protected:
	Odometry * odom;
	MotionControl * motionCtrl;
};


#endif /* ROBOTCORE_H_ */
