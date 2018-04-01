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

	Odom_t GetOdomValue(){
		return odom->GetOdomValue();
	}

	int32_t EncoderLeftGetAbsoluteStep(){
		return odom->EncoderLeftGetAbsoluteStep();
	}

	int32_t EncoderRightGetAbsoluteStep(){
		return odom->EncoderRightGetAbsoluteStep();
	}

	double  EncoderLeftGetAbsoluteMM(){
		return odom->EncoderLeftGetAbsoluteMM();
	}

	double  EncoderRightGetAbsoluteMM(){
		return odom->EncoderRightGetAbsoluteMM();
	}

protected:
	Odometry *      odom;
	MotionControl * motionCtrl;
	Test *          test;
};


#endif /* ROBOTCORE_H_ */
