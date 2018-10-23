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
#include "calibration/Calibration.h"

#include "thb-bsp.h"

#define CMD_MAX_LEN  64

enum
{
	ERROR_NONE,
	ERROR_MOTION,
};

class RobotCore {
public:
	RobotCore();
	virtual ~RobotCore();

	void Init(void);
	void Task(void);

	void SetError(uint32_t Error);

	uint32_t GetPercentPower(){
		return thb_param_GetPercentPower();
	}

	void SetMotionMotor(uint32_t Id, uint32_t Direction, uint32_t Percentage){
		return motionCtrl->SetMotionMotor(Id, Direction, Percentage);
	}

	Odom_t GetOdomValue(){
		return motionCtrl->GetOdomValue();
	}

	int32_t EncoderLeftGetAbsoluteStep(){
		return motionCtrl->EncoderLeftGetAbsoluteStep();
	}

	int32_t EncoderRightGetAbsoluteStep(){
		return motionCtrl->EncoderRightGetAbsoluteStep();
	}

	double  EncoderLeftGetAbsoluteMM(){
		return motionCtrl->EncoderLeftGetAbsoluteMM();
	}

	double  EncoderRightGetAbsoluteMM(){
		return motionCtrl->EncoderRightGetAbsoluteMM();
	}

	int32_t EncoderLeftAbsoluteStepFromDelta(){
		return motionCtrl->EncoderLeftGetAbsoluteStepFromDelta();
	}

	int32_t EncoderRightGetAbsoluteStepFromDelta(){
		return motionCtrl->EncoderRightGetAbsoluteStepFromDelta();
	}

	void IncrStatIndex(){
		statsIndex++;
	}

protected:
	MotionControl * motionCtrl;
	Test *          test;
	Calibration *	calibr;
	uint32_t        statsIndex;
	uint32_t        Error;

	uint32_t		CmdIndex;
	char CmdArray[CMD_MAX_LEN];

	void ManageCmd(void);
};


#endif /* ROBOTCORE_H_ */
