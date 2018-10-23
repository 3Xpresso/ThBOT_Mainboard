/*
 * MotionControl.h
 *
 *  Created on: 10 mars 2018
 *      Author: jpb
 */


#ifndef MOTIONCONTROL_H_
#define MOTIONCONTROL_H_

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "bsp/DcMotor.h"
#include "Odometry/Odometry.h"
#include <MotionControl/MotionMotor.h>
#include "thb-bsp.h"

#define ECH_PERIOD_MS    10

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

	Odom_t GetOdomValue(){
		return odom->GetOdomValue();
	}

	int32_t EncoderLeftGetAbsoluteStep(){
		return odom->EncoderLeftGetAbsoluteStep();
	}

	int32_t EncoderRightGetAbsoluteStep(){
		return odom->EncoderRightGetAbsoluteStep();
	}

	int32_t EncoderLeftGetAbsoluteStepFromDelta(){
		return odom->EncoderLeftGetAbsoluteStepFromDelta();
	}

	int32_t EncoderRightGetAbsoluteStepFromDelta(){
		return odom->EncoderRightGetAbsoluteStepFromDelta();
	}

	double EncoderLeftGetAbsoluteMM(){
		return odom->EncoderLeftGetAbsoluteMM();
	}

	double EncoderRightGetAbsoluteMM(){
		return odom->EncoderRightGetAbsoluteMM();
	}

	void SetMotionMotor(uint32_t Id, uint32_t Direction, uint32_t Percentage){
		switch(Id) {
		case MOTION_MOTOR_LEFT:
			motionMotorLeft->SetDirection(Direction);
			motionMotorLeft->SetPercentPower(Percentage);
			if (Percentage > 0)
				motorLeftIsRunning = true;
			else {
				motorLeftIsRunning = false;
				motionMotorLeft->SetPercentPower(Percentage);
			}
		break;
		case MOTION_MOTOR_RIGHT:
			motionMotorRight->SetDirection(Direction);
			motionMotorRight->SetPercentPower(Percentage);
			if (Percentage > 0)
				motorRightIsRunning = true;
			else {
				motorRightIsRunning = false;
				motionMotorLeft->SetPercentPower(Percentage);
			}
		break;
		}
	}

	void PidReload(void){
		motionMotorRight->PidReload();
		motionMotorLeft->PidReload();
	}
	void PrintStats(void){
		thb_StatsPrintResponse();
	}

	void ClearStats(void){
		thb_StatsInit();
	}

	void task(void);

	bool isRunning(void)
	{
		if (motorLeftIsRunning | motorRightIsRunning)
			return true;
		else
			return false;
	}

protected:

	RobotCore * Robocore;

	osThreadId motionTaskHandle;

	MotionMotor * motionMotorLeft;
	MotionMotor * motionMotorRight;
	Odometry    * odom;
	bool motorLeftIsRunning;
	bool motorRightIsRunning;
};

#endif /* MOTIONCONTROL_H_ */
