/*
 * MotionMotor.cpp
 *
 *  Created on: 5 oct. 2018
 *      Author: jpb
 */

#include "thb-bsp.h"

#include "RobotCore/RobotCore.h"
#include <MotionControl/MotionMotor.h>

MotionMotor::MotionMotor(RobotCore * Rob, uint32_t id, double Te) {

	Robocore = Rob;
	float PidSpeedKp = thb_param_GetPidSpeedKp();
	float PidSpeedKd = thb_param_GetPidSpeedKd();
	float PidSpeedKi = thb_param_GetPidSpeedKi();

	dcMotor     = new DcMotor(id);
	speedPid    = new Pid(PidSpeedKp, PidSpeedKd, PidSpeedKi, 10000.0);
	targetSpeed = 0;
	Direction   = FORWARD;
	MotionErrorDetector = 0;
}

MotionMotor::~MotionMotor() {
}

void MotionMotor::SetPercentPower(float Percentage){

	dcMotor->SetPercentPower(Percentage);
	if (Percentage == 0) {
		targetSpeed = 0;
		MotionErrorDetector = 0;
		speedPid->Reset();
	}
	else {
		targetSpeed = 210;
		dcMotor->SetSpeed(targetSpeed);
	}
}

void MotionMotor::SetDirection(uint32_t Direction){

	dcMotor->SetDirection(Direction);
	this->Direction = Direction;
}

void MotionMotor::SetSpeed(double expectedSpeed){

	targetSpeed = expectedSpeed;
	dcMotor->SetSpeed(targetSpeed);
}

double MotionMotor::UpdateSpeed(double currentSpeed){
	double error;

	if (this->Direction == BACKWARD)
		error = targetSpeed + currentSpeed;
	else
		error = targetSpeed - currentSpeed;

	float Correction = speedPid->Compute(error, 0.010)/100;

	MotionErrorDetector++;
	if ((currentSpeed == 0)&&(MotionErrorDetector>= 20)){
		Robocore->SetError(ERROR_MOTION);
		return Correction;
	}

	// WARNING : overrun protection!!!
	if (Correction <= -10) {
		Correction = -10;
	} else if (Correction >= 20) {
		Correction = 20;
	}
	dcMotor->UpdatePercentPower(Correction);
	dcMotor->SetSpeed(targetSpeed);

	return Correction;
}

void MotionMotor::PidReload(void)
{
	float PidSpeedKp = thb_param_GetPidSpeedKp();
	float PidSpeedKd = thb_param_GetPidSpeedKd();
	float PidSpeedKi = thb_param_GetPidSpeedKi();

	speedPid->Reload(PidSpeedKp, PidSpeedKd, PidSpeedKi);
}
