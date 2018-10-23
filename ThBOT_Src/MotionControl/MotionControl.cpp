/*
 * MotionControl.cpp
 *
 *  Created on: 10 mars 2018
 *      Author: jpb
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

#include "thb-task.h"

#include "MotionControl.h"

static void taskWrapper(const void* arg)
{
	MotionControl* motionCtrl = const_cast<MotionControl*>(static_cast<const MotionControl*>(arg));

	motionCtrl->task();
}

MotionControl::MotionControl(RobotCore * Rob) {

	Robocore = Rob;
	double Te = (double)ECH_PERIOD_MS/1000;

	osThreadDef(motionTask, taskWrapper, PRIORITY_MOTIONCTRL, 0, 1024);
	motionTaskHandle = osThreadCreate(osThread(motionTask), this);

	motionMotorLeft  = new MotionMotor(Rob, BSP_DCMOTOR_1, Te);
	motionMotorRight = new MotionMotor(Rob, BSP_DCMOTOR_2, Te);
	odom             = new Odometry(Rob);

	motorLeftIsRunning  = false;
	motorRightIsRunning = false;

	thb_StatsInit();
}

MotionControl::~MotionControl() {
}

void MotionControl::task(void)
{
	TickType_t xLastWakeTime;
	Wheel_t WheelVal;
	bool printStats = false;
	uint32_t statsIndex = 0;

	motionMotorLeft->SetPercentPower(0.0);
	motionMotorRight->SetPercentPower(0.0);

	printf("Start motion control task...\n");

	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		osDelay(ECH_PERIOD_MS);

		WheelVal = odom->UpdateValue();

		if (isRunning()){
			statsIndex = thb_StatsGetIndex();
			if (statsIndex > 5) {
				thb_StatsSetPidResLeft(motionMotorLeft->UpdateSpeed(WheelVal.SpeedLeft));
				thb_StatsSetPidResRight(motionMotorRight->UpdateSpeed(WheelVal.SpeedRight));
				thb_StatsSetSpeedLeft(WheelVal.SpeedLeft);
				thb_StatsSetSpeedRight(WheelVal.SpeedRight);
			}

			thb_StatsIncr();

			printStats = true;
			vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( ECH_PERIOD_MS ));
		}
		else {
			if (printStats == true) {
				printStats = false;
				thb_StatsPrintResponse();
				thb_StatsInit();
			}
			else {
				vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( ECH_PERIOD_MS ));
			}
		}
	}
}

