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

#include "thb-task.h"

#include "MotionControl.h"

static void taskWrapper(const void* arg)
{
	MotionControl* motionCtrl = const_cast<MotionControl*>(static_cast<const MotionControl*>(arg));

	motionCtrl->task();
}

MotionControl::MotionControl() {

	osThreadDef(motionTask, taskWrapper, PRIORITY_MOTIONCTRL, 0, 128);
	motionTaskHandle = osThreadCreate(osThread(motionTask), this);

	Param1 = 1;
	Param2 = 2;
	printf("Motion control task...(%i - %i)\n", Param1, Param2);
}

MotionControl::~MotionControl() {
	// TODO Auto-generated destructor stub
}

void MotionControl::task(void)
{
	printf("Start motion control task...(%i - %i)\n", Param1, Param2);

	while(1)
	{
		osDelay(10);
	}
}
