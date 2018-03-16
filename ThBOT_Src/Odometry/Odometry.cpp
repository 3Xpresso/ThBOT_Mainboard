/*
 * Odometry.cpp
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
#include "thb-tests.h"

#include "Odometry.h"

static char TestResponse[48];

static void taskWrapper(const void* arg)
{
	Odometry* odom = const_cast<Odometry*>(static_cast<const Odometry*>(arg));

	odom->task();
}

Odometry::Odometry() {

	osThreadDef(odomTask, taskWrapper, PRIORITY_ODOMETRY, 0, 128);
	odomTaskHandle = osThreadCreate(osThread(odomTask), this);

	odomEncoderLeft  = new EncoderABZ(ENCODER_1);
	odomEncoderRight = new EncoderABZ(ENCODER_2);
}

Odometry::~Odometry() {
	// TODO Auto-generated destructor stub
}

void Odometry::task(void)
{
	uint32_t LoopCounter = 0;
	float PosX = 0.01;
	float PosY = 1040.01;
	float Teta = 137.24;
	float Speed = 0.78345;

	printf("Start odometry task...\n");

	while(1)
	{
		osDelay(10);
		odomEncoderLeft->GetDeltaStep();
		odomEncoderRight->GetDeltaStep();

		// Print each 2s
		if (((LoopCounter /200) != 0) && ((LoopCounter%200) == 0))
		{
			//printf("PosX=%10.5f PosY=%10.5f Teta=%10.5f Speed=%10.5f\n",
			//						PosX, PosY,
			//						Teta, Speed);

			//sprintf(TestResponse, "%10.5f:%10.5f:%10.5f:%10.5f:",
			//		PosX, PosY,
			//		Teta, Speed);
			sprintf(TestResponse, "Problem printing float!!!");
			thb_test_PrintOdometry(TestResponse);
		}
		LoopCounter++;
	}
}
