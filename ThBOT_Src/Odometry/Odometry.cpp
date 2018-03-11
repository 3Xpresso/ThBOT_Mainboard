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

#include "thb-task.h"
#include "Odometry.h"

static void taskWrapper(const void* arg)
{
	Odometry* odom = const_cast<Odometry*>(static_cast<const Odometry*>(arg));

	odom->task();
}


Odometry::Odometry() {

	osThreadDef(odomTask, taskWrapper, PRIORITY_ODOMETRY, 0, 128);
	odomTaskHandle = osThreadCreate(osThread(odomTask), this);

	//odomEncoder[ODOM_ENCODER_LEFT]  ENCODER_1;
	//odomEncoder[ODOM_ENCODER_RIGHT] ENCODER_2;
}

Odometry::~Odometry() {
	// TODO Auto-generated destructor stub
}

void Odometry::task(void)
{
	printf("Start odometry task...\n");

	while(1)
	{
		osDelay(10);
	}
}
