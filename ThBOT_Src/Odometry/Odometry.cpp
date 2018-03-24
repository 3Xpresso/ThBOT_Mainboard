/*
 * Odometry.cpp
 *
 *  Created on: 10 mars 2018
 *      Author: jpb
 */

/* Inc/FreeRTOSConfig.h: #define INCLUDE_vTaskDelayUntil 1 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "thb-task.h"
#include "thb-tests.h"

#include "Odometry.h"
#include "ThBot_platform.h"

static void taskWrapper(const void* arg)
{
	Odometry* odom = const_cast<Odometry*>(static_cast<const Odometry*>(arg));

	odom->task();
}

Odometry::Odometry() {

	osThreadDef(odomTask, taskWrapper, PRIORITY_ODOMETRY, 0, 128);
	odomTaskHandle = osThreadCreate(osThread(odomTask), this);

	odomEncoderLeft  = new EncoderABZ(ENCODER_LEFT);
	odomEncoderRight = new EncoderABZ(ENCODER_RIGHT);
	Init(0, 0, 0.0);
}

void Odometry::Init(int32_t pos_x, int32_t pos_y, double angle)
{
	Pos_x = pos_x;
	Pos_y = pos_y;
	Angle = angle;
	previous_tick_count = xTaskGetTickCount();
}

Odometry::~Odometry() {
	// TODO Auto-generated destructor stub
}

void Odometry::task(void)
{
	TickType_t xLastWakeTime;
	uint32_t LoopCounter = 0;
	float PosX = 0.01;
	float PosY = 1040.01;
	float Teta = 137.24;
	float Speed = 0.78345;
	char buffer[24];

	printf("Start odometry task...\n");

	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		osDelay(10);
		double delta_l = odomEncoderLeft->GetDeltaMM();
		double delta_r = odomEncoderRight->GetDeltaMM();
		Update_value(delta_l, delta_r);

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
			//sprintf(TestResponse, "Problem printing float!!!");
			//thb_test_PrintOdometry(TestResponse);
		}
		LoopCounter++;

		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 ));
	}

}

// mise a jour par approximation de droite
void Odometry::Update_value(double delta_left_mm, double delta_right_mm)
{
	TickType_t current_tick_count = xTaskGetTickCount();
	TickType_t delta_tick = current_tick_count - previous_tick_count;
	previous_tick_count = current_tick_count;

	uint32_t delay_ms = delta_tick/portTICK_PERIOD_MS;
	if (!delay_ms)
	  return;

	double deplacement_mm = (delta_left_mm + delta_right_mm)/2;
	if (!deplacement_mm)
	  return;

	Speed = deplacement_mm / delay_ms;

	int32_t delta_x = deplacement_mm*cos(Angle);
	int32_t delta_y = deplacement_mm*sin(Angle);
	double delta_angle = (double)(delta_left_mm - delta_right_mm) / ENCODER_DISTANCE;

	Angle = Angle + delta_angle;
	Pos_x = Pos_x + delta_x;
	Pos_y = Pos_y + delta_y;
}
