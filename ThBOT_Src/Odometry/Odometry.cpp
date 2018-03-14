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
#include "thb-fsm.h"
#include "thb-bsp.h"

#include "Odometry.h"

static char SendResponse[64];
static char TestResponse[48];

static void taskWrapper(const void* arg)
{
	Odometry* odom = const_cast<Odometry*>(static_cast<const Odometry*>(arg));

	odom->task();
}

static void test_send_reponse(uint32_t State, char * pu8_Buff)
{
	sprintf(SendResponse, "RSP:%2.2u:%2.2u:%s\n", MODE_TESTS, (unsigned int)State, pu8_Buff);
	//printf("Response [%i] : %s", strlen(SendResponse), SendResponse);
	thb_UART5_SendData(SendResponse, strlen(SendResponse));
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
	uint32_t Mode;
	uint32_t State;
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
			thb_fsm_GetModeState(&Mode, &State);

			//if ((Mode == MODE_TESTS) && (State == STATE_ODOM_PARAMS))
			if ((Mode == MODE_IDLE) && (State == STATE_IDLE))
			{
				//printf("PosX=%10.5f PosY=%10.5f Teta=%10.5f Speed=%10.5f\n",
				//						PosX, PosY,
				//						Teta, Speed);

				//sprintf(TestResponse, "%10.5f:%10.5f:%10.5f:%10.5f:",
				//		PosX, PosY,
				//		Teta, Speed);
				//test_send_reponse(STATE_ODOM_PARAMS, TestResponse);
				printf("%s\n", "Problem printing float!!!");
			}
		}
		LoopCounter++;
	}
}
