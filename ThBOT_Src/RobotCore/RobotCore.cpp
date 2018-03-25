/*
 * RobotCore.cpp
 *
 *  Created on: 20 mars 2018
 *      Author: jpb
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "thb-task.h"

//#include "Odometry.h"
#include "ThBot_platform.h"

#include "RobotCore.h"

/***************************************************************/
#include "thb-fsm.h"

typedef struct Cmd
{
	uint32_t Mode;
	uint32_t State;
} t_cmd;

#define MAX_CMD          16

typedef struct CmdRec
{
	uint32_t RdIndex;
	uint32_t WrIndex;
	t_cmd st_Cmd[MAX_CMD];
} t_CmdRec;

static t_CmdRec st_CmdRec = {
	.RdIndex = 0,
	.WrIndex = 0,
	.st_Cmd = {
	},
};

static uint32_t Mode  = MODE_IDLE;
static uint32_t State = STATE_IDLE;
static uint32_t CounterLoop = 0;

static void idle_callback(uint32_t Evt);


static void idle_callback(uint32_t Evt)
{
    CounterLoop++;
    if (CounterLoop > 300)
    {
        printf("Mode %i - State Idle\n", (int)Mode);
        CounterLoop = 0;
    }
}

static uint32_t get_state(void)
{
	uint32_t State = STATE_IDLE;

   	if (st_CmdRec.RdIndex != st_CmdRec.WrIndex)
    {
    	st_CmdRec.RdIndex++;
    	if (st_CmdRec.RdIndex >= MAX_CMD)
    		st_CmdRec.RdIndex = 0;
    }

   	Mode  = st_CmdRec.st_Cmd[st_CmdRec.RdIndex].Mode;
   	State = st_CmdRec.st_Cmd[st_CmdRec.RdIndex].State;

   	return State;
}

void thb_fsm_ChangeModeState(uint32_t Mode, uint32_t State)
{
	st_CmdRec.WrIndex++;
	if (st_CmdRec.WrIndex >= MAX_CMD)
		st_CmdRec.WrIndex = 0;

	st_CmdRec.st_Cmd[st_CmdRec.WrIndex].Mode  = Mode;
	st_CmdRec.st_Cmd[st_CmdRec.WrIndex].State = State;
}
/***************************************************************/


RobotCore::RobotCore() {

	odom       = new Odometry();
	motionCtrl = new MotionControl();
	test       = new Test(this);
}

RobotCore::~RobotCore() {
	// TODO Auto-generated destructor stub
}

void RobotCore::Init(void)
{

}

void RobotCore::Task(void)
{
	thb_fsm_ChangeModeState(MODE_IDLE, STATE_IDLE);

    for(;;)
    {
    	osDelay(10);

    	State = get_state();

    	switch (Mode)
    	{
    		case MODE_IDLE :
    		{
    			idle_callback(State);
    		}break;
    		case MODE_COMPET :
    		{

    		}break;
    		case MODE_QUALIF :
    		{

    		}break;
    		case MODE_CALIBR :
    		{

    		}break;
    		case MODE_TESTS :
    		{
    			test->Run(State);
    		}break;
    	}
    }
}
