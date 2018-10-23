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

	//odom       = new Odometry(this);
	motionCtrl = new MotionControl(this);
	test       = new Test(this);
	calibr     = new Calibration(this);
	statsIndex = 0;
	Error      = ERROR_NONE;
	CmdIndex   = 0;
}

RobotCore::~RobotCore() {
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
    			calibr->Run(State);
    		}break;
    		case MODE_TESTS :
    		{
    			test->Run(State);
    		}break;
    	}

    	ManageCmd();
    }
}

void RobotCore::SetError(uint32_t Error){
	thb_fsm_ChangeModeState(MODE_IDLE, STATE_IDLE);

	SetMotionMotor(MOTION_MOTOR_LEFT, BACKWARD, 0);
	SetMotionMotor(MOTION_MOTOR_RIGHT, FORWARD, 0);
	printf("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	printf("Error detected : %lu", Error);
	printf("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
}

void RobotCore::ManageCmd(void){

	CmdArray[CmdIndex] = getchar();
	if (CmdArray[CmdIndex] != '\0'){
		if (CmdArray[CmdIndex] == '\r') {
			CmdArray[CmdIndex] = '\0';
			printf("CMD[%i] : %s\n", CmdIndex, CmdArray);

			// PARAM:PID:SET:KP:4.0
			// PARAM:PID:SET:KI:4.0
			if (strncmp("PARAM:",CmdArray, strlen("PARAM:")) == 0 ) {
				if (strncmp("PID:",&CmdArray[6], strlen("PID:")) == 0 ) {
					if (strncmp("SET:",&CmdArray[6+4], strlen("SET:")) == 0 ) {
						if (strncmp("KP:",&CmdArray[6+4+4], strlen("KP:")) == 0 ) {
							float Value;

							Value = strtod(&CmdArray[6+4+4+3], NULL);
							thb_param_SetPidSpeedKp(Value);
						} else if (strncmp("KI:",&CmdArray[6+4+4], strlen("KI:")) == 0 ) {
							float Value;

							Value = strtod(&CmdArray[6+4+4+3], NULL);
							thb_param_SetPidSpeedKi(Value);
						} else if (strncmp("KD:",&CmdArray[6+4+4], strlen("KD:")) == 0 ) {
							float Value;

							Value = strtod(&CmdArray[6+4+4+3], NULL);
							thb_param_SetPidSpeedKd(Value);
						}
					}
					motionCtrl->PidReload();
				}
			} else if (strncmp("MOTION:",CmdArray, strlen("MOTION:")) == 0 ) {
				// MOTION:STAT:CLEAR
				// MOTION:STAT:PRINT
				if (strncmp("STAT:",&CmdArray[7], strlen("STAT:")) == 0 ) {
					if (strncmp("CLEAR",&CmdArray[7+5], strlen("CLEAR")) == 0 ) {
						printf("OK : %s\n", &CmdArray[7+5]);
						motionCtrl->ClearStats();

					} else if (strncmp("PRINT",&CmdArray[7+5], strlen("PRINT")) == 0 ) {
						printf("OK : %s\n", &CmdArray[7+5]);
						motionCtrl->PrintStats();
					}
				}
			}

			memset(CmdArray, 0, CMD_MAX_LEN);
			CmdIndex = 0;
		} else {
			CmdIndex++;
			if (CmdIndex>= CMD_MAX_LEN)
				CmdIndex = 0;
		}
	}

	if (CmdIndex > 0)
		CounterLoop = 0;
}
