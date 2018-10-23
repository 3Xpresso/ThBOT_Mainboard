/*
 * thb-param.c
 *
 *  Created on: 16 mars 2018
 *      Author: jpb
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <stdlib.h>

#include "thb-param.h"

static uint32_t DefaultPercentage = 10;
static float PidSpeedKp = 5.0;
static float PidSpeedKi = 0.025;
static float PidSpeedKd = 0.0;

void thb_param_SetParameter(uint32_t ParamId, char* ParamValue, uint32_t ValueLen)
{
	uint32_t uint32_ParamVal;

	switch(ParamId)
	{
		case PARAM_PWM :
		{
			uint32_ParamVal = atoi(ParamValue);
			if (uint32_ParamVal <= 100)
				DefaultPercentage = uint32_ParamVal;
		}break;
	}
}
void thb_param_SetPercentPower(uint32_t Percentage)
{
	if (Percentage <= 100)
		DefaultPercentage = Percentage;
	else
		DefaultPercentage = 100;
}

uint32_t thb_param_GetPercentPower(void)
{
	return DefaultPercentage;
}

float thb_param_GetPidSpeedKp(void){
	return PidSpeedKp;
}

float thb_param_GetPidSpeedKd(void){
	return PidSpeedKd;
}

float thb_param_GetPidSpeedKi(void){
	return PidSpeedKi;
}

void thb_param_SetPidSpeedKp(float Val){
	printf("PARAM KP set to %7.3f\n", Val);
	PidSpeedKp = Val;
}

void thb_param_SetPidSpeedKd(float Val){
	printf("PARAM KD set to %7.3f\n", Val);
	PidSpeedKd = Val;
}

void thb_param_SetPidSpeedKi(float Val){
	printf("PARAM KI set to %7.3f\n", Val);
	PidSpeedKi = Val;
}
