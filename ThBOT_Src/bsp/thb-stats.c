/*
 * thb-stats.c
 *
 *  Created on: 11 oct. 2018
 *      Author: jpb
 */

#include "FreeRTOS.h"
#include "cmsis_os.h"

#include <string.h>

#include "thb-param.h"
#include "thb-stats.h"

#define MAX_STATS_ELEMENTS        500
static uint32_t statsIndex;

typedef struct{
  double TargetSpeedLeft;
  double TargetSpeedRight;
  double SpeedLeft;
  double SpeedRight;
  double PidResLeft;
  double PidResRight;
  uint32_t PwmLeft;
  uint32_t PwmRight;
}WheelStat_t;

static WheelStat_t WheelValStats[MAX_STATS_ELEMENTS];

void thb_StatsInit(void){
	statsIndex = 0;
	memset(WheelValStats, 0, sizeof(WheelStat_t)*MAX_STATS_ELEMENTS);
}

uint32_t thb_StatsGetIndex(void) {
	return statsIndex;
}

void thb_StatsSetTargetSpeedLeft(double TargetSpeedLeft) {
	WheelValStats[statsIndex].TargetSpeedLeft = TargetSpeedLeft;
}

void thb_StatsSetTargetSpeedRight(double TargetSpeedRight) {
	WheelValStats[statsIndex].TargetSpeedRight = TargetSpeedRight;
}

void thb_StatsSetSpeedLeft(double SpeedLeft) {
	WheelValStats[statsIndex].SpeedLeft = SpeedLeft;
}

void thb_StatsSetSpeedRight(double SpeedRight) {
	WheelValStats[statsIndex].SpeedRight = SpeedRight;
}

void thb_StatsSetPidResLeft(double PidResLeft) {
	WheelValStats[statsIndex].PidResLeft = PidResLeft;
}

void thb_StatsSetPidResRight(double PidResRight) {
	WheelValStats[statsIndex].PidResRight = PidResRight;
}

void thb_StatsSetPwmLeft(uint32_t PwmVal) {
	WheelValStats[statsIndex].PwmLeft = PwmVal;
}

void thb_StatsSetPwmRight(uint32_t PwmVal) {
	WheelValStats[statsIndex].PwmRight = PwmVal;
}

void thb_StatsIncr(void){
	statsIndex++;
	if (statsIndex >= MAX_STATS_ELEMENTS)
		statsIndex = 0;
}

void thb_StatsPrint(void) {
	uint32_t Index;
	float PidSpeedKp = thb_param_GetPidSpeedKp();
	float PidSpeedKd = thb_param_GetPidSpeedKd();
	float PidSpeedKi = thb_param_GetPidSpeedKi();

	printf("=============================================== \n");
	printf("[%3lu] Statistics Response :\n", statsIndex);
	printf("=> Kp = %7.3f - Kd = %7.3f _ Ki = %7.3f :\n", PidSpeedKp, PidSpeedKd, PidSpeedKi);
	osDelay(30);
	for (Index=0; Index < MAX_STATS_ELEMENTS; Index++) {

		printf("%3li;%4.2f;%4.2f; ;%4.2f;%4.2f\n", Index,
				WheelValStats[Index].TargetSpeedLeft,
				WheelValStats[Index].SpeedLeft,
				WheelValStats[Index].TargetSpeedRight,
				WheelValStats[Index].SpeedRight);
		if (Index > statsIndex)
			break;
		osDelay(50);
	}
	printf("=============================================== \n");
}

void thb_StatsPrintResponse(void) {
	uint32_t Index;
	float PidSpeedKp = thb_param_GetPidSpeedKp();
	float PidSpeedKd = thb_param_GetPidSpeedKd();
	float PidSpeedKi = thb_param_GetPidSpeedKi();

	printf("=============================================== \n");
	printf("[%03lu] Statistics :\n", statsIndex);
	printf("=> Kp = %7.3f - Kd = %7.3f - Ki = %7.3f :\n", PidSpeedKp, PidSpeedKd, PidSpeedKi);
	for (Index=0; Index < MAX_STATS_ELEMENTS; Index++) {

		printf("%03li;%07.2f;%07.2f;%08.3f;%04lu; ;%07.2f;%07.2f;%08.3f;%04lu\n", Index,
				WheelValStats[Index].TargetSpeedLeft,
				WheelValStats[Index].SpeedLeft,
				WheelValStats[Index].PidResLeft,
				WheelValStats[Index].PwmLeft,
				WheelValStats[Index].TargetSpeedRight,
				WheelValStats[Index].SpeedRight,
				WheelValStats[Index].PidResRight,
				WheelValStats[Index].PwmRight);
		if (Index > statsIndex)
			break;
		osDelay(20);
	}
	printf("=============================================== \n");
}
