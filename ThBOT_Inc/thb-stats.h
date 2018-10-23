/*
 * thb-stats.h
 *
 *  Created on: 11 oct. 2018
 *      Author: jpb
 */

#ifndef THB_STATS_H_
#define THB_STATS_H_

extern void thb_StatsInit(void);
extern uint32_t thb_StatsGetIndex(void);
extern void thb_StatsIncr(void);

extern void thb_StatsSetTargetSpeedLeft(double TargetSpeedLeft);
extern void thb_StatsSetTargetSpeedRight(double TargetSpeedRight);
extern void thb_StatsSetSpeedLeft(double SpeedLeft);
extern void thb_StatsSetSpeedRight(double SpeedRight);
extern void thb_StatsSetPidResLeft(double PidResLeft);
extern void thb_StatsSetPidResRight(double PidResRight);

extern void thb_StatsSetPwmLeft(uint32_t PwmVal);
extern void thb_StatsSetPwmRight(uint32_t PwmVal);

extern void thb_StatsPrintResponse(void);

#endif /* THB_STATS_H_ */
