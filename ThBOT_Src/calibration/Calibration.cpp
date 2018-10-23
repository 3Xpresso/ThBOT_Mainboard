/*
 * Calibration.cpp
 *
 *  Created on: 2 oct. 2018
 *      Author: jpb
 */

#include <calibration/Calibration.h>

#include "thb-fsm.h"
#include "thb-bsp.h"

Calibration::Calibration(RobotCore * Rob) {
	Robocore = Rob;
}

Calibration::~Calibration() {
}

int Calibration::Run(uint32_t State)
{
	switch (State)
	{
    	case STATE_IDLE :
    	{
	    }break;
	}
	osDelay(2000);
	thb_fsm_ChangeModeState(MODE_CALIBR, STATE_IDLE);
	return 0;
}
