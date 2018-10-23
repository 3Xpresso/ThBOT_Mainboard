/*
 * Calibration.h
 *
 *  Created on: 2 oct. 2018
 *      Author: jpb
 */

#ifndef CALIBRATION_CALIBRATION_H_
#define CALIBRATION_CALIBRATION_H_

#include "FreeRTOS.h"
#include "cmsis_os.h"

class RobotCore;

class Calibration {
public:
	Calibration(RobotCore * Rob);
	virtual ~Calibration();

	int Run(uint32_t Evt);

protected:
	RobotCore * Robocore;
};


#endif /* CALIBRATION_CALIBRATION_H_ */
