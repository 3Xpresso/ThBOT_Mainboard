/*
 * Odometry.h
 *
 *  Created on: 10 mars 2018
 *      Author: jpb
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#ifndef ODOMETRY_ODOMETRY_H_
#define ODOMETRY_ODOMETRY_H_

class Odometry {
public:
	Odometry();
	virtual ~Odometry();

	void task();

protected:

	osThreadId odomTaskHandle;
};

#endif /* ODOMETRY_ODOMETRY_H_ */
