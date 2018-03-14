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

#include "EncoderABZ.h"

#ifndef ODOMETRY_ODOMETRY_H_
#define ODOMETRY_ODOMETRY_H_

enum
{
	ODOM_ENCODER_LEFT = 0,
	ODOM_ENCODER_RIGHT,
	ODOM_ENCODER_MAX,
};

class Odometry {
public:
	Odometry();
	virtual ~Odometry();

	void task();

protected:

	osThreadId odomTaskHandle;

	EncoderABZ * odomEncoderLeft;
	EncoderABZ * odomEncoderRight;
};

#endif /* ODOMETRY_ODOMETRY_H_ */
