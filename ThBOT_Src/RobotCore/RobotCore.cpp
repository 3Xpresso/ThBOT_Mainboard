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
#include "thb-tests.h"

//#include "Odometry.h"
#include "ThBot_platform.h"

#include "RobotCore.h"

RobotCore::RobotCore() {

	odom       = new Odometry();
	motionCtrl = new MotionControl();
}

RobotCore::~RobotCore() {
	// TODO Auto-generated destructor stub
}

