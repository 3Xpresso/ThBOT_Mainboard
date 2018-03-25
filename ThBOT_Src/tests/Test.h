/*
 * Test.h
 *
 *  Created on: 25 mars 2018
 *      Author: jpb
 */

#ifndef TEST_H_
#define TEST_H_

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

class RobotCore;

class Test {
public:
	Test(RobotCore * Rob);
	virtual ~Test();

	int Run(uint32_t Evt);

protected:
	RobotCore * Robocore;
};


#endif /* TEST_H_ */
