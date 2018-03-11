/*
 * DcMotor.h
 *
 *  Created on: 11 mars 2018
 *      Author: jpb
 */

#ifndef BSP_DCMOTOR_H_
#define BSP_DCMOTOR_H_

enum
{
	BSP_DCMOTOR_1 = 0,
	BSP_DCMOTOR_2,
	BSP_DCMOTOR_MAX,
};

class DcMotor {
public:
	DcMotor();
	virtual ~DcMotor();
};

#endif /* BSP_DCMOTOR_H_ */
