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

enum
{
	BACKWARD,
	FORWARD,
};

class DcMotor {
public:
	DcMotor(uint32_t id);
	virtual ~DcMotor();

	void SetSpeed(double expectedSpeed);
	void SetPercentPower(float Percentage);
	void SetDirection(uint32_t Direction);
	void UpdatePercentPower(float DeltaPercentage);

protected:
	uint32_t id;
	float    Percentage;
};

#endif /* BSP_DCMOTOR_H_ */
