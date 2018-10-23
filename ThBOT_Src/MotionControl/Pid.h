/*
 * Pid.h
 *
 *  Created on: 5 oct. 2018
 *      Author: jpb
 */

#ifndef MOTIONCONTROL_PID_H_
#define MOTIONCONTROL_PID_H_

#if 1
class Pid {
public:
	Pid(float Kp, float Ki, float Kd, float Max);
	virtual ~Pid();

	void  Reset();
	float Compute(float error, float dt);
	void  Reload(float Kp, float Ki, float Kd);

protected:
	float kp;
	float ki;
	float kd;
	float max_out;
	float max_integral;
	float integral;
	float lastError;
};
#else
class Pid {
public:
	Pid(double Ku, double Tu, double Te);
	virtual ~Pid();

	void Reset();
	double Compute(double e);

protected:

	double e0, e1, e2; //valeurs de l'entrée du correcteur à t, t-1, t-2
	double s0, s1, s2; //valeurs de la sortie du correcteur à t, t-1, t-2

	double alpha;
	double beta;
	double delta;

};
#endif

#endif /* MOTIONCONTROL_PID_H_ */
