/*
 * Pid.cpp
 *
 *  Created on: 5 oct. 2018
 *      Author: jpb
 */

#include <stdio.h>
#include <MotionControl/Pid.h>

#if 1

Pid::Pid(float Kp, float Ki, float Kd, float Max)
{
	kp = Kp;
	ki = Ki;
	kd = Kd;
	max_out = Max;
	max_integral = Max;

	integral = 0;
	lastError = 0;
}

Pid::~Pid() {
}

void Pid::Reload(float Kp, float Ki, float Kd)
{
	kp = Kp;
	ki = Ki;
	kd = Kd;
}

float Pid::Compute(float error, float dt)
{
	float out;

	float derivate = (error - lastError) / dt;
	lastError = error;
	integral += error * dt;

	// saturation de l'integrale
	if(max_integral)
	{
		if(integral > max_integral)
		{
			integral = max_integral;
		}
		else if(integral < -max_integral)
		{
			integral = -max_integral;
		}
	}

	// calcul du PID
	out = kp * error + ki * integral + kd * derivate;

	// saturation de la sortie
	if(max_out)
	{
		if(out > max_out)
		{
			out = max_out;
		}
		else if(out < -max_out)
		{
			out = -max_out;
		}
	}

	return out;
}

void Pid::Reset()
{
	integral = 0;
	lastError = 0;
}

#else
Pid::Pid(double Ku, double Tu, double Te) {

    //Réglage de Ziegler Nichols classique
    double Kp = 0.6 * Ku;
    double Ti = 0.5 * Tu;
    double Td = 0.125 * Tu;

    //Réglage de Ziegler Nichols sans dépassements : un tout petit peu mou
//    double Kp = 0.2 * Ku;
//    double Ti = 0.5 * Tu;
//    double Td = 0.33 * Tu;

    //Réglage de Ziegler Nichols faible dépassements : limite de stabilité
//    double Kp = 0.33 * Ku;
//    double Ti = 0.5 * Tu;
//    double Td = 0.33 * Tu;

    //Réglage de Ziegler Nichols par la règle de Pessen intégrale : très peu stable
//    double Kp = 0.7 * Ku;
//    double Ti = 0.4 * Tu;
//    double Td = 0.15 * Tu;

    double Ki = Kp / Ti;
    double Kd = Kp * Td;
    alpha = Kp + Ki * Te + Kd / Te;
    beta = -Kp - 2 * Kd / Te;
    delta = Kd / Te;

    //Autre formulation équivalente
//    alpha = Kp*(1+Te/Ti+Td/Te);
//    beta = Kp*(-1-2*Td/Te);
//    delta = Kp*Td/Te;

    Reset();
    printf("PID init : alpha = %f - beta = %f - delta = %f\n", alpha, beta, delta);
}

Pid::~Pid() {
}

void Pid::Reset(){
	e2 = 0.0;
	e1 = 0.0;
	e0 = 0.0;
	s2 = 0.0;
	s1 = 0.0;
	s0 = 0.0;
}

double Pid::Compute(double e){
    e2 = e1;
    e1 = e0;
    e0 = e;
    if ((e2 == 0) && (e1 == 0) && (e0 == 0))
    	s1 = 0;
    else
    	s1 = s0;

    s0 = s1 + e0 * alpha + e1 * beta + e2 * delta;
    //if (s0 != 0)
    //	printf("Error %f...\n", e);
    return s0;
}
#endif
