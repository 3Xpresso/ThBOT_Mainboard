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

#include "bsp/EncoderABZ.h"

#ifndef ODOMETRY_ODOMETRY_H_
#define ODOMETRY_ODOMETRY_H_

typedef struct {
  double pos_x;
  double pos_y;
  double angle;
  double speed; // mm/ms
  double accel; // Speed/ms
} Odom_t;

class RobotCore;

class Odometry {
public:
	Odometry(RobotCore * Rob);
	virtual ~Odometry();

	void Init(double pos_x, double pos_y, double angle);

	void task();

	Odom_t GetOdomValue()
	{
	  Odom_t res = {
	      this->Pos_x,
	      this->Pos_y,
	      this->Angle,
	      this->Speed,
	      this->Accel
	  };
	  return res;
	}

	int32_t EncoderLeftGetAbsoluteStep(){
		return odomEncoderLeft->GetAbsoluteStep();
	}

	int32_t EncoderRightGetAbsoluteStep(){
		return odomEncoderRight->GetAbsoluteStep();
	}

	double EncoderLeftGetAbsoluteMM(){
		return odomEncoderLeft->GetAbsoluteMM();
	}

	double EncoderRightGetAbsoluteMM(){
		return odomEncoderRight->GetAbsoluteMM();
	}

protected:

	void Update_value(double delta_left_mm, double delta_right_mm);

	double Pos_x;
	double Pos_y;
	double Angle;
	double Speed; // mm/ms
	double Accel; // Speed/ms

	TickType_t previous_tick_count;

	osThreadId odomTaskHandle;

	EncoderABZ * odomEncoderLeft;
	EncoderABZ * odomEncoderRight;

	RobotCore * Robocore;
};

#endif /* ODOMETRY_ODOMETRY_H_ */
