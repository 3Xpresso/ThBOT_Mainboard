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
  int32_t pos_x;
  int32_t pos_y;
  double angle;
  double speed; // mm/ms
  double accel; // Speed/ms
} Odom_t;

class Odometry {
public:
	Odometry();
	virtual ~Odometry();

	void Init(int32_t pos_x, int32_t pos_y, double angle);

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

protected:

	void Update_value(double delta_left_mm, double delta_right_mm);

	int32_t Pos_x;
	int32_t Pos_y;
	double Angle;
	double Speed; // mm/ms
	double Accel; // Speed/ms

	TickType_t previous_tick_count;

	osThreadId odomTaskHandle;

	EncoderABZ * odomEncoderLeft;
	EncoderABZ * odomEncoderRight;
};

#endif /* ODOMETRY_ODOMETRY_H_ */
