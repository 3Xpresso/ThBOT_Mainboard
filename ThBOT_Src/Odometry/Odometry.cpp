/*
 * Odometry.cpp
 *
 *  Created on: 10 mars 2018
 *      Author: jpb
 */

/* Inc/FreeRTOSConfig.h: #define INCLUDE_vTaskDelayUntil 1 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "thb-task.h"

#include "Odometry.h"
#include "ThBot_platform.h"

static void taskWrapper(const void* arg)
{
	Odometry* odom = const_cast<Odometry*>(static_cast<const Odometry*>(arg));

	odom->task();
}

Odometry::Odometry(RobotCore * Rob) {

	Robocore = Rob;

	osThreadDef(odomTask, taskWrapper, PRIORITY_ODOMETRY, 0, 128);
	odomTaskHandle = osThreadCreate(osThread(odomTask), this);

	odomEncoderLeft  = new EncoderABZ(ENCODER_LEFT);
	odomEncoderRight = new EncoderABZ(ENCODER_RIGHT);
	Init(0.0, 0.0, 0.0);
}

void Odometry::Init(double pos_x, double pos_y, double angle)
{
	Pos_x = pos_x;
	Pos_y = pos_y;
	Angle = angle;
	Accel = 0.0;
	previous_tick_count = xTaskGetTickCount();
}

Odometry::~Odometry() {
	// TODO Auto-generated destructor stub
}

void Odometry::task(void)
{
	TickType_t xLastWakeTime;

	printf("Start odometry task...\n");

	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		osDelay(3000);

		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 3000 ));
	}

}

// mise a jour par approximation de droite
void Odometry::Update_value(double delta_left_mm, double delta_right_mm)
{
	TickType_t current_tick_count = xTaskGetTickCount();
	TickType_t delta_tick = current_tick_count - previous_tick_count;
	previous_tick_count = current_tick_count;

	uint32_t delay_ms = delta_tick/portTICK_PERIOD_MS;

	double deplacement_mm = (delta_left_mm + delta_right_mm)/2;

	if (!delay_ms) {
		Speed      = 0;
		SpeedLeft  = 0;
		SpeedRight = 0;
	}
	else {
		Speed      = deplacement_mm / delay_ms * 1000;
		SpeedLeft  = delta_left_mm  / delay_ms * 1000;
		SpeedRight = delta_right_mm / delay_ms * 1000;
	}
	double delta_x = deplacement_mm*cos(Angle);
	double delta_y = deplacement_mm*sin(Angle);
	double delta_angle = (delta_right_mm - delta_left_mm) / ENCODER_DISTANCE;

	Angle = Angle + delta_angle;
	Pos_x = Pos_x + delta_x;
	Pos_y = Pos_y + delta_y;
}

Wheel_t Odometry::UpdateValue(void){
	double delta_l = odomEncoderLeft->GetDeltaMM();
	double delta_r = odomEncoderRight->GetDeltaMM();
	Update_value(delta_l, delta_r);

	Wheel_t res = {
	      this->SpeedLeft,
	      this->SpeedRight,
	};
	return res;
}

Odom_t Odometry::GetOdomValue() {
  Odom_t res = {
      this->Pos_x,
      this->Pos_y,
      this->Angle,
      this->Speed,
      this->Accel
  };
  return res;
}
