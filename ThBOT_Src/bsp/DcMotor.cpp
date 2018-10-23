/*
 * DcMotor.cpp
 *
 *  Created on: 11 mars 2018
 *      Author: jpb
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#include "thb-bsp.h"

#include "DcMotor.h"

DcMotor::DcMotor(uint32_t id) {
	this->id = id;
	this->Percentage = 0;
}

DcMotor::~DcMotor() {
}

void DcMotor::SetSpeed(double expectedSpeed){

	switch (this->id)
	{
	case BSP_DCMOTOR_1 :
	{
		thb_StatsSetTargetSpeedLeft(expectedSpeed);
	}break;
	case BSP_DCMOTOR_2 :
	{
		thb_StatsSetTargetSpeedRight(expectedSpeed);
	}break;
	}
}

void DcMotor::SetPercentPower(float Percentage)
{
	this->Percentage = Percentage;

	switch (this->id)
	{
	case BSP_DCMOTOR_1 :
	{
		thb_SetPwmLeft(Percentage);
	}break;
	case BSP_DCMOTOR_2 :
	{
		thb_SetPwmRight(Percentage);
	}break;
	}
}

void DcMotor::UpdatePercentPower(float DeltaPercentage)
{
	if ((DeltaPercentage > -100.0) && (DeltaPercentage < 100.0)){
		switch (this->id)
		{
		case BSP_DCMOTOR_1 :
		{
			thb_SetPwmLeft(Percentage + DeltaPercentage);
		}break;
		case BSP_DCMOTOR_2 :
		{
			thb_SetPwmRight(Percentage + DeltaPercentage);
		}break;
		}
	}
}

void DcMotor::SetDirection(uint32_t Direction)
{
	switch (this->id)
	{
	case BSP_DCMOTOR_1 :
	{
		if (Direction == BACKWARD)
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
	}break;
	case BSP_DCMOTOR_2 :
	{
		if (Direction == BACKWARD)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	}break;
	}
}

