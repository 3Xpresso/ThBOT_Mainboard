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
}

DcMotor::~DcMotor() {
	// TODO Auto-generated destructor stub
}

void DcMotor::SetPercentPower(uint32_t Percentage)
{
	switch (this->id)
	{
	case BSP_DCMOTOR_1 :
	{
		thb_SetPwmLeft(10);
	}break;
	case BSP_DCMOTOR_2 :
	{
		thb_SetPwmRight(10);
	}break;
	}
}

void DcMotor::SetDirection(uint32_t Direction)
{
	switch (this->id)
	{
	case BSP_DCMOTOR_1 :
	{
		if (Direction == FORWARD)
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
	}break;
	case BSP_DCMOTOR_2 :
	{
		if (Direction == FORWARD)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	}break;
	}
}

