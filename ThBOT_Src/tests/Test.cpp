/*
 * Test.cpp
 *
 *  Created on: 25 mars 2018
 *      Author: jpb
 */

#include <string.h>

#include <RobotCore/RobotCore.h>
#include <tests/Test.h>

#include "stm32f4xx_hal.h"
#include "tim.h"

#include "thb-fsm.h"
#include "thb-bsp.h"

static char SendResponse[64];
static char TestResponse[48];

#define TIMER_INIT_VALUE     2147483647
static uint32_t count_left = 0;
static uint32_t count_right = 0;

static void test_send_reponse(uint32_t State, char * pu8_Buff)
{
	sprintf(SendResponse, "RSP:%2.2u:%2.2u:%s\n", MODE_TESTS, (unsigned int)State, pu8_Buff);
	//printf("Response [%i] : %s", strlen(SendResponse), SendResponse);
	thb_UART5_SendData(SendResponse, strlen(SendResponse));
}

Test::Test(RobotCore * Rob) {
	Robocore = Rob;

}

Test::~Test() {
}

int Test::Run(uint32_t State)
{
	GPIO_PinState PwrIn;
	uint32_t count_1 = 0;
	uint32_t count_2 = 0;
	char dir_1 = '\0';
	char dir_2 = '\0';
	uint32_t PercentPower = Robocore->GetPercentPower();

	switch (State)
	{
    	case STATE_IDLE :
    	{
    		thb_SetPwmRight(0);
    		thb_SetPwmLeft(0);
    	    return 0;
    	}break;
	    case STATE_GET_PWR_STATE :
	    {
	    	PwrIn = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_0);
	    	printf("JPB3 : %d \n", PwrIn);

	    	if (PwrIn == GPIO_PIN_RESET)
	    	  sprintf(TestResponse, "%s", "ON ");
	    	else
	    	  sprintf(TestResponse, "%s", "OFF");

	    	test_send_reponse(State, TestResponse);

	    }break;
	    case STATE_ENC_READ :
	    {
	    	count_1 = __HAL_TIM_GET_COUNTER(&htim2);
	    	count_2 = __HAL_TIM_GET_COUNTER(&htim5);

	    	if (count_1 >= TIMER_INIT_VALUE)
	    	{
	    		count_right = count_1 - TIMER_INIT_VALUE;
	    		dir_1 = '+';
	    	}
	    	else
	    	{
	    		count_right = TIMER_INIT_VALUE - count_1;
	    		dir_1 = '-';
	    	}

	    	if (count_2 > TIMER_INIT_VALUE)
	    	{
	    		count_left  = count_2 - TIMER_INIT_VALUE;
	    		dir_2 = '-';
	    	}
	    	else
	    	{
	    		count_left  = TIMER_INIT_VALUE - count_2;
	    		dir_2 = '+';
	    	}

	    	printf("Tick right = %10.10u (%c)\n",
	    			(unsigned int)count_right, dir_1);
	    	printf("Tick left  = %10.10u (%c)\n",
	    			(unsigned int)count_left, dir_2);
	    	sprintf(TestResponse, "%10.10u:%c:%10.10u:%c",
	    			(unsigned int)count_left, dir_2,
					(unsigned int)count_right, dir_1);
	    	test_send_reponse(State, TestResponse);
	    }break;
	    case STATE_MOVE_FORWARD :
	    {
	    	printf("JPB3 : test marche avant \n");
	    	thb_SetPwmRight(PercentPower);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	    	thb_SetPwmLeft(PercentPower);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);

	    }break;
	    case STATE_MOVE_BACKWARD :
	    {
	    	printf("JPB3 : test marche arriere \n");
	    	thb_SetPwmRight(PercentPower);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	    	thb_SetPwmLeft(PercentPower);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
	    }break;
	    case STATE_TURN_RIGTH :
	    {
	    	printf("JPB3 : test rotation a droite \n");
	    	thb_SetPwmRight(PercentPower);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	    	thb_SetPwmLeft(PercentPower);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
	    }break;
	    case STATE_TURN_LEFT :
	    {
	    	printf("JPB3 : test rotation a gauche \n");
	    	thb_SetPwmRight(PercentPower);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	    	thb_SetPwmLeft(PercentPower);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
	    }break;
	    case STATE_ODOM_PARAMS :
	    {
	    	sprintf(TestResponse, "%s",
	    			"0000000.01:0001040.01:0000137.24:0000.78345");
	    	test_send_reponse(State, TestResponse);
	    }break;
	}
	osDelay(2000);
	thb_fsm_ChangeModeState(MODE_TESTS, STATE_IDLE);
	return 0;
}
