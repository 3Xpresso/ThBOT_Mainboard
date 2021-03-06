#include "tim.h"
#include "stm32f4xx_hal_tim.h"

#include "thb-stats.h"

//extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
//extern TIM_HandleTypeDef htim5;

void thb_SetPwmRight(float Val)
{
	float PwmValue = Val * 10;
	int tmpInt = PwmValue;
    /* Set the Capture Compare Register value */
    //TIM3->CCR1  = Val;  // PWM_CH1
    //TIM3->CCR2  = Val;  // PWM_CH2
    //TIM3->CCR3  = 10000;  // PWM_CH3
    //TIM3->CCR4  = 10000;  // PWM_CH4

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, tmpInt);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    thb_StatsSetPwmRight(tmpInt);
}

void thb_SetPwmLeft(float Val)
{
	float PwmValue = Val * 10;
	int tmpInt = PwmValue;
    //TIM4->CCR1  = Val;  // PWM_CH1
	//TIM4->CCR2  = Val;  // PWM_CH2
    // __HAL_TIM_SET_AUTORELOAD

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, tmpInt);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

    thb_StatsSetPwmLeft(tmpInt);
}
