/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "stm32f4xx_hal.h"
#include "tim.h"
#include "math.h"

#include "ThBot_platform.h"
#include "EncoderABZ.h"

#define TIMER_INIT_VALUE     2147483647

EncoderABZ::EncoderABZ(Encodeur_t id) {
	this->id = id;
	this->Absolute_value = 0;
	this->MM_per_step = (M_PI * ENCODER_WHEEL_SIZE / ENCODER_NB_STEP);
}

EncoderABZ::~EncoderABZ() {
	// TODO Auto-generated destructor stub
}

int32_t EncoderABZ::GetDeltaStep()
{
	int32_t new_absolute = 0;
	int32_t delta = 0;

	new_absolute = GetAbsoluteStep();

	// calculate delta (new - old)
	delta = new_absolute - this->Absolute_value;

	//update reference = new
	this->Absolute_value = new_absolute;

	return delta;
}

double EncoderABZ::GetDeltaMM()
{
	int32_t delta_step = GetDeltaStep();
	double delta_mm = delta_step * MM_per_step;

	return delta_mm;
}

int32_t EncoderABZ::GetAbsoluteStep()
{
	int32_t absoluteCount = 0;

	switch (this->id)
	{
		case ENCODER_LEFT:
		{
			const uint32_t count_1 = __HAL_TIM_GET_COUNTER(&htim5);
			absoluteCount = TIMER_INIT_VALUE - count_1;
		}break;
		case ENCODER_RIGHT:
		{
			const uint32_t count_1 = __HAL_TIM_GET_COUNTER(&htim2);
			absoluteCount = TIMER_INIT_VALUE - count_1;
		}break;
	}
	return absoluteCount;
}
