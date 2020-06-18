
#include "PWM.h"

extern TIM_HandleTypeDef htim1;

void OutputPWM (int16_t dutycycle)
{
	int16_t dutycyclex = 0;
	dutycyclex = -dutycycle;
	if (dutycycle<0)
		  {
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
			  dutycyclex = dutycyclex +35;
		  	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutycyclex);
		  	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dutycyclex);
		  }
		  else
		  {
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
			  dutycycle = dutycycle +35;
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutycycle);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dutycycle);
		  }
}
