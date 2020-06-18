#include "main.h"
#include "sd_hal_mpu6050.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
float PID(float setpoint, float measure, float kp, float ki, float kd);
