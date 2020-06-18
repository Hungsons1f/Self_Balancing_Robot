/*Description
 *Clock Configuration: HCLK: 64MHz
 *Peripherals:
 **TIM2: Slave Mode: External Clock Mode 1; Trigger Source: TI1FP1; Counter Period: 0xffff; Trigger Filter: 15
 **TIM3: Internal Clock; Prescaler: 63; Counter Period: 9999
 **USART1:
 *Pin:
 **PA0: TIM2_CH1
 **PA9: USART1_TX
 **PA10: USART1_RX
*/
#include "ReadEncoder.h"

volatile float kpmotor = 6;//3;
volatile float kimotor = 8;//0.5
volatile float kdmotor = 0.15;
float time = 0.011;
volatile float kpangle = 3.3;
volatile float kiangle = 19.8;
volatile float kdangle = 0.1375;
/*uint16_t vec = 0;
*/

volatile int16_t count = 0;
volatile int16_t count1 = 0;
volatile float synchrocount = 0;
extern volatile int16_t duty;				//duty min: 40, duty max: 85; count max 1: 550, count max 2: 590
volatile float synchroduty =0;
volatile float set = 0;
uint8_t mode = 1;
extern float roll;


extern uint16_t gx[20];
extern SD_MPU6050 mpu1;
extern float sum_gForcex;
extern float sum_gForcey ;
extern float sum_gForcez ;
extern float gForcex ;
extern float gForcey ;
extern float gForcez ;
extern float bias_gForcex ;
extern float bias_gForcey ;
extern float bias_gForcez ;
extern float sum_rotx ;
extern float sum_roty ;
extern float sum_rotz;
extern float rotx ;
extern float roty ;
extern float rotz ;
extern float bias_rotx ;
extern float bias_roty ;
extern float bias_rotz ;
extern SD_MPU6050_Result result ;
float dt = 0.011;
float accel = 0;
float gyro = 0;

uint8_t cache[3];
volatile uint16_t x = 0;
volatile uint8_t dem = 0;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_UPDATE))
	{
	/*count = __HAL_TIM_GET_COUNTER(&htim2);			//standard: duty: 70; count 1: 380, count 2: 380*1.1
	count1 =  __HAL_TIM_GET_COUNTER(&htim3);
	//vec = (count1*5/3 );							//tinh toc do

	synchrocount = (float)count*1500/550.0;
	synchroduty = PID(set, (float)synchrocount, kpmotor, kimotor, kdmotor);
	duty = (int16_t)(synchroduty*30/1000.0);
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_COUNTER(&htim3, 0);*/


		 //init mpu
		  result = SD_MPU6050_Init(&hi2c2,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_250s );

		  	  //read all sensors
		  		SD_MPU6050_ReadAll(&hi2c2,&mpu1);
		  		//read gyro
		  //	  SD_MPU6050_ReadGyroscope(&hi2c2,&mpu1);
		  	  int16_t g_x = mpu1.Gyroscope_X;
		  	  int16_t g_y = mpu1.Gyroscope_Y;
		  	  int16_t g_z = mpu1.Gyroscope_Z;
	    /* USER CODE END WHILE */

	    /* USER CODE BEGIN 3 */
		  	  //read accel
		//		SD_MPU6050_ReadAccelerometer(&hi2c2,&mpu1);
			  int16_t a_x = mpu1.Accelerometer_X;
			  int16_t a_y = mpu1.Accelerometer_Y;
			  int16_t a_z = mpu1.Accelerometer_Z;

			  //cal gyro
				rotx = (float)(g_x - bias_rotx)/131;
				roty = (float)(g_y - bias_roty)/131;
				rotz = (float)(g_z - bias_rotz)/131;
				//cal accel
				gForcex = (float)a_x/16384 - bias_gForcex;
				gForcey = (float)a_y/16384 - bias_gForcey;
				gForcez = (float)a_z/16384 - bias_gForcez;


				//cal rol and pitch
				 accel  = (float)atan2((double)gForcey,(double)gForcez)*57.29577951;
				 if (accel< 0) accel += 179;
				 else accel -=179;
				 gyro  = roll + rotx*dt;
				 roll = 0.962*gyro + 0.038* accel;
				 //pitch = -atan2((double)gForcex,(double)sqrt((double)gForcey*(double)gForcey+(double)gForcez*(double)gForcez))*57.29577951;





	//roll range: -40 -> 40, duty: 40 -> 85
	synchrocount = roll*1000/40.0;
	synchroduty  =PID(set, synchrocount, kpangle, kiangle, kdangle);
	duty = (int16_t)(synchroduty*75/1000.0);

	uint8_t temp = 0;
	 if (roll <0)
	 {
		HAL_UART_Transmit(&huart2, "-", 1, 1000);
		while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC));
		temp = (uint8_t)(-roll);
	 }
	 else
	 {
		 temp = (uint8_t)(roll);
	 }
	 cache[0] = (temp/100) +48;
	 cache[1] = (temp- (cache[0]-48)*100)/10 +48 ;
	 cache[2] = ((temp - (cache[0]-48)*100 - (cache[1]-48)*10))+48;
	HAL_UART_Transmit(&huart2, cache, sizeof(cache), 1000);
	while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC));
	HAL_UART_Transmit(&huart2, "\n", 1, 1000);
	while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC));
	/*x++;
	if (x==200)
			{
				if (dem ==0)
				{
					set+=100;
					if (set == 1000) dem = 1;
				}
				else
				{
					set-=100;
					if (set == -1000) dem =0;
				}
				x=0;
			}
	switch (mode)
	{
	case 1:
		if (x == 400)
		{
			if (set == 1000) set = 0;
			else set = 1000;
			x = 0;
		}
		break;
	case 2:
		if (x==400)
		{
			if (set == 1000) set = 0;
			else if (set == 0) set = -1000;
			else set = 1000;
			x = 0;
		}
		break;
	case 3:
		if (x==400)
		{
			//if (set = 1000)
		}
		break;
	default:
		break;
	}*/

	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);		//tin hieu debug
	}
}

float PID(float setpoint, float measure, float kp, float ki, float kd)
{
	static volatile float e0 = 0;
	static volatile float e1 = 0;
	static volatile float e2 = 0;
	static volatile float u0 = 0;
	static volatile float u1 = 0;
	e2 = e1;
	e1 = e0;
	e0 = setpoint - measure;
	u1 = u0;
	u0 = (u1 + kp*(e0 - e1) + ki*time*(e0 + e1)/2.0 + kd*(e0 - 2*e1 + e2)/time);
	//if (u0 < 0 ) u0 = 0;
	//if (u0>1500 ) u0 =1500;
	return u0;
}
