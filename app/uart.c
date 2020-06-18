
#include "uart.h"

extern uint8_t rxbuff;
volatile uint8_t status = 0;
uint8_t txbuff[][50] =
{
		"\nKhoi dong debug... ",
		"\nDung debug./.",
		"\nNhap Kp: ",
		"\nNhap Ki: ",
		"\nNhap Kd: ",
		"\nChon che do debug: ",
		"\nChinh pwm: "
};
uint8_t rxbuffs[15];
volatile float k = 0;
extern volatile float kpangle;//kpmotor ;
extern volatile float kiangle;//kimotor ;
extern volatile float kdangle;//kdmotor ;

extern volatile int16_t count;
extern volatile float synchrocount;
extern volatile int16_t duty;
extern volatile float synchroduty;
extern volatile float set;
extern uint8_t mode;
extern volatile uint16_t x;
extern uint8_t choose ;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//if (__HAL_UART_GET_IT_SOURCE(&huart2,UART_IT_RXNE))
	{
		HAL_UART_Transmit(&huart2, &rxbuff, 1, 1000);
		while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC));

		//Bat dau Debug
		if ((rxbuff == 'b') && (status == 0))
		{
			count = 0;
			synchrocount = 0;
			synchroduty = 0;
			duty = 0;
			set = 0;
			x = 0;

			HAL_TIM_Base_Start_IT(&htim4);
			HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
			HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

			HAL_UART_Transmit(&huart2, txbuff[0], sizeof(txbuff[0]), 1000);
			while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC));
			status = 1;
		}

		//Ket thuc debug
		if ((rxbuff == 's') && (status == 1))
		{
			HAL_TIM_Base_Stop_IT(&htim4);
			HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_1);
			HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);

			HAL_UART_Transmit(&huart2, txbuff[1], sizeof(txbuff[1]), 1000);
			while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC));
			status = 0;

			count = 0;
			synchrocount = 0;
			synchroduty = 0;
			duty = 0;
			set = 1000;
			x = 0;
		}

		//Nhap Kp
		if ((rxbuff == 'p') && (status == 0))
		{
			HAL_UART_Transmit(&huart2, txbuff[2], sizeof(txbuff[2]), 1000);
			while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC));
			uint8_t i = 0;
			uint8_t j = 0;
			float dec = 0;
			do
			{
				while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_RXNE));
				HAL_UART_Receive(&huart2, rxbuffs+i, 1, 1000);
				HAL_UART_Transmit(&huart2, rxbuffs+i, 1, 1000);
				while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC));
				if (rxbuffs[i] == '.')
				{
					k = rxbuffs[0]-48;
					for (j = 1;j<i ; j++)
						k = k*10 + (rxbuffs[j]-48);
					//j = i+1;
				}
			} while (rxbuffs[i++] != '\n');
			if (j)
			{
				for (i=i-3;i>j;i--)
					dec = dec*0.1 + (rxbuffs[i]-48);
				dec = dec*0.1;
			}
			else
			{
				k = rxbuffs[0]-48;
				for (j = 1;j<i-2 ; j++)
				k = k*10 + (rxbuffs[j]-48);
			}
			k+=dec;
			kpangle = k;
		}

		//Nhap Ki
		if ((rxbuff == 'i') && (status == 0))
		{
			HAL_UART_Transmit(&huart2, txbuff[3], sizeof(txbuff[3]), 1000);
			while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC));
			uint8_t i = 0;
			uint8_t j = 0;
			float dec = 0;
			do
			{
				while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_RXNE));
				HAL_UART_Receive(&huart2, rxbuffs+i, 1, 1000);
				HAL_UART_Transmit(&huart2, rxbuffs+i, 1, 1000);
				while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC));
				if (rxbuffs[i] == '.')
				{
					k = rxbuffs[0]-48;
					for (j = 1;j<i ; j++)
						k = k*10 + (rxbuffs[j]-48);
					//j = i+1;
				}
			} while (rxbuffs[i++] != '\n');
			if (j)
			{
				for (i=i-3;i>j;i--)
					dec = dec*0.1 + (rxbuffs[i]-48);
				dec = dec*0.1;
			}
			else
			{
				k = rxbuffs[0]-48;
				for (j = 1;j<i-2 ; j++)
				k = k*10 + (rxbuffs[j]-48);
			}
			k+=dec;
			kiangle = k;
		}

		//Nhap Kd
		if ((rxbuff == 'd') && (status == 0))
		{
			HAL_UART_Transmit(&huart2, txbuff[4], sizeof(txbuff[4]), 1000);
			while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC));
			uint8_t i = 0;
			uint8_t j = 0;
			float dec = 0;
			do
			{
				while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_RXNE));
				HAL_UART_Receive(&huart2, rxbuffs+i, 1, 1000);
				HAL_UART_Transmit(&huart2, rxbuffs+i, 1, 1000);
				while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC));
				if (rxbuffs[i] == '.')
				{
					k = rxbuffs[0]-48;
					for (j = 1;j<i ; j++)
						k = k*10 + (rxbuffs[j]-48);
					//j = i+1;
				}
			} while (rxbuffs[i++] != '\n');
			if (j)
			{
				for (i=i-3;i>j;i--)
					dec = dec*0.1 + (rxbuffs[i]-48);
				dec = dec*0.1;
			}
			else
			{
				k = rxbuffs[0]-48;
				for (j = 1;j<i-2 ; j++)
				k = k*10 + (rxbuffs[j]-48);
			}
			k+=dec;
			kdangle = k;
		}

		//Chinh che do debug
		/*if ((rxbuff == 'm') && (status ==0))
		{
			HAL_UART_Transmit(&huart2, txbuff[5], sizeof(txbuff[5]), 1000);
			while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC));
			while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_RXNE));
			HAL_UART_Receive(&huart2, &rxbuff, 1, 1000);
			HAL_UART_Transmit(&huart2, &rxbuff, 1, 1000);
			while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC));
			mode = rxbuff - 48;
		}*/

		/*if ((rxbuff == 'c') && (status == 0))
		{
			HAL_UART_Transmit(&huart2, txbuff[6], sizeof(txbuff[6]), 1000);
			while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC));
			uint8_t i = 0;
			uint8_t j = 0;
			do
			{
				while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_RXNE));
				HAL_UART_Receive(&huart2, rxbuffs+i, 1, 1000);
				HAL_UART_Transmit(&huart2, rxbuffs+i, 1, 1000);
				while (!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC));
			} while (rxbuffs[i++] != '\n');
			{
				k = rxbuffs[0]-48;
				for (j = 1;j<i-2 ; j++)
				k = k*10 + (rxbuffs[j]-48);
			}
			choose = k;
		}*/

		HAL_UART_Receive_IT(&huart2, &rxbuff, 1);
	}
}
