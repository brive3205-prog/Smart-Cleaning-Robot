#include "Motor.h"




void Motor_Init(void)
{
	PWM_Init();
}

/**
  * 函    数：直流电机设置速度
  * 参    数：Speed 要设置的速度，范围：-100~100
  * 返 回 值：无
  */
void Motor_SetSpeed(uint8_t n,int16_t Speed)
{
	if(n == 1)
	{
		if (Speed >= 0)							//如果设置正转的速度值
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);	//PB12置高电平
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);	//PB13置低电平，设置方向为正转
			PWM_SetCompare1(Speed);				//PWM设置为速度值
		}
		else									//否则，即设置反转的速度值
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	//PB12置低电平
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);	//PB13置高电平，设置方向为反转
			PWM_SetCompare1(-Speed);			//PWM设置为负的速度值，因为此时速度值为负数，而PWM只能给正数
		}
	}
	else if(n == 2)
	{
		if (Speed >= 0)							
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);	
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);	
			PWM_SetCompare2(Speed);				
		}
		else									
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);	
			PWM_SetCompare2(-Speed);			
		}
	}
	else if(n == 3)
	{
		if (Speed >= 0)							
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);	
			PWM_SetCompare3(Speed);				
		}
		else								
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);	
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			PWM_SetCompare3(-Speed);	
		}		
	}
	else if(n == 4)
	{
		if (Speed >= 0)							
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);	
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);	
			PWM_SetCompare4(Speed);				
		}
		else									
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);	
			PWM_SetCompare4(-Speed);			
		}
	}
	//扫地
	else if(n == 5)
	{
		if (Speed >= 0)							
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);	
			PWM_SetCompare5(Speed);				
			PWM_SetCompare6(Speed);
		}
		else									
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);	
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);	
			PWM_SetCompare5(-Speed);			
			PWM_SetCompare6(-Speed);
		}
	}
}

