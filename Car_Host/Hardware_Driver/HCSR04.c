/**
 * @file    HCSR04.c
 * @brief   超声波传感器底层输入捕获驱动及中值滤波去噪算法
 */

#include "HCSR04.h"

#define FILTER_N 5 // 采样次数

/**
 * @brief 底层原始测距 (极其严格的时序安检版本)
 */
float HCSR04_GetDistance(void)
{
	// 1. 清除标志位，准备迎接新一轮
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC3);
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC4);
    
	// 2. 启动输入捕获
	HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_4);
    
	// 3. 发送 Trig 脉冲
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	for(uint32_t i = 0; i < 500; i++) { __NOP(); }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    
	// 4. 等待上升沿 (CC3)，增加超时机制，最多等 5 毫秒
	uint32_t wait_start = HAL_GetTick();
	while(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC3) == RESET)
	{
		if(HAL_GetTick() - wait_start > 5) 
		{
			HAL_TIM_IC_Stop(&htim1, TIM_CHANNEL_3);
			HAL_TIM_IC_Stop(&htim1, TIM_CHANNEL_4);
			return 0.0f; // 发出去没声音，直接判死刑
		}
	}
	
	// 锁定上升沿时间！
	uint16_t ccr3 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
	
	// 极其重要：在等下降沿之前，再清一次 CC4，防止前面的杂波捣乱
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC4);

	// 5. 等待下降沿 (CC4)，超时保护最多等 18 毫秒 (对应距离约 3 米)
    // 限制 18 毫秒，保证定时器(20ms周期)绝对不可能溢出超过 1 次！
	wait_start = HAL_GetTick();
	while(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC4) == RESET)
	{
		if(HAL_GetTick() - wait_start > 18) 
		{
			HAL_TIM_IC_Stop(&htim1, TIM_CHANNEL_3);
			HAL_TIM_IC_Stop(&htim1, TIM_CHANNEL_4);
			return 0.0f; // 太远了或者丢失了，直接判死刑
		}
        // 安全让出 CPU 给别的任务
		vTaskDelay(pdMS_TO_TICKS(1)); 
	}
    
	// 锁定下降沿时间！
	uint16_t ccr4 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
    
	// 6. 关门闭客
	HAL_TIM_IC_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_IC_Stop(&htim1, TIM_CHANNEL_4);
    
	// 7. 溢出补偿计算
	uint32_t pulseWidth = 0;
	if(ccr4 >= ccr3) pulseWidth = ccr4 - ccr3; 
	else pulseWidth = (ccr4 + 20000) - ccr3; 

	return pulseWidth * 0.017f; 
}

/**
 * @brief  获取经过中值滤波后的超声波距离
 */
float HCSR04_GetDistance_Filtered(void)
{
    static float dist_buffer[FILTER_N] = {0}; 
    static uint8_t buf_idx = 0;               
    static uint8_t is_buf_full = 0;
    static float last_valid_dist = 0.0f; 

    float raw_dist = HCSR04_GetDistance();

    // 异常数据屏蔽网：距离小于 2 厘米(物理盲区杂音) 或 大于 300 厘米(太远)，拒绝进入缓冲区
    if (raw_dist < 2.0f || raw_dist > 300.0f) 
    {
        return last_valid_dist; 
    }

    // 安全数据，放行进入数组
    dist_buffer[buf_idx] = raw_dist;
    buf_idx++;
    if (buf_idx >= FILTER_N) 
    {
        buf_idx = 0;
        is_buf_full = 1; 
    }

    if (is_buf_full == 0) 
    {
        last_valid_dist = raw_dist;
        return raw_dist;
    }
    
    // 冒泡排序，取中值
    float temp_buf[FILTER_N];
    for (int i = 0; i < FILTER_N; i++) {
        temp_buf[i] = dist_buffer[i];
    }

    for (int i = 0; i < FILTER_N - 1; i++) {
        for (int j = 0; j < FILTER_N - 1 - i; j++) {
            if (temp_buf[j] > temp_buf[j + 1]) {
                float temp = temp_buf[j];
                temp_buf[j] = temp_buf[j + 1];
                temp_buf[j + 1] = temp;
            }
        }
    }
    
    // 提取最中间的那个“完美数据”
    last_valid_dist = temp_buf[FILTER_N / 2]; 
    return last_valid_dist; 
}
