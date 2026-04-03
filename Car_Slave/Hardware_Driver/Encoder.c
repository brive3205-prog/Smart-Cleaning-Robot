#include "Encoder.h"

// 完美匹配你的最小系统板：TIM1, TIM2, TIM3, TIM4
extern TIM_HandleTypeDef htim1; 
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

Car_Speed_t current_car_speed = {0, 0, 0, 0};
uint8_t current_power_percentage = 0; // 全局电量百分比变量

/**
 * @brief 启动所有 4 个定时器的编码器模式
 */
void Encoder_Init(void)
{
    // 注意：如果是高级定时器 TIM1，底层库依然是这句代码
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

/**
 * @brief 刷新 4 个轮子的速度，并完美处理正负号转换
 */
void Encoder_Update_Speed(void)
{
    // 1. 先用无符号 16 位变量把底层寄存器的原值读出来 (0~65535)
    uint16_t raw_1 = __HAL_TIM_GET_COUNTER(&htim1);
    uint16_t raw_2 = __HAL_TIM_GET_COUNTER(&htim2);
    uint16_t raw_3 = __HAL_TIM_GET_COUNTER(&htim3);
    uint16_t raw_4 = __HAL_TIM_GET_COUNTER(&htim4);
    
    // 2. 抓取完立刻清零
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    // 3. 强制转换为有符号的 int16_t
    // 这样 65535 就会自动变成 -1，65500 就会变成 -36！
    int16_t speed_1 = (int16_t)raw_1;
    int16_t speed_2 = (int16_t)raw_2;
    int16_t speed_3 = (int16_t)raw_3;
    int16_t speed_4 = (int16_t)raw_4;
    
    // 4. 方向矫正
    // 习惯上我们认为前进应该是正数！所以直接在这里加个负号取反：
    current_car_speed.speed_1 = -speed_1;
    current_car_speed.speed_2 = speed_2;
    current_car_speed.speed_3 = -speed_3;
    current_car_speed.speed_4 = speed_4;
}
