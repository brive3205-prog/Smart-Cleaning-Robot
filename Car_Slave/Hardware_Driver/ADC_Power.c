#include "ADC_Power.h"
#include "adc.h" // 包含此头文件以使用 hadc1

// 定义电池电压对应的ADC采样值范围
// 用户需要根据实际的电压采样电路和电池类型来校准这两个值
// 你的电池是 7.4V (标称电压), 通常满电电压为 8.4V, 亏电截止电压约为 6.4V.
//
// 假设:
// - 使用12位ADC (0-4095)
// - ADC参考电压为3.3V
// - PA4 引脚前有一个分压电路, 例如 R1=20k, R2=10k, 那么分压比为 1/3.
//
// 校准计算示例:
// 电池满电 8.4V -> PA4实际电压 = 8.4V * R2 / (R1 + R2) = 8.4V / 3 = 2.8V
//                  ADC采样值 = (2.8V / 3.3V) * 4095 ≈ 3474
// 电池亏电 6.4V -> PA4实际电压 = 6.4V / 3 ≈ 2.13V
//                  ADC采样值 = (2.13V / 3.3V) * 4095 ≈ 2643
//
// !!!警告!!! 下面的值是基于以上假设的示例, 在实际使用前, 请务必用万用表测量
// 满电和亏电时 PA4 引脚的真实电压, 并重新计算下面的宏定义!
#define ADC_POWER_FULL      3474  // 示例值, 对应满电 (例如 8.4V)
#define ADC_POWER_EMPTY     2643  // 示例值, 对应亏电 (例如 6.4V)

// 【新增】用于低通滤波的静态变量
static float filtered_adc_value = 0.0f;

/**
 * @brief  测量电源电压并返回电量百分比
 * @retval uint8_t 电量百分比 (0-100)
 * @note   此函数为阻塞式, 会等待ADC转换完成.
 *         但转换时间很短(微秒级), 在低频调度的RTOS任务中使用是完全可以接受的,
 *         不会对其他任务造成明显影响.
 */
uint8_t ADC_Get_Power_Percentage(void)
{
    uint32_t adc_value;
    int32_t percentage;

    // 1. 启动ADC转换
    HAL_ADC_Start(&hadc1);

    // 2. 等待ADC转换完成 (带100ms超时)
    //    对于STM32的ADC来说, 一次转换通常在几个微秒到几十个微秒,
    //    所以100ms的超时非常充足.
    if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK)
    {
        // 如果等待超时, 返回0电量作为错误指示
        return 0;
    }

    // 3. 读取ADC转换结果
    adc_value = HAL_ADC_GetValue(&hadc1);
    
    // 4. 【新增】对ADC原始值进行低通滤波
    if (filtered_adc_value == 0.0f)
    {
        // 如果是第一次运行，直接用当前值初始化滤波器
        filtered_adc_value = (float)adc_value;
    }
    else
    {
        // IIR低通滤波：新值占10%，历史值占90%
        filtered_adc_value = filtered_adc_value * 0.9f + (float)adc_value * 0.1f;
    }

    // 5. 停止ADC, 这是一个好习惯
    HAL_ADC_Stop(&hadc1);

    // 6. 根据【滤波后】的最大/最小采样值计算电量百分比 (线性映射)
    if (filtered_adc_value >= ADC_POWER_FULL)
    {
        percentage = 100;
    }
    else if (filtered_adc_value <= ADC_POWER_EMPTY)
    {
        percentage = 0;
    }
    else
    {
        // 使用长整型(long)进行计算, 避免中间结果溢出
        percentage = (long)(filtered_adc_value - ADC_POWER_EMPTY) * 100 / (ADC_POWER_FULL - ADC_POWER_EMPTY);
    }

    return (uint8_t)percentage;
}
