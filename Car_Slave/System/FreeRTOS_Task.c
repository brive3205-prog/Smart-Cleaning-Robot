/**
 * @file    FreeRTOS_Task.c
 * @brief   从机 FreeRTOS 系统任务创建及总调度
 */

#include "FreeRTOS_Task.h"



// ==============================================================================================================================
// FreeRTOS 任务句柄与参数定义
// ==============================================================================================================================

// ==========================================
// 1. CAN 发送任务 (优先级较高：3，时间敏感，确保数据实时上报)
// ==========================================
void can_tx_task(void *pvParameters);
#define CAN_TX_TASK_STACK_SIZE 256
#define CAN_TX_TASK_PRIORITY 3
TaskHandle_t can_tx_task_handle;
#define CAN_TX_TASK_PERIOD 20

// ==========================================
// 2. ADC 电量检测任务 (优先级较低：2，实时性要求不高)
// ==========================================
void adc_power_task(void *pvParameters);
#define ADC_POWER_TASK_STACK_SIZE 128
#define ADC_POWER_TASK_PRIORITY 2
TaskHandle_t adc_power_task_handle;
#define ADC_POWER_TASK_PERIOD 500 // 每 500ms 检测一次





/**
 * @brief 启动FreeRTOS操作系统
 * 
 */
void FreeRTOS_start(void)
{
    // ==========================================
    // 硬件与模块的集中初始化 (在RTOS启动前)
    // ==========================================
    Encoder_Init();
    MyCAN_Init();

    // ==========================================
    // 任务创建
    // ==========================================
    // 1. 创建CAN发送任务
    xTaskCreate(can_tx_task, "can_tx_task", CAN_TX_TASK_STACK_SIZE, NULL, CAN_TX_TASK_PRIORITY, &can_tx_task_handle);
    
    // 2. 创建ADC电量检测任务
    xTaskCreate(adc_power_task, "adc_power_task", ADC_POWER_TASK_STACK_SIZE, NULL, ADC_POWER_TASK_PRIORITY, &adc_power_task_handle);

    // 启动调度器
    vTaskStartScheduler();
    
}


// ==========================================
// 1. ADC 电量检测任务实体
// ==========================================
void adc_power_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // 1. 采集：获取当前电量百分比, 并更新全局变量
        current_power_percentage = ADC_Get_Power_Percentage();

        // 2. (可选) 打包发送：调用 CAN 应用层把电量数据塞进邮箱
        //    如果需要通过CAN上报电量, 你需要先实现 Slave_Send_Power_Data() 函数
        Slave_Send_Power_Data();
        
        // 3. 阻塞延时：精准休息 500ms，让出 CPU
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ADC_POWER_TASK_PERIOD));
    }
}



// ==========================================
// 2. CAN 发送任务实体
// ==========================================
void can_tx_task(void *pvParameters)
{
    // 开启 CAN 接收中断，以便响应主机的指令 (如LED控制等)
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // 1. 采集：获取当前车速 (它会自动更新 current_car_speed 全局变量)
        Encoder_Update_Speed();

        // 2. 打包发送：调用 CAN 应用层把速度塞进邮箱 (瞬间完成)
        Slave_Send_Speed_Data();
        
        // 3. 阻塞延时：精准休息 20ms，让出 CPU (对应 CAN_TX_TASK_PERIOD)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CAN_TX_TASK_PERIOD));
    }
}
