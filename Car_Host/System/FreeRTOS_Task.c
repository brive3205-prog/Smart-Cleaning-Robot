/**
 * @file    FreeRTOS_Task.c
 * @brief   FreeRTOS 系统任务创建、资源分配(队列/互斥锁)及总调度
 */

#include "FreeRTOS_Task.h"

// ==========================================
// 变量与句柄的真正实体定义区
// ==========================================
SystemStatus_t g_car_status = {0.0f, 30.0f, 0.0f, 0.0f, 0, 0, 0, 0, 0, 0.0f, 0, 0, 0};
MotorCmd_t g_motor_cmd = {0, 0};

// 定义全局标志位，用于通知弓字形任务有新速度到达
uint8_t g_new_speed_ready = 0;
BaseType_t g_can_communication_ok = pdTRUE; // CAN总线通信状态，默认为正常
TickType_t g_last_can_rx_tick = 0;          // CAN总线看门狗时间戳

QueueHandle_t motor_cmd_queue;
QueueHandle_t bluetooth_queue;
QueueHandle_t slave_data_queue;
SemaphoreHandle_t status_mutex;

// 串口接收中断用的 1 字节缓存
uint8_t BT_RxTemp;          // 蓝牙接收临时数据
uint8_t rx_byte;            // 从机接收临时数据

uint16_t debug_rx_count = 0; // 用来记录到底有没有收到字节


// ==========================================
// 1. OLED显示任务 (优先级最低：1，耗时最长，防止阻塞系统)
// ==========================================
void OLED_task(void *pvParameters);
#define OLED_TASK_STACK_SIZE 256      // sprintf格式化字符极费栈空间
#define OLED_TASK_PRIORITY 1          // 最低优先级
TaskHandle_t OLED_task_handle;
#define OLED_TASK_PERIOD 50           // 50ms刷新一次(20FPS)足够了，太快浪费CPU

// ==========================================
// 2. 超声波采集任务 (优先级中等：3)
// ==========================================
void sensor_task(void *pvParameters);
#define SENSOR_TASK_STACK_SIZE 128
#define SENSOR_TASK_PRIORITY 3
TaskHandle_t sensor_task_handle;
#define SENSOR_TASK_PERIOD 60         // HCSR04两次触发必须间隔 60ms 以上！

// ==========================================
// 3. MPU6050采集任务 (优先级中等：3)
// ==========================================
void mpu6050_task(void *pvParameters);
#define MPU6050_TASK_STACK_SIZE 128
#define MPU6050_TASK_PRIORITY 4
TaskHandle_t mpu6050_task_handle;
#define MPU6050_TASK_PERIOD 10        

// ==========================================
// 4. 蓝牙通讯任务 (优先级中等：3)
// ==========================================
void bluetooth_task(void *pvParameters);
#define BLUETOOTH_TASK_STACK_SIZE 128
#define BLUETOOTH_TASK_PRIORITY 3
TaskHandle_t bluetooth_task_handle;
#define BLUETOOTH_TASK_PERIOD 20      // 20ms轮询一次接收足够

// ==========================================
// 5. 电机驱动任务 (优先级中等：3)
// ==========================================
void motor_task(void *pvParameters);
#define MOTOR_TASK_STACK_SIZE 128
#define MOTOR_TASK_PRIORITY 3
TaskHandle_t motor_task_handle;
#define MOTOR_TASK_PERIOD 20      

// ==========================================
// 6. PID控制任务 (优先级高：4，时间极其敏感，确保PID周期稳定)
// ==========================================
void motor_PID_task(void *pvParameters);
#define MOTOR_PID_TASK_STACK_SIZE 128
#define MOTOR_PID_TASK_PRIORITY 4     // 高优先级
TaskHandle_t motor_PID_task_handle;
#define MOTOR_PID_TASK_PERIOD 10      // 保持10ms，标准的100Hz控制频率

// ==========================================
// 7. CAN通信任务 (优先级中等：3)
// ==========================================
void can_task(void *pvParameters);
#define CAN_TASK_STACK_SIZE 128
#define CAN_TASK_PRIORITY 3
TaskHandle_t can_task_handle;
#define CAN_TASK_PERIOD 20

/**
 * @brief 启动FreeRTOS操作系统
 * 
 */
void FreeRTOS_start(void)
{
    // ==========================================
    // 硬件与模块的集中初始化 (在RTOS启动前)
    // ==========================================
    OLED_Init();
    MPU6050_Data_Init();
    Motor_Init();
    PID_Init();
    MyCAN_Init();
    Mode_Init();

    // 初始化CAN看门狗时间戳，防止小车上电立刻急停
    g_can_communication_ok = pdTRUE; // 初始状态为正常
    g_last_can_rx_tick = xTaskGetTickCount();

    // 任务创建前，必须先创建队列和互斥量！
    // 假设蓝牙队列存32个字节，从机队列存64个字节，电机指令队列存10条
    bluetooth_queue = xQueueCreate(128, sizeof(uint8_t));
    slave_data_queue = xQueueCreate(64, sizeof(uint8_t));
    motor_cmd_queue = xQueueCreate(10, sizeof(MotorCmd_t));
    status_mutex = xSemaphoreCreateMutex();

    if(bluetooth_queue != NULL && slave_data_queue != NULL && status_mutex != NULL)
    {
        // 1.创建OLED显示任务
        xTaskCreate(OLED_task, "OLED_task", OLED_TASK_STACK_SIZE, NULL, OLED_TASK_PRIORITY, &OLED_task_handle);
        // 2.创建超波采集任务
        xTaskCreate(sensor_task, "sensor_task", SENSOR_TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, &sensor_task_handle);
        // 3.创建MPU6050采集任务
        xTaskCreate(mpu6050_task, "mpu6050_task", MPU6050_TASK_STACK_SIZE, NULL, MPU6050_TASK_PRIORITY, &mpu6050_task_handle);
        // 4.创建蓝牙通讯任务
        xTaskCreate(bluetooth_task, "bluetooth_task", BLUETOOTH_TASK_STACK_SIZE, NULL, BLUETOOTH_TASK_PRIORITY, &bluetooth_task_handle);
        // 5.创建电机驱动任务
        xTaskCreate(motor_task, "motor_task", MOTOR_TASK_STACK_SIZE, NULL, MOTOR_TASK_PRIORITY, &motor_task_handle);
        // 6.创建PID控制任务
        xTaskCreate(motor_PID_task, "motor_PID_task", MOTOR_PID_TASK_STACK_SIZE, NULL, MOTOR_PID_TASK_PRIORITY, &motor_PID_task_handle);
        // 7.创建CAN通信任务
        xTaskCreate(can_task, "can_task", CAN_TASK_STACK_SIZE, NULL, CAN_TASK_PRIORITY, &can_task_handle);

        // 开启蓝牙串口接收中断 (准备接收数据)
        HAL_UART_Receive_IT(&huart1, &BT_RxTemp, 1);

        // 启动调度器
        vTaskStartScheduler();
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* 1. 蓝牙串口 USART1 */
    if (huart->Instance == USART1)
    {
        debug_rx_count++; // 记录成功进入中断接收的次数

        // 将收到的 1 个字符放入蓝牙队列
        xQueueSendFromISR(bluetooth_queue, &BT_RxTemp, &xHigherPriorityTaskWoken);
        
        // 重新开启接收中断
        HAL_UART_Receive_IT(&huart1, &BT_RxTemp, 1);
    }
    // 如果队列操作唤醒了高优先级任务，立刻请求上下文切换
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



// ==========================================
// 1. OLED显示任务 (优先级最低：1，耗时最长，防止阻塞系统)
// ==========================================
void OLED_task(void *pvParameters)
{
    // 执行开机动画和硬件自检 (包含 vTaskDelay，必须在调度器启动后执行)
    OLED_startup_picture();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // 调用状态机动态显示界面
        OLED_show_status();
        
        // 任务延时
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(OLED_TASK_PERIOD));
    }
}


// ==========================================
// 2. 超声波采集任务 (优先级中等：3)
// ==========================================
void sensor_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // 超声波传感器任务主循环
        // TODO: 实现超声波传感器数据采集和处理逻辑
        g_car_status.ultrasonic_distance = HCSR04_GetDistance_Filtered();
        // 任务延时
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_TASK_PERIOD));
    }
}

// ==========================================
// 3. MPU6050采集任务 (优先级中等：3)
// ==========================================
void mpu6050_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // MPU6050传感器任务主循环
        MPU6050_Data_Update();
        
        // 任务延时
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MPU6050_TASK_PERIOD));
    }
}
// ==========================================
// 4. 蓝牙通讯任务 (优先级中等：3)
// ==========================================
void bluetooth_task(void *pvParameters)
{
    uint8_t bt_char;
    while (1)
    {
        // 1. 阻塞等待串口字节 (如果没有数据，任务直接休眠，不吃CPU)
        if (xQueueReceive(bluetooth_queue, &bt_char, portMAX_DELAY) == pdTRUE)
        {
            // 2. 拿到一个字节，直接扔进蓝牙数据处理黑盒
            Parse_Bluetooth_Protocol(bt_char); 
        }
    }
}

// ==========================================
// 5. 电机驱动任务 (优先级中等：3)
// ==========================================
void motor_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // 模式控制主状态机循环
        Mode_Task_Loop();

        // 任务延时
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MOTOR_TASK_PERIOD));
    }
}

// ==========================================
// 6. PID控制任务 (优先级高：4，时间极其敏感，确保PID周期稳定)
// ==========================================
void motor_PID_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // 物理层闭环驱动更新
        Motor_Execute_PID();

        // 任务延时
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MOTOR_PID_TASK_PERIOD));
    }
}



// ==========================================
// 7. CAN通信任务 (优先级中等：5)
// ==========================================
void can_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // CAN通信任务主循环
        Host_Process_CAN_Data();

        // 调用封装好的里程解算模块
        Odometry_Update();

        // 任务延时
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CAN_TASK_PERIOD));
    }
}


// ==========================================
// 8. UART错误回调处理
// ==========================================
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // STM32F1 系列 HAL 库专用的清除错误标志位宏
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        // 重新强行开启中断接收，满血复活
        HAL_UART_Receive_IT(&huart1, &BT_RxTemp, 1);
    }
}
