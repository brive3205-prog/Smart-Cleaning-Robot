/**
 * @file    mode_control.c
 * @brief   系统工作模式(手动/避障/录放/寻路)切换调度总控及安全看门狗
 */

#include "mode_control.h"

// 内部函数声明（每个模式的具体逻辑）
static void Task_Manual(void);
static void Task_Avoidance(void);
static void Task_RecordPlay(void);
static void Task_BowPath(void);

extern BaseType_t g_can_communication_ok; 
extern TickType_t g_last_can_rx_tick;

/**
 * @brief 模式控制初始化  默认为手动模式
 */
void Mode_Init(void)
{
    g_car_status.current_mode = MODE_MANUAL;
}


/**
 * @brief 设置当前模式
 * @param mode 要切换到的模式
 */
void Mode_SetCurrent(RobotMode_t mode)
{
    if (g_car_status.current_mode != mode) 
    {
        // 切换模式前，先把电机停下，保证安全
        Motor_stop();
        //如果要切入特定模式，先把它的子状态机复位！
        if (mode == MODE_AVOIDANCE) 
        {
            obs_state = OBS_STATE_FORWARD; // 强制复位避障状态
            state_start_tick = 0;
        }
        else if (mode == MODE_RECORD_PLAY) 
        {
            // 以后如果有记录回放的状态变量，也在这里复位
            rp_state = RP_STATE_IDLE;
            record_start_tick = 0;
            play_start_tick = 0;
        }
        g_car_status.current_mode = mode;
    }
}


/**
 * @brief 模式任务循环
 */
void Mode_Task_Loop(void)
{
    // ==================================================
    // CAN总线掉线安全保护机制 (Watchdog)
    // ==================================================
    if ((xTaskGetTickCount() - g_last_can_rx_tick) > pdMS_TO_TICKS(200))
    {
        Motor_stop(); // 如果超过200ms没收到任何速度包，则强制停车
        g_can_communication_ok = pdFALSE; // 标记CAN通信中断
        return;       // 并且不再执行下面的任何模式逻辑
    }

    switch(g_car_status.current_mode)
    {
        case MODE_MANUAL:
            Task_Manual();
            break;
        case MODE_AVOIDANCE:
            Task_Avoidance();
            break;
        case MODE_RECORD_PLAY:
            Task_RecordPlay();
            break;
        case MODE_BOW_PATH:
            Task_BowPath();
            break;
    }
}



/**
 * @brief 手动模式任务
 */
static void Task_Manual(void)
{
    // 手动模式下，直接根据运行状态控制电机
    switch(g_car_status.running_status)
    {
        case FORWARD:
            Motor_advance(50);
            break;
        case BACKWARD:
            Motor_back(50);
            break;
        case LEFT:
            Motor_turn_left(50);
            break;
        case RIGHT:
            Motor_turn_right(50);
            break;
        case STOP:
            Motor_stop();
            break;
    }
    // 手动模式下，根据清洁状态控制清洁电机
    switch (g_car_status.current_clean)
    {
    case 1:
        Motor_clean_start();
        break;
    case 0:
        Motor_clean_stop();
        break;
    }
}

/**
 * @brief 避障模式任务
 */
static void Task_Avoidance(void)
{
    Avoidance_Task_Loop();
}

/**
 * @brief 轨迹录制与回放模式任务
 */
static void Task_RecordPlay(void)
{
    RecordPlay_Task_Loop();
}

/**
 * @brief 弓字形轨迹规划模式任务
 */
static void Task_BowPath(void)
{
    // 根据坐标点，执行闭环跟踪
    Path_Follow_Routine();
}
