#ifndef __MODE_CONTROL_H__
#define __MODE_CONTROL_H__

#include "FreeRTOS.h"
#include "task.h"

#include "App_Types.h"
#include "Motor_PID.h"
#include "Engine.h"
#include "Record_Play_Task.h"
#include "Bow_Path_Task.h"
#include "Avoidance_Task.h"
#include "Odometry.h"


/**
 * @brief 模式控制初始化  默认为手动模式
 */
void Mode_Init(void);

/**
 * @brief 设置当前模式
 * @param mode 要切换到的模式
 */
void Mode_SetCurrent(RobotMode_t mode);


/**
 * @brief 模式任务循环
 */
void Mode_Task_Loop(void);

#endif // __MODE_CONTROL_H__
