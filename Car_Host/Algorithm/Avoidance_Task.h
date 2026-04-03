#ifndef AVOIDANCE_TASK_H
#define AVOIDANCE_TASK_H

#include "App_Types.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Motor_PID.h"
#include "Engine.h"


extern AvoidanceState_t obs_state;
extern uint32_t state_start_tick;



/**
 * @brief 避障模式核心大循环
 */
void Avoidance_Task_Loop(void);




#endif // AVOIDANCE_TASK_H
