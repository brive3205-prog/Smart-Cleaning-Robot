#ifndef _SLAVE_CAN_APP_H_
#define _SLAVE_CAN_APP_H_

#include "App_Types.h"
#include "can.h" // 保留 can.h，因为接收中断回调函数的参数里有 CAN_HandleTypeDef

/**
 * @brief 从机发送 2 个轮子的速度给主机
 * @note  将在从机的 FreeRTOS 定时任务中被循环调用
 */
void Slave_Send_Speed_Data(void);

/**
 * @brief 从机发送电池电量给主机
 * @note  将在从机的 FreeRTOS ADC 任务中被调用
 */
void Slave_Send_Power_Data(void);

#endif // _SLAVE_CAN_APP_H_
