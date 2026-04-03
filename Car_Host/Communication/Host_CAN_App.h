#ifndef _HOST_CAN_APP_H_
#define _HOST_CAN_APP_H_

#include "App_Types.h"
// 移除了 #include "can.h"，因为解耦后应用层不需要直接接触 HAL 库句柄

/**
 * @brief 主机轮询解析 CAN 接收数据
 * @note  放在 FreeRTOS 的接收任务中循环调用
 */
void Host_Process_CAN_Data(void);

/**
 * @brief 主机通过 CAN 发送 LED 控制指令
 */
void Host_Send_LED_Cmd_CAN(uint8_t led1, uint8_t led2, uint8_t led3);

#endif // _HOST_CAN_APP_H_

