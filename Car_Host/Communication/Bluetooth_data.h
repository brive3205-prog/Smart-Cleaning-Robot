#ifndef _BLUETOOTH_DATA_H_
#define _BLUETOOTH_DATA_H_

// 引入全局类型和句柄声明
#include "App_Types.h"  
#include "mode_control.h"
#include "OLED.h"
#include "Motor.h"
#include "Engine.h"
#include "Motor_PID.h"
// 引入字符串处理标准库
#include <string.h>    
#include <stdlib.h> 

/**
 * @brief 蓝牙应用层指令解析与分发
 * @param packet 传入的完整有效字符串，例如 "[key,qianjin,down]" 或 "[mode,bizhang]"
 */
void BT_Execute_Command(char *packet);

/**
 * @brief 蓝牙串口接收中断解析协议
 * @param bt_char 接收到的单个字符
 */
void Parse_Bluetooth_Protocol(uint8_t bt_char);

// 【新增】硬件自检函数
uint8_t Bluetooth_Check(void);

#endif // _BLUETOOTH_DATA_H_
