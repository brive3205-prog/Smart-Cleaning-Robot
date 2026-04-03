/* BlueTooth.h */
#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "main.h"
#include <stdint.h>

extern char BlueTooth_RxPacket[100];
extern uint8_t BlueTooth_RxFlag;

/* 函数声明 */
void BlueTooth_SendByte(uint8_t Byte);
void BlueTooth_SendArray(uint8_t *Array, uint16_t Length);
void BlueTooth_SendString(const char *String);
void BlueTooth_SendNumber(uint32_t Number, uint8_t Length);
void BlueTooth_Printf(const char *format, ...);
void BlueTooth_Init(void);       // 新增：启动串口中断接收

#endif
