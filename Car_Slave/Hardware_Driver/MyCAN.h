#ifndef __MYCAN_H
#define __MYCAN_H

#include "main.h" // 引入 HAL 库必需的头文件
#include "can.h" // 必须包含此头文件，以调用 CubeMX 生成的 hcan 结构体

void MyCAN_Init(void);
void MyCAN_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data);
uint8_t MyCAN_ReceiveFlag(void);
void MyCAN_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data);

#endif
