#ifndef _MPU6050_DATA_H_
#define _MPU6050_DATA_H_

#include "App_Types.h"
#include "MPU6050.h"

// ==========================================
// 暴露给外部的接口
// ==========================================

// 初始化传感器和解算参数
void MPU6050_Data_Init(void);

// 核心解算更新函数（请在 FreeRTOS 的某个定时任务中以固定频率调用，比如每 10ms）
void MPU6050_Data_Update(void);

// 【新增】硬件自检函数
uint8_t MPU6050_Check(void);

#endif // _MPU6050_DATA_H_
