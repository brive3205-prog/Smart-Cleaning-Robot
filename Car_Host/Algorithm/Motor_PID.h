#ifndef __MOTOR_PID_H__
#define __MOTOR_PID_H__

#include "Motor.h"
#include <stdint.h>
#include <stdlib.h>
#include "App_Types.h"
#include "Vacuum.h"


// ==========================================
// 核心控制函数声明
// ==========================================
void PID_Init(void);
int16_t PID_Realize(PID_TypeDef *pid, int16_t actual_speed);

// ==========================================
// 运动学解算函数
// ==========================================

void Motor_Execute_PID(void);

void Motor_advance(int16_t target_speed);
void Motor_back(int16_t target_speed);
void Motor_turn_left(int16_t target_speed);
void Motor_turn_right(int16_t target_speed);
void Motor_clean_start(void);
void Motor_clean_stop(void);
void Motor_stop(void);




void Motor_Turn_Angle(int16_t speed, float target_angle, uint8_t dir);
void Motor_Turn_Left_90(int16_t speed);
void Motor_Turn_Right_90(int16_t speed);
void Motor_Turn_Left_180(int16_t speed);
void Motor_Turn_Right_180(int16_t speed);





#endif // __MOTOR_PID_H__
