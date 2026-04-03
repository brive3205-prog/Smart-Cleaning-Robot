#ifndef _RECORD_PLAY_TASK_H_
#define _RECORD_PLAY_TASK_H_

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

#include "App_Types.h"
#include "Flash.h"
#include "Motor_PID.h"

// 向外暴露的状态变量
extern RecordPlayState_t rp_state;
extern uint32_t record_start_tick;
extern uint32_t play_start_tick;

// 向外暴露的控制接口
void CMD_Start_Record(void);
void CMD_Stop_Record(uint8_t flag);
void CMD_Start_Play(uint8_t flag);
void Record_Key_Action(uint8_t key, uint8_t state);
void RecordPlay_Task_Loop(void);


#endif // _RECORD_PLAY_TASK_H_
