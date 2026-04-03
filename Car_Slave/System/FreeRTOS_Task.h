#ifndef _FREERTOS_TASK_H
#define _FREERTOS_TASK_H

#include <stdio.h>      // <-- 确保这行存在，解决 sscanf 警告
#include <string.h>     // 解决 strtok, strcmp 等警告

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"    
#include "semphr.h"

#include "App_Types.h"   
#include "Encoder.h"
#include "Slave_CAN_App.h"
#include "MyCAN.h"
#include "ADC_Power.h"

void FreeRTOS_start(void);

#endif
