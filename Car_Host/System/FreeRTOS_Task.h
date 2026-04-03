#ifndef _FREERTOS_TASK_H
#define _FREERTOS_TASK_H

#include <stdio.h>      // <-- 确保这行存在，解决 sscanf 警告
#include <string.h>     // 解决 strtok, strcmp 等警告

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"    
#include "semphr.h"

#include "App_Types.h"    
#include "usart.h"  
#include "Bluetooth_data.h"
#include "OLED.h"
#include "HCSR04.h"
#include "OLED_Show_Task.h"
#include "Motor.h"
#include "MPU6050_data.h"
#include "Motor_PID.h"
#include "Host_CAN_App.h"
#include "MyCAN.h"
#include "Odometry.h"
void FreeRTOS_start(void);

#endif
