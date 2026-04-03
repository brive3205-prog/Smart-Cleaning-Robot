#ifndef __HCSR04_H
#define __HCSR04_H

#include "main.h"
#include "tim.h"
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"



float HCSR04_GetDistance(void);

float HCSR04_GetDistance_Filtered(void);

#endif
