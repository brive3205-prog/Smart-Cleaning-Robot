#ifndef BOW_PATH_TASK_H
#define BOW_PATH_TASK_H

#include <stdint.h>
#include "App_Types.h" // 确保能引用到 g_car_status
#include "Motor_PID.h" // 包含电机底层驱动
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include "Odometry.h"

#define MAX_WAYPOINTS 50  // 预留 50 个拐点，足够一般的房间使用了

// 航点结构体
typedef struct {
    float x;
    float y;
} Waypoint_t;

// 暴露给外部的变量和函数
extern Waypoint_t path_list[MAX_WAYPOINTS];
extern uint8_t total_points;
extern uint8_t current_target;
extern BowPathState_t bow_state;

void Path_Generate_Bow(float length, float width);
void Path_Follow_Routine(void);

#endif // BOW_PATH_TASK_H
