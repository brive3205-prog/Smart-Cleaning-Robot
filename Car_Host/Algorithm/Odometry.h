#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

#include "App_Types.h" 
#include "FreeRTOS.h"
#include "semphr.h"

// ==========================================
// 里程计与坐标系解算 API
// ==========================================

/**
 * @brief  里程计初始化 (将物理坐标和里程清零)
 * @note   可以在开机时调用，或者在每次开始“弓字形”清扫前调用
 */
void Odometry_Init(void);

/**
 * @brief  里程与坐标实时更新函数 (航位推测)
 * @note   必须在主机的 CAN 接收任务 (can_task) 中高频循环调用
 */
void Odometry_Update(void);

#endif // __ODOMETRY_H__
