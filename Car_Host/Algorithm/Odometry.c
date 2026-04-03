/**
 * @file    Odometry.c
 * @brief   基于底盘双轮编码器和陀螺仪航向角的航迹推算(里程计)算法
 */

#include "Odometry.h"
#include <math.h>  // 用于 cos, sin, fabs

// 引入外部的速度就绪标志位
extern uint8_t g_new_speed_ready; 

// ==========================================
// 机器人底盘物理参数配置区 (修改这里即可适配不同小车)
// ==========================================
#define WHEEL_PERIMETER_CM  21.2f     // 车轮走一圈的物理周长 (单位: 厘米)
#define PULSE_PER_REV       1900.0f   // 车轮走一圈对应的编码器脉冲数
#define TIME_RATIO          0.4f      // 积分时间补偿倍率 (若主机CAN接收周期是20ms，从机测速周期是10ms，则为 2.0)
#define PI                  3.14159265f // 圆周率


/**
 * @brief  里程计初始化 (清零)
 */
void Odometry_Init(void)
{
    if(status_mutex != NULL && xSemaphoreTake(status_mutex, portMAX_DELAY) == pdTRUE)
    {
        g_car_status.total_mileage = 0.0f;
        g_car_status.pos_x = 0.0f;
        g_car_status.pos_y = 0.0f;
        // 注意：yaw 是 MPU6050 硬件解算出来的，一般由传感器那边去清零，这里不强制修改 yaw
        xSemaphoreGive(status_mutex);
    }
}


/**
 * @brief  里程与坐标实时更新函数
 */
void Odometry_Update(void)
{
    // 只有当接收到来自从机的最新 CAN 速度数据时，才进行一次积分推算
    if (g_new_speed_ready == 1)
    {
        g_new_speed_ready = 0; // 消耗掉该标志位

        // 1. 求左右轮子的平均“脉冲速度” (取平均值能一定程度上抵消打滑误差)
        float avg_pulse_speed = (g_car_status.current_speed_1 +
                                 g_car_status.current_speed_2) / 2.0f;

        // 2. 算出一个脉冲代表多少厘米
        float pulse_to_cm_ratio = WHEEL_PERIMETER_CM / PULSE_PER_REV;

        // 3. 计算在这一个积分周期内，小车前进的微小物理距离 (cm)
        float delta_distance = avg_pulse_speed * pulse_to_cm_ratio * TIME_RATIO;

        // 4. 互斥锁保护，更新到全局状态机
        if(status_mutex != NULL && xSemaphoreTake(status_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            // --- A. 累加总里程 (齿轮磨损量/总行驶路程) ---
            // 无论前进还是后退，里程都在增加，所以使用 fabs() 取绝对值
            g_car_status.total_mileage += fabs(delta_distance);

            // --- B. 累加 X/Y 平面世界坐标 ---
            // C语言标准库的三角函数要求传入弧度制 (Radian)，所以要把角度转换为弧度
            float yaw_radians = g_car_status.yaw * PI / 180.0f;
            
            // 矢量分解：累加当前微小位移在 X 和 Y 轴上的投影
            g_car_status.pos_x += delta_distance * cos(yaw_radians);
            g_car_status.pos_y += delta_distance * sin(yaw_radians);

            xSemaphoreGive(status_mutex);
        }
    }
}
