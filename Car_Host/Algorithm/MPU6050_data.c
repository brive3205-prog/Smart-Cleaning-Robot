/**
 * @file    MPU6050_data.c
 * @brief   MPU6050姿态传感器数据读取、死区滤波及偏航角(Yaw)积分算法
 */

#include "MPU6050_data.h"

// 采样周期：必须和你调用 MPU6050_Data_Update 的时间间隔一致！
// 比如你放在一个 vTaskDelay(10) 的任务里，dt 就是 0.01 秒
#define DT 0.01f  

// 角速度灵敏度：根据 MPU6050.c 中设置的 ±2000°/s，换算系数为 16.4 LSB/(°/s)
#define GYRO_SCALE 16.4f 

void MPU6050_Data_Init(void)
{
    MPU6050_Init();
    g_car_status.yaw = 0.0f; // 初始化时偏航角清零
}

/**
 * @brief 读取数据并进行滤波与积分姿态解算
 * @note  务必保证调用频率稳定（推荐 100Hz，即 10ms 调用一次）
 */
void MPU6050_Data_Update(void)
{
    Gyro_Accel_Struct raw_data;
    
    // 1. 获取原始数据
    MPU6050_Get_Data(&raw_data);

    // 2. 将 Z 轴原始数据转换为真实的角速度 (单位：度/秒)
    float gyro_z_dps = (float)raw_data.gyro.gyro_z / GYRO_SCALE;

    // 3. 死区滤波：滤除静态噪点，防止角度缓慢累加漂移
    if (gyro_z_dps > -0.3f && gyro_z_dps < 0.3f) {
        gyro_z_dps = 0.0f;
    }

    // 4. 积分算角度：当前角度 = 之前的角度 + (角速度 * 时间)
    // 使用临界区保护对全局变量的写操作，防止被其他任务打断
    taskENTER_CRITICAL();
    g_car_status.yaw += gyro_z_dps * DT;
    taskEXIT_CRITICAL();
    
    // (可选) 如果你希望角度在 0~360 之间循环，可以在这里加判断；
    // 但对于我们的车，记录无限累加的角度其实更好计算相对转向差。
}


/**
 * @brief 硬件在线自检函数
 * @return 0: 成功, 1: 失败
 * @note 通过读取 WHO_AM_I 寄存器来判断设备是否在线
 */
uint8_t MPU6050_Check(void)
{
    // 读取 MPU6050 的 WHO_AM_I 寄存器 (地址 0x75)，正常返回值应为 0x68
    // 注意：请确保你的底层驱动中确实有 MPU6050_Read_Byte 这个读取函数
    if (MPU6050_Read_Byte(0x75) == 0x70) 
    {
        return 0; // 成功
    }
    return 1; // 失败
}
