/**
 * @file    App_Types.h
 * @brief   系统全局数据结构、枚举定义及外部全局变量声明
 */

#ifndef __APP_TYPES_H__
#define __APP_TYPES_H__

#include "stdint.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

//结构体（Struct）、枚举（Enum）和宏定义（Macro）

// ==============================================================================================================================
// FreeRTOS 任务相关的类型定义
// ==============================================================================================================================


// ==========================================
// 1. 电机控制指令结构体 (供队列使用)
// ==========================================
typedef struct {
    uint8_t cmd_type;   // 指令类型：1=前进, 2=后退, 3=左转, 4=右转, 5=停止, 6=清扫
    int16_t cmd_value;  // 附加参数：PWM速度值或转向角度
} MotorCmd_t;

// ==========================================
// 2. 系统全局状态结构体 (供互斥锁保护)
// ==========================================
typedef struct {
    float ultrasonic_distance;  // 超声波距离 (cm)
    float warning_distance;     // 警告距离  
    float left_distance;        // 左侧距离传感器距离 (cm)
    float right_distance;       // 右侧距离传感器距离 (cm)
    uint8_t current_mode;       // 当前运行模式 (1:手动 2:避障 3:轨迹录制 4:轨迹规划)
    uint8_t running_status;     // 运行状态 (0:停止 1:前进 2:后退 3:左转 4:右转)
    int16_t current_speed;      // 当前设定速度
    int16_t current_speed_1;      // 当前电机1实际运行速度
    int16_t current_speed_2;      // 当前电机2实际运行速度
    float yaw;                    // 当前偏航角
    uint16_t current_length;         // 弓字型路径预设长度
    uint16_t current_width;          // 弓字型路径预设宽度
    uint8_t current_clean;           // 清扫状态
    // 里程与坐标变量 ⬇️
    float total_mileage;        // 记录小车运行的总里程 (单位：厘米 cm)
    float pos_x;                // 世界坐标系 X 坐标 (单位：厘米 cm)
    float pos_y;                // 世界坐标系 Y 坐标 (单位：厘米 cm)
    uint8_t power;              // 电池电量 (0-100)
} SystemStatus_t;

// 运行状态枚举
typedef enum {
    STOP = 0,       // 停止
    FORWARD,        // 前进
    BACKWARD,       // 后退
    LEFT,           // 左转
    RIGHT           // 右转
} RunningStatus_t;


//定义小车的四个工作模式枚举
typedef enum {
    MODE_MANUAL = 0,    // 手动模式
    MODE_AVOIDANCE,     // 避障模式
    MODE_RECORD_PLAY,   // 记录回放模式
    MODE_BOW_PATH       // 弓字形路径模式
} RobotMode_t;

// 避障专属子状态机枚举
typedef enum {
    OBS_MODE = 0,
    OBS_STATE_FORWARD = 0,     // 正常前进
    OBS_STATE_STOP,            // 刚发现障碍，刹车停稳
    OBS_STATE_LOOK_LEFT,       // 舵机正在转左等待
    OBS_STATE_LOOK_RIGHT,      // 舵机正在转右等待
    OBS_STATE_LOOK_CENTER,     // 舵机正在回正，并做决策
    OBS_STATE_TURN_LEFT,       // 决策结果：小车向左转
    OBS_STATE_TURN_RIGHT,      // 决策结果：小车向右转
    OBS_STATE_BACKWARD         // 决策结果：死胡同，小车后退
} AvoidanceState_t;

// ====================== 录制回放结构体 ======================
// 记录回放模式的子状态
typedef enum {
    RP_STATE_IDLE = 0,      // 空闲待机（等待蓝牙指令）
    RP_STATE_RECORDING,     // 正在录制
    RP_STATE_PLAYING        // 正在回放
} RecordPlayState_t;

// 总大小：8字节 (完美对齐)
#pragma pack(push, 1)
typedef struct {
    uint32_t time_ms;     // 相对时间戳（ms）
    uint8_t  key;         // 1=前进, 2=后退, 3=左转, 4=右转
    uint8_t  state;       // 1=按下, 0=松开
    int16_t  speed;       // 执行该动作时的速度
    int16_t  angle;       // 转向角度 (直行时设为0，转向松开时记录真实角度)
} Event;
#pragma pack(pop)
// ====================== 录制回放结构体 ======================

// ====================== 弓字形模式专属定义 ======================
typedef enum {
    BOW_STATE_IDLE = 0,        // 待机，等待蓝牙发来长宽数据
    BOW_STATE_FORWARD,         // 沿着“长”直行清扫
    BOW_STATE_TURN_1,          // 第 1 次 90 度转向
    BOW_STATE_STEP_SIDE,       // 沿着“宽”平移一小段（跨车道）
    BOW_STATE_TURN_2,          // 第 2 次 90 度转向
    BOW_STATE_RETURN_TURN,     // 返航：车头对准原点
    BOW_STATE_RETURN_DRIVE,    // 返航：直线开回原点
    BOW_STATE_RETURN_ALIGN,    // 返航到点后，原地转圈把车头回正到 0 度
    BOW_STATE_FINISH           // 清扫完成
} BowPathState_t;
// ====================== 弓字形模式专属定义 ======================



// ==========================================
// 3. 全局变量与操作句柄的外部声明 (Extern)
// ==========================================
// 这样在其他引入了此头文件的 .c 文件中，也能认出这些变量
extern SystemStatus_t g_car_status;  
extern MotorCmd_t g_motor_cmd;

// 如果你的外设驱动也需要往队列发数据，可以把句柄也 extern 出去
extern QueueHandle_t motor_cmd_queue;
extern SemaphoreHandle_t status_mutex;

extern BaseType_t g_can_communication_ok; // CAN总线通信状态，pdTRUE=正常，pdFALSE=中断
extern TickType_t g_last_can_rx_tick;    // CAN总线最后一次收到数据的Tick
extern QueueHandle_t bluetooth_queue;    //蓝牙数据队列
extern QueueHandle_t slave_data_queue;    //从机数据队列





// ==============================================================================================================================
// PID 相关的类型定义
// ==============================================================================================================================

// ==========================================
// 1. 定义位置式 PID 控制器结构体
// ==========================================
typedef struct {
    float Kp;             // 比例系数
    float Ki;             // 积分系数
    float Kd;             // 微分系数
    
    int16_t target_val;   // 目标速度
    int16_t actual_val;   // 实际速度
    
    int16_t err;          // 当前误差 e(k)
    int16_t err_last;     // 上次误差 e(k-1)
    
    float integral;       // 误差积分累加器 (使用float防止溢出)
    
    int16_t pwm_output;   // 最终输出给电机的 PWM 值
} PID_TypeDef;

// 外部声明 4 个轮子的 PID 结构体，供 RTOS 任务调用
extern PID_TypeDef pid_motor1;
extern PID_TypeDef pid_motor2;



// ==============================================================================================================================
// MPU6050 相关的类型定义
// ==============================================================================================================================

//角速度
typedef struct
{
    int16_t gyro_x; // 前为负,后为正
    int16_t gyro_y; // 右为正,左为负
    int16_t gyro_z; // 顺时针负,逆时针正
} Gyro_struct;
//加速度
typedef struct
{
    int16_t accel_x; // 左为正,右为负
    int16_t accel_y; // 前为负,后为正
    int16_t accel_z; // 朝上为正,朝下为负
} Accel_struct;

typedef struct
{
    Gyro_struct gyro;
    Accel_struct accel;
} Gyro_Accel_Struct;

//解算得到的欧拉角
typedef struct
{
    float yaw;
    float pitch;
    float roll;
} Euler_struct;



// ==============================================================================================================================
// CAN通信 相关的类型定义
// ==============================================================================================================================

// 定义 CAN 通信 ID
#define CAN_ID_SLAVE_SPEED   0x201  // 从机上报：4个速度
#define CAN_ID_SLAVE_POWER   0x202  // 从机上报：电池电量
#define CAN_ID_HOST_LED      0x101  // 主机下发：LED 控制指令

// 1. 速度上报共用体 (占用 4 字节)
typedef union {
    int16_t speeds[2]; // speed[0]和[1]分别对应电机1和2 (左右轮)
    uint8_t bytes[4];
} CAN_SpeedPayload_t;

// 3. LED 控制共用体 (占用 3 字节)
typedef union {
    struct {
        uint8_t led1;
        uint8_t led2;
        uint8_t led3;
    } leds;
    uint8_t bytes[3];
} CAN_LEDPayload_t;





#endif // __APP_TYPES_H_
