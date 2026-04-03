/**
 * @file    Motor_PID.c
 * @brief   电机位置式PID闭环控制、角度环航向补偿及底层运动学解算
 */

#include "Motor_PID.h"

// ==========================================
// 宏定义与可调参数
// ==========================================
#define SPEED_MAPPING_FACTOR   3      // 速度映射因子 (0~100 -> 0~300)
#define YAW_CORRECTION_KP      20.0f  // 航向角P控制器比例系数
#define YAW_COMP_SPEED_MAX     150    // 航向角补偿速度上限


// 实例化 2 个轮子的 PID 控制器
PID_TypeDef pid_motor1;
PID_TypeDef pid_motor2;


uint8_t is_straight_mode = 0; // 是否处于直线锁定模式 (1=是, 0=否)
float target_yaw = 0.0f;      // 锁定的目标航向角


/**
 * @brief 初始化 2 个轮子的 PID 参数
 */
void PID_Init(void)
{
    // 位置式 PID 的参数量级和增量式不同，Ki 通常比较小
    float p = 0.2f, i = 0.045f, d = 0.0f; 
    
    PID_TypeDef* pids[] = {&pid_motor1, &pid_motor2};
    
    for(int j = 0; j < 2; j++)
    {
        pids[j]->Kp = p;
        pids[j]->Ki = i;
        pids[j]->Kd = d;
        pids[j]->target_val = 0;
        pids[j]->actual_val = 0;
        pids[j]->err = 0;
        pids[j]->err_last = 0;
        pids[j]->integral = 0.0f;  // 初始化积分清零
        pids[j]->pwm_output = 0;
    }
}




/**
 * @brief 位置式 PID 核心计算算法
 */
int16_t PID_Realize(PID_TypeDef *pid, int16_t actual_speed)
{
    // 零点死区强断：设定速度为0时直接切断输出
    if (pid->target_val == 0) 
    {
        pid->err = 0;
        pid->err_last = 0;
        pid->integral = 0.0f; // 注意：停车时必须清空积分，否则下次起步会不受控狂飙
        pid->pwm_output = 0; 
        return 0;            
    }

    pid->actual_val = actual_speed;
    pid->err = pid->target_val - pid->actual_val; 

    // 1. 积分累加
    pid->integral += pid->err;

    // 积分限幅 (Anti-Windup)
    // 这里限制积分的绝对上限，防止电机堵转或被外力卡住时误差无限放大
    float max_integral = 1000.0f; // 根据你的 Ki 大小可微调此值
    if (pid->integral > max_integral)  pid->integral = max_integral;
    if (pid->integral < -max_integral) pid->integral = -max_integral;

    // 2. 位置式 PID 公式
    // P 负责当前差距，I 负责消除稳态误差(低频发力)，D 负责抵抗速度突变(阻尼)
    float output = pid->Kp * pid->err + 
                   pid->Ki * pid->integral + 
                   pid->Kd * (pid->err - pid->err_last);

    // 3. 最终输出限幅 (电机 PWM 只能在 -100 到 100 之间)
    if (output > 100.0f)  output = 100.0f;
    if (output < -100.0f) output = -100.0f;

    // 4. 更新历史状态
    pid->err_last = pid->err;
    pid->pwm_output = (int16_t)output;

    return pid->pwm_output;
}

/**
 * @brief 执行 2 个轮子的 PID 计算 (加入航向锁定功能)
 */
void Motor_Execute_PID(void)
{
    static float filter_v1 = 0.0f;
    static float filter_v2 = 0.0f;
    float alpha = 0.15f; 

    // 在临界区内快速拷贝需要用到的共享数据
    int16_t local_speed_1;
    int16_t local_speed_2;
    float local_yaw;
    taskENTER_CRITICAL();
    local_speed_1 = g_car_status.current_speed_1;
    local_speed_2 = g_car_status.current_speed_2;
    local_yaw = g_car_status.yaw;
    taskEXIT_CRITICAL();
    

    // 1. 速度滤波
    // 后续所有计算都使用局部变量，确保数据一致性
    filter_v1 = (alpha * local_speed_1) + ((1.0f - alpha) * filter_v1);
    filter_v2 = (alpha * local_speed_2) + ((1.0f - alpha) * filter_v2);

    // ==========================================
    // 角度环 (航向锁定补偿计算)
    // ==========================================
    int16_t comp_speed = 0;
    if (is_straight_mode == 1)
    {
        // 计算车头偏离了多少度
        float angle_err = target_yaw - local_yaw;
        
        // 处理 180 度和 -180 度的边界跳变
        while (angle_err > 180.0f)  angle_err -= 360.0f;
        while (angle_err < -180.0f) angle_err += 360.0f;

        // 角度环的比例系数 (Kp_yaw)。如果修正力度太弱就调大，车身左右乱甩就调小
        comp_speed = (int16_t)(angle_err * YAW_CORRECTION_KP);
        
        // 限幅：防止补偿速度过大，一瞬间把车身打转
        if (comp_speed > YAW_COMP_SPEED_MAX) comp_speed = YAW_COMP_SPEED_MAX;
        if (comp_speed < -YAW_COMP_SPEED_MAX) comp_speed = -YAW_COMP_SPEED_MAX;
    }

    // 提取基准速度
    int16_t base_t1 = pid_motor1.target_val;
    int16_t base_t2 = pid_motor2.target_val;

    // 动态叠加角度补偿
    if (is_straight_mode == 1)
    {
        // 必须判断前后方向，倒车时补偿量需要反转，否则车尾会越甩越偏
        if (base_t1 >= 0) 
        {
            pid_motor1.target_val = base_t1 - comp_speed; // 前进：左轮减速
            pid_motor2.target_val = base_t2 + comp_speed; // 前进：右轮加速
        } 
        else 
        {
            pid_motor1.target_val = base_t1 + comp_speed; // 后退：左轮加(变小)
            pid_motor2.target_val = base_t2 - comp_speed; // 后退：右轮减(变负)
        }
    }

    // 2. 把最终目标速度和实际速度喂给 PID
    int16_t pwm_1 = PID_Realize(&pid_motor1, (int16_t)filter_v1);
    int16_t pwm_2 = PID_Realize(&pid_motor2, (int16_t)filter_v2);

    // 注意：算完 PWM 后，立刻把目标速度恢复为基准速度，防止补偿值无限累加
    if (is_straight_mode == 1)
    {
        pid_motor1.target_val = base_t1;
        pid_motor2.target_val = base_t2;
    }

    // 3. 下发给物理层驱动
    Motor_SetSpeed(1, (int8_t)pwm_1);
    Motor_SetSpeed(2, (int8_t)pwm_2);
}


// ==========================================
// 运动学解算：映射目标速度 (0~100 -> 0~300)
// ==========================================

void Motor_advance(int16_t target_speed) 
{
    int16_t mapped_speed = target_speed * SPEED_MAPPING_FACTOR;  
    
    // 如果刚开始起步，瞬间锁定当前的车头角度！
    if (is_straight_mode == 0) 
    {
        target_yaw = g_car_status.yaw;
        is_straight_mode = 1; 
    }

    pid_motor1.target_val = mapped_speed;
    pid_motor2.target_val = mapped_speed;
}

void Motor_back(int16_t target_speed) 
{
    int16_t mapped_speed = target_speed * SPEED_MAPPING_FACTOR;
    
    // 后退也可以锁定航向
    if (is_straight_mode == 0) {
        target_yaw = g_car_status.yaw;
        is_straight_mode = 1; 
    }

    pid_motor1.target_val = -mapped_speed;
    pid_motor2.target_val = -mapped_speed;
}

void Motor_turn_left(int16_t target_speed) 
{
    is_straight_mode = 0; // 转弯必须解除航向锁定
    int16_t mapped_speed = target_speed * SPEED_MAPPING_FACTOR;
    pid_motor1.target_val = -mapped_speed; // 左轮后退
    pid_motor2.target_val = mapped_speed;  // 右轮前进
}

void Motor_turn_right(int16_t target_speed) 
{
    is_straight_mode = 0; // 转弯必须解除航向锁定
    int16_t mapped_speed = target_speed * SPEED_MAPPING_FACTOR;
    pid_motor1.target_val = mapped_speed;  // 左轮前进
    pid_motor2.target_val = -mapped_speed; // 右轮后退
}

void Motor_stop(void) 
{
    is_straight_mode = 0; // 停车解除锁定
    pid_motor1.target_val = 0;
    pid_motor2.target_val = 0;
}

void Motor_clean_start(void)
{  
    Motor_SetSpeed(5, 50);
    Vacuum_Click();
}

void Motor_clean_stop(void)
{
    Motor_SetSpeed(5, 0);
    Vacuum_Click();
}




/**
 * @brief 通用角度闭环转向函数 (阻塞式，但会出让 CPU 资源)
 * @param speed 转向速度
 * @param target_angle 目标角度 (如 90.0, 180.0)
 * @param dir 转向方向 (1=右转, 2=左转)
 */
void Motor_Turn_Angle(int16_t speed, float target_angle, uint8_t dir)
{
    // 1. 获取转弯前的绝对初始角度（绝不能清零！）
    float start_yaw = g_car_status.yaw;
    float stop_offset = 0.0f;
    
    // 2. 下达底层物理转动指令，并计算目标绝对角度
    if(dir == 1) 
    { 
        // 右转 (MPU6050 中右转偏航角是减小的)
        stop_offset = 3.0f;
        Motor_turn_right(speed);
        // 阻塞等待：当前角度 一直减小到 (初始角度 - 目标角度 + 补偿量)
        while (g_car_status.yaw > (start_yaw - target_angle + stop_offset))
        {
            vTaskDelay(pdMS_TO_TICKS(10)); 
        }
    }
    else 
    { 
        // 左转 (MPU6050 中左转偏航角是增加的)
        stop_offset = 7.0f;
        Motor_turn_left(speed); 
        // 阻塞等待：当前角度 一直增加到 (初始角度 + 目标角度 - 补偿量)
        while (g_car_status.yaw < (start_yaw + target_angle - stop_offset))
        {
            vTaskDelay(pdMS_TO_TICKS(10)); 
        }
    }

    // 3. 角度到了，立刻刹车并留出余震时间！
    Motor_stop();
    vTaskDelay(pdMS_TO_TICKS(300));
}

// ==========================================
// 傻瓜式调用接口
// ==========================================

// 原地左转 90 度
void Motor_Turn_Left_90(int16_t speed) {
    Motor_Turn_Angle(speed, 90.0f, 2);
}

// 原地右转 90 度
void Motor_Turn_Right_90(int16_t speed) {
    Motor_Turn_Angle(speed, 90.0f, 1);
}

// 原地左转 180 度 (掉头)
void Motor_Turn_Left_180(int16_t speed) {
    Motor_Turn_Angle(speed, 180.0f, 2);
}

// 原地右转 180 度 (掉头)
void Motor_Turn_Right_180(int16_t speed) {
    Motor_Turn_Angle(speed, 180.0f, 1);
}
