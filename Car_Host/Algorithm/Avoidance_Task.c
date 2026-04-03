/**
 * @file    Avoidance_Task.c
 * @brief   避障模式核心逻辑与有限状态机(FSM)实现
 */

#include "Avoidance_Task.h"

//避障模式任务
AvoidanceState_t obs_state = OBS_STATE_FORWARD; // 静态变量保存当前状态
TickType_t state_start_tick = 0;              // 记录进入某个状态的时间戳


/**
 * @brief 避障模式核心大循环
 */
void Avoidance_Task_Loop(void)
{
     g_car_status.warning_distance = 30;

    switch(obs_state)
    {
        // ==========================================
        // 1. 巡航前进阶段
        // ==========================================
        case OBS_STATE_FORWARD:
            if(g_car_status.ultrasonic_distance <= g_car_status.warning_distance)
            {
                // 如果距离小于警告距离 则停止前进 进入避障状态
                Motor_stop();
                obs_state = OBS_STATE_STOP;
                state_start_tick = xTaskGetTickCount();
            }
            else
            {
                // 注意：在安全距离内必须持续下发前进指令，防止在避障结束切回 FORWARD 状态后小车停在原地
                Motor_advance(g_car_status.current_speed);
            }
            break;
        // ==========================================
        // 2. 刹车停稳 & 看左边
        // ==========================================
        case OBS_STATE_STOP:
            // 等待 100ms 让车轮彻底停转消除惯性，然后让舵机看左边
            if((xTaskGetTickCount() - state_start_tick) >= pdMS_TO_TICKS(100))
            {
                Engine_SetAngle(180); // 舵机左转
                obs_state = OBS_STATE_LOOK_LEFT;
                state_start_tick = xTaskGetTickCount(); // 刷新时间戳
            }
            break;
        // ==========================================
        // 3. 记录左边 & 看右边
        // ==========================================
        case OBS_STATE_LOOK_LEFT:
            if((xTaskGetTickCount() - state_start_tick) >= pdMS_TO_TICKS(500))
            {
                g_car_status.left_distance = g_car_status.ultrasonic_distance; // 记录左侧距离
                
                // 从180度直接转到0度看右边
                Engine_SetAngle(0); 
                obs_state = OBS_STATE_LOOK_RIGHT;
                state_start_tick = xTaskGetTickCount();
            }
            break;
        // ==========================================
        // 4. 记录右边 & 舵机回正
        // ==========================================
        case OBS_STATE_LOOK_RIGHT:
            // 从左边(180)扫到右边(0)路程长，我们给 1000ms 的等待时间
            if((xTaskGetTickCount() - state_start_tick) >= pdMS_TO_TICKS(1000))
            {
                g_car_status.right_distance = g_car_status.ultrasonic_distance; // 记录右侧距离
                
                Engine_SetAngle(90); // 舵机回正
                obs_state = OBS_STATE_LOOK_CENTER;
                state_start_tick = xTaskGetTickCount();
            }
            break; 
        // ==========================================
        // 5. 等待回正 & 逻辑决策
        // ==========================================
        case OBS_STATE_LOOK_CENTER:
            if((xTaskGetTickCount() - state_start_tick) >= pdMS_TO_TICKS(500))
            {
                // ========== 开始判断走哪边 ==========
                // 距离判断使用 >= 防止左右两边距离相等时，错误地落入死胡同逻辑
                if(g_car_status.left_distance >= g_car_status.right_distance && g_car_status.left_distance > g_car_status.warning_distance)
                {
                    obs_state = OBS_STATE_TURN_LEFT;
                    state_start_tick = xTaskGetTickCount();
                    Motor_turn_left(g_car_status.current_speed);
                }
                else if(g_car_status.right_distance > g_car_status.left_distance && g_car_status.right_distance > g_car_status.warning_distance)
                {
                    obs_state = OBS_STATE_TURN_RIGHT;
                    state_start_tick = xTaskGetTickCount();
                    Motor_turn_right(g_car_status.current_speed);
                }
                else 
                {
                    // 左右两边都有障碍（死胡同），直接强制转向脱离，防止后退后死循环撞墙
                    obs_state = OBS_STATE_TURN_LEFT; 
                    state_start_tick = xTaskGetTickCount();
                    Motor_turn_left(g_car_status.current_speed);
                }
            }
            break; 
        // ==========================================
        // 6. 执行车身转向（给点时间让车身转过去）
        // ==========================================
        case OBS_STATE_TURN_LEFT:
        case OBS_STATE_TURN_RIGHT:
        case OBS_STATE_BACKWARD:
            // 假设小车转向或后退需要 600ms 来脱离障碍物范围
            if((xTaskGetTickCount() - state_start_tick) >= pdMS_TO_TICKS(600))
            {
                Motor_stop(); // 动作完成，停一下
                obs_state = OBS_STATE_FORWARD; // 重新切回正常前进巡航！
            }
            break;
        default:
            obs_state = OBS_STATE_FORWARD;
            break;
    }
    
}
