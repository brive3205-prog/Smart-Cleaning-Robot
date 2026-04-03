/**
 * @file    Bow_Path_Task.c
 * @brief   弓字形自主路径规划、坐标监控与返航状态机算法
 */

#include "Bow_Path_Task.h"
#include <math.h>

#define PI 3.14159265f

BowPathState_t bow_state = BOW_STATE_IDLE;
float bow_target_length = 0;
float bow_target_width = 0;
float bow_current_width = 0;
uint8_t bow_pass_count = 0; 

void Path_Generate_Bow(float length, float width)
{
    // 初始化清零
    bow_target_length = length;
    bow_target_width = width;
    bow_current_width = 0;
    bow_pass_count = 0; 

    // 确保每次发车前，原点绝对干净！
    Odometry_Init(); 
    g_car_status.yaw = 0.0f; // 强制车头朝向正前方 (+X轴)
    
    bow_state = BOW_STATE_FORWARD;
}

void Path_Follow_Routine(void)
{
    switch (bow_state)
    {
        case BOW_STATE_IDLE: 
            break;

        // ==========================================
        // 1. 直行扫“长” (严格看 X 坐标)
        // ==========================================
        case BOW_STATE_FORWARD:
        { // 加上大括号，限制内部变量作用域
            uint8_t reached_end = 0;
            
            if (bow_pass_count % 2 == 0) {
                // 偶数趟：往 +X 方向开
                if (g_car_status.pos_x >= bow_target_length) reached_end = 1;
            } else {
                // 奇数趟：往 -X 方向开，回到起点线
                if (g_car_status.pos_x <= 0.0f) reached_end = 1;
            }

            if (reached_end) {
                Motor_stop();
                vTaskDelay(pdMS_TO_TICKS(300));
                bow_state = BOW_STATE_TURN_1;
            } else {
                Motor_advance(40); 
            }
            break;
        }

        // ==========================================
        // 2. 第一次转向 (奇偶趟相反)
        // ==========================================
        case BOW_STATE_TURN_1:
            if (bow_pass_count % 2 == 0) Motor_Turn_Right_90(40); 
            else                         Motor_Turn_Left_90(40);

            bow_state = BOW_STATE_STEP_SIDE;
            break;

        // ==========================================
        // 3. 跨越车道 (严格看 Y 坐标累加量)
        // ==========================================
        case BOW_STATE_STEP_SIDE:
        { 
            // 因为第一趟是向右转，所以小车是在往 -Y 轴方向平移，我们取绝对值比较好算
            float current_abs_y = fabs(g_car_status.pos_y);
            float target_abs_y = bow_current_width + 20.0f; 

            if (current_abs_y >= target_abs_y)
            {
                Motor_stop();
                vTaskDelay(pdMS_TO_TICKS(300));
                
                bow_current_width += 20.0f;
                
                // 扫完全部宽度，进入返航阶段！
                if (bow_current_width >= bow_target_width) {
                    bow_state = BOW_STATE_RETURN_TURN; 
                } else {
                    bow_state = BOW_STATE_TURN_2;
                }
            } else {
                Motor_advance(30); 
            }
            break;
        }

        // ==========================================
        // 4. 第二次转向
        // ==========================================
        case BOW_STATE_TURN_2:
            if (bow_pass_count % 2 == 0) Motor_Turn_Right_90(40);
            else                         Motor_Turn_Left_90(40);

            bow_pass_count++; 
            bow_state = BOW_STATE_FORWARD; 
            break;

        // ==========================================
        // 5. 返航阶段一：原地打方向盘，车头精确瞄准 (0,0)
        // ==========================================
        case BOW_STATE_RETURN_TURN:
        { 
            // 关掉扫地电机，准备下班回家
            Motor_clean_stop();
            
            // 计算原点 (0,0) 相对于当前位置的向量角度
            float dy = 0.0f - g_car_status.pos_y;
            float dx = 0.0f - g_car_status.pos_x;
            float target_angle = atan2(dy, dx) * 180.0f / PI; 
            
            // 计算需要转多少度才能对准原点
            float angle_err = target_angle - g_car_status.yaw;
            while(angle_err > 180.0f)  angle_err -= 360.0f;
            while(angle_err < -180.0f) angle_err += 360.0f;

            // 调用底层执行精准原地转向
            if (angle_err > 0) Motor_Turn_Angle(40, angle_err, 2);  // 误差为正，向左转
            else               Motor_Turn_Angle(40, -angle_err, 1); // 误差为负，向右转
            
            bow_state = BOW_STATE_RETURN_DRIVE;
            break;
        }

        // ==========================================
        // 6. 返航阶段二：朝着原点直线狂奔！
        // ==========================================
        case BOW_STATE_RETURN_DRIVE:
        { 
            float dist_to_origin = sqrt(g_car_status.pos_x * g_car_status.pos_x + 
                                        g_car_status.pos_y * g_car_status.pos_y);
            
            // 进入原点 8cm 半径圈内，完美停车，准备车头回正！
            if (dist_to_origin < 8.0f) {
                Motor_stop();
                vTaskDelay(pdMS_TO_TICKS(300)); // 停稳，等陀螺仪数据平复
                bow_state = BOW_STATE_RETURN_ALIGN; // 切换到回正状态
            } 
            else 
            {
                Motor_advance(45); 
            }
            break;
        }

        // ==========================================
        // 7. 返航阶段三：车头绝对回正 (归零)
        // ==========================================
        case BOW_STATE_RETURN_ALIGN:
        {
            // 我们的目标是回到初始的 0 度朝向
            float angle_err = 0.0f - g_car_status.yaw;
            
            // 同样处理 180 度跳变，找出最近的转弯方向
            while(angle_err > 180.0f)  angle_err -= 360.0f;
            while(angle_err < -180.0f) angle_err += 360.0f;

            // 给 3 度的容错死区，防止因为极其微小的误差导致原地来回抽搐
            if (fabs(angle_err) > 3.0f) 
            {
                if (angle_err > 0) Motor_Turn_Angle(40, angle_err, 2);  // 误差为正，向左转补齐
                else               Motor_Turn_Angle(40, -angle_err, 1); // 误差为负，向右转补齐
            }
            
            // 转完收工，进入彻底结束状态！
            bow_state = BOW_STATE_FINISH;
            break;
        }

        // ==========================================
        // 8. 彻底停机
        // ==========================================
        case BOW_STATE_FINISH:
        {
            Motor_stop();
            bow_state = BOW_STATE_IDLE;
            break;
        }
    }
}
