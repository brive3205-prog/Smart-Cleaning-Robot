/**
 * @file    Record_Play_Task.c
 * @brief   轨迹录制与回放模式算法，及内部Flash存储调度
 */

#include "Record_Play_Task.h"
#include <math.h>   // 引入 fabs
#include <string.h> // 引入 memcpy

// ====================== 配置区 ======================
#define Flash_BASE1          0x0800F800    // 存储地址第62页 
#define Flash_BASE2          0x0800F400    // 存储地址第61页
#define MAX_EVENTS           100           // 限制为100个事件(1000字节)，确保连同4字节头部能完美塞入1KB的Flash页中
#define MAGIC1               0xA5A5        // 路线1魔数
#define MAGIC2               0xA5A5        // 路线2魔数

// ====================== 全局状态变量 ======================
// 注意：这三个变量不要加 static，它们在 .h 中被 extern，供整个工程查看状态
RecordPlayState_t rp_state = RP_STATE_IDLE; 
uint32_t record_start_tick = 0;
uint32_t play_start_tick = 0;



// ====================== 内部私有变量 ======================
static Event events[MAX_EVENTS];           // 事件存盘数组
static uint16_t event_cnt = 0;             // 当前录制的事件总数
static uint16_t play_idx = 0;              // 播放时的“进度条（书签）”

// ==========================================================
// 1. Flash 底层存储读取函数
// ==========================================================
static void save_to_flash(uint8_t FLAG)
{
    uint32_t base_addr = (FLAG == 1) ? Flash_BASE1 : Flash_BASE2;
    uint16_t magic = (FLAG == 1) ? MAGIC1 : MAGIC2;

    Flash_ErasePage(base_addr); // 擦除对应页

    // 1. 写入魔数和事件总数 (占前4个字节)
    Flash_ProgramHalfWord(base_addr, magic);
    Flash_ProgramHalfWord(base_addr + 2, event_cnt);

    // 2. 批量写入整个 events 数组！
    if (event_cnt > 0) {
        Flash_Program(base_addr + 4, (const uint8_t *)events, event_cnt * sizeof(Event));
    }
}

static void load_to_flash(uint8_t FLAG)
{
    uint32_t base_addr = (FLAG == 1) ? Flash_BASE1 : Flash_BASE2;
    uint16_t magic = (FLAG == 1) ? MAGIC1 : MAGIC2;

    // 1. 校验魔数
    if(Flash_ReadHalfWord(base_addr) != magic) {
        event_cnt = 0;
        return;
    }
    
    // 2. 读取事件数量
    event_cnt = Flash_ReadHalfWord(base_addr + 2);
    if(event_cnt > MAX_EVENTS) event_cnt = MAX_EVENTS;

    // 3. 使用 memcpy 批量安全拷贝，避免出现跨字节边界的非对齐访问错误
    memcpy(events, (const void*)(base_addr + 4), event_cnt * sizeof(Event));
}

// ==========================================================
// 2. 暴露给外部（蓝牙协议解析）的控制接口
// ==========================================================

// 蓝牙命令：开始录制
void CMD_Start_Record(void) 
{
    // 判断当前模式是否为录制播放模式
    if(g_car_status.current_mode == MODE_RECORD_PLAY && rp_state == RP_STATE_IDLE) 
    {
        event_cnt = 0;
        record_start_tick = xTaskGetTickCount();
        rp_state = RP_STATE_RECORDING;
    }
}

// 蓝牙命令：停止录制并存入 Flash
void CMD_Stop_Record(uint8_t flag) 
{
    // 判断是否在录制状态
    if(rp_state == RP_STATE_RECORDING) 
    {
        rp_state = RP_STATE_IDLE;
        Motor_stop();        // 确保车停下
        save_to_flash(flag); // 存入 Flash
    }
}

// 蓝牙命令：开始播放
void CMD_Start_Play(uint8_t flag) 
{
    // 判断当前模式是否为录制播放模式且处于空闲状态
    if(g_car_status.current_mode == MODE_RECORD_PLAY && rp_state == RP_STATE_IDLE) {
        load_to_flash(flag); // 从 Flash 读取动作序列
        if(event_cnt > 0) 
        {
            play_idx = 0;
            play_start_tick = xTaskGetTickCount();
            rp_state = RP_STATE_PLAYING;
        }
    }
}

// ==========================================================
// 3. 录制逻辑：执行并录制按键动作 (供蓝牙解析任务调用)
// ==========================================================
void Record_Key_Action(uint8_t key, uint8_t state)
{
    // 如果在录制状态，抓取状态并存入数组
    if(rp_state == RP_STATE_RECORDING) 
    {
        Event e;
        e.time_ms = (xTaskGetTickCount() - record_start_tick) * portTICK_PERIOD_MS;
        e.key = key;
        e.state = state;
        e.speed = g_car_status.current_speed; // 记录当前设定速度
        e.angle = 0; 
        
        // 核心：偏航角处理
        if((key == 3 || key == 4) && state == 1) 
        {
            // 刚按下转向键：把 MPU6050 偏航角清零，作为参考零点
            taskENTER_CRITICAL();
            g_car_status.yaw = 0.0f; 
            taskEXIT_CRITICAL();
        }
        else if((key == 3 || key == 4) && state == 0) 
        {
            // 松开转向键：读取实际转过的绝对角度存下来！
            taskENTER_CRITICAL();
            e.angle = (int16_t)fabs(g_car_status.yaw); // 取绝对值，适配闭环转向函数
            taskEXIT_CRITICAL();
        }

        // 存入数组
        if(event_cnt < MAX_EVENTS) 
        {
            events[event_cnt++] = e;
        }
    }

    // 如果在播放状态下，强行屏蔽人为的遥控操作，防止蓝牙指令与自动回放打架
    if(rp_state == RP_STATE_PLAYING)
    {
        return;
    }

    // 实际控制底层电机的逻辑（不管录不录制，按下按键车总得动）
    if(state == 1) 
    { // 按下
        if (key == 1) { Motor_advance(g_car_status.current_speed); }
        else if (key == 2) { Motor_back(g_car_status.current_speed); }
        else if (key == 3) { Motor_turn_left(g_car_status.current_speed); }
        else if (key == 4) { Motor_turn_right(g_car_status.current_speed); }
    } 
    else 
    { // 松开
        Motor_stop();
    }
}

// ==========================================================
// 4. 播放逻辑与核心状态机 (在 mode_control.c 的死循环中被调用)
// ==========================================================
void RecordPlay_Task_Loop(void)
{
    switch(rp_state)
    {
        case RP_STATE_IDLE:
            // 空闲待命，不占CPU
            break;

        case RP_STATE_RECORDING:
            // 录制动作由 Record_Key_Action() 触发，这里只防数组越界
            if(event_cnt >= MAX_EVENTS) 
            {
                CMD_Stop_Record(1); // 存满自动停止（默认存线路1）
            }
            break;

        case RP_STATE_PLAYING:
            // 1. 判断是否所有动作都已经播放完毕
            if(play_idx >= event_cnt)
            {
                Motor_stop();
                rp_state = RP_STATE_IDLE;
                break;
            }

            // 2. 计算当前播放进度时间 (ms)
            uint32_t now_ms = (xTaskGetTickCount() - play_start_tick) * portTICK_PERIOD_MS;
            Event *e = &events[play_idx];

            // 3. 【核心判断】时间到了吗？到了就执行动作！没到就迅速退出让出CPU！
            if(now_ms >= e->time_ms)
            {
                if(e->state == 1) 
                { 
                    // 动作：按下 (带上当时录制的速度)
                    if (e->key == 1) { Motor_advance(e->speed); }
                    else if (e->key == 2) { Motor_back(e->speed); }
                    else if (e->key == 3 || e->key == 4) 
                    { 
                        // 闭环转向接管：往后透视寻找对应的“松开”事件，提前拿到刚才录制时真实转过的角度
                        float target_angle = 0.0f;
                        for (int i = play_idx + 1; i < event_cnt; i++) {
                            if (events[i].key == e->key && events[i].state == 0) {
                                target_angle = (float)events[i].angle;
                                break;
                            }
                        }
                        
                        if (target_angle > 5.0f) {
                            // 如果找到了有效角度，调用底层的闭环转向函数进行高精度转弯！
                            uint8_t dir = (e->key == 3) ? 2 : 1; // 3左(dir=2), 4右(dir=1)
                            Motor_Turn_Angle(e->speed, target_angle, dir);
                        } else {
                            // 降级方案：按开环转
                            if (e->key == 3) Motor_turn_left(e->speed);
                            else             Motor_turn_right(e->speed);
                        }
                    }
                } 
                else
                { 
                    // 动作：松开 (刹车)
                    Motor_stop();
                }
                
                play_idx++; // 准备读下一个动作
            }
            break;
    }
}
