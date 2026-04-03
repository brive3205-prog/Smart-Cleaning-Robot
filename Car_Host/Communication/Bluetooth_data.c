/**
 * @file    Bluetooth_data.c
 * @brief   蓝牙串口自定义字符串通信协议解析与指令分发执行
 */

#include "Bluetooth_data.h"

/**
 * @brief 蓝牙应用层指令解析与分发
 * @param packet 传入的完整有效字符串，例如 "key,qianjin,down"
 */
void BT_Execute_Command(char *packet)
{
    // 1. 按照逗号分割出第一个词 (Tag标签)
    char *Tag = strtok(packet, ",");
    if (Tag == NULL) return; 

    // ==================================================
    // 类别 A：遥控按键指令 (如 "key,qianjin,down")
    // ==================================================
    if (strcmp(Tag, "key") == 0)
    {
        char *Name   = strtok(NULL, ",");
        char *Action = strtok(NULL, ",");
        
        if (Name != NULL && Action != NULL)
        {
            uint8_t state = (strcmp(Action, "down") == 0) ? 1 : 0;
            uint8_t key_code = 0;
            
            // 提取键值
            if(strcmp(Name, "qianjin") == 0)      key_code = 1;
            else if(strcmp(Name, "houtui") == 0)  key_code = 2;
            else if(strcmp(Name, "zuozhuan") == 0) key_code = 3;
            else if(strcmp(Name, "youzhuan") == 0) key_code = 4;

            // 若是录放模式，把动作转交给专门的录制器处理；否则走普通的手动驱动
            if (g_car_status.current_mode == MODE_RECORD_PLAY)
            {
                if (key_code != 0) {
                    Record_Key_Action(key_code, state);
                }
            }
            else
            {
                switch(state)
                {
                    case 1:
                        if(key_code == 1) g_car_status.running_status = FORWARD;
                        else if(key_code == 2) g_car_status.running_status = BACKWARD;
                        else if(key_code == 3) g_car_status.running_status = LEFT;
                        else if(key_code == 4) g_car_status.running_status = RIGHT;
                        break;
                    case 0:
                        g_car_status.running_status = STOP;
                        break;
                }
            }
        }
    }
    // ==================================================
    // 类别 B：模式切换指令 (如 "mode,bizhang")
    // ==================================================
    else if (strcmp(Tag, "mode") == 0)
    {
        char *ModeName = strtok(NULL, ",");
        if (ModeName != NULL)
        {
            if (strcmp(ModeName, "shoudong") == 0) 
            {
                Mode_SetCurrent(MODE_MANUAL);
            } 
            else if (strcmp(ModeName, "bizhang") == 0) 
            {
                Mode_SetCurrent(MODE_AVOIDANCE);
            } 
            else if (strcmp(ModeName, "luzhi") == 0) 
            {
                Mode_SetCurrent(MODE_RECORD_PLAY);
            } 
            else if (strcmp(ModeName, "guihua") == 0) 
            {
                Mode_SetCurrent(MODE_BOW_PATH);
            }
        }
    }
    // ==================================================
    // 类别 C：录制回放开关指令 (如 "cmd,rec_start")
    // ==================================================
    else if (strcmp(Tag, "cmd") == 0)
    {
        char *CmdName = strtok(NULL, ",");
        if (CmdName != NULL)
        {
            if(strcmp(CmdName, "rec_start") == 0) 
            {
                CMD_Start_Record();
            } 
            else if(strcmp(CmdName, "rec_stop") == 0) 
            {
                CMD_Stop_Record(1); // 默认存在路线1
            } 
            else if(strcmp(CmdName, "play_start") == 0) 
            {
                CMD_Start_Play(1);  // 默认播放路线1
            }
            else if(strcmp(CmdName, "guihua_start") == 0)
            {
                Path_Generate_Bow(80, 80);
                Motor_clean_start();
            }
            else if(strcmp(CmdName, "clear_start") == 0)
            {
                g_car_status.current_clean = 1;
            }
            else if(strcmp(CmdName, "clear_stop") == 0)
            {
                g_car_status.current_clean = 0;
            }
        }
    }
    // ==================================================
    // 类别 D：弓字形路径长宽设定 (如 "bow,150,200")
    // ==================================================
    else if (strcmp(Tag, "bow") == 0)
    {
        char *len_str = strtok(NULL, ",");
        char *wid_str = strtok(NULL, ",");
        if (len_str && wid_str) 
        {
            // 安全地更新弓字形路径参数
            if(status_mutex != NULL && xSemaphoreTake(status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_car_status.current_length = atof(len_str);
                g_car_status.current_width = atof(wid_str);
                xSemaphoreGive(status_mutex);
            }
        }
    }
    // ==================================================
    // 类别 E：全局速度设定 (如 "speed,80")
    // ==================================================
    else if (strcmp(Tag, "speed") == 0)
    {
        char *spd_str = strtok(NULL, ",");
        if (spd_str) 
        {
            // 安全地更新全局速度
            if(status_mutex != NULL && xSemaphoreTake(status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) 
            {
                g_car_status.current_speed = atoi(spd_str);
                xSemaphoreGive(status_mutex);
            }
        }
    }
}




/**
 * @brief 蓝牙底层数据帧防粘包组装状态机
 * @param bt_char 从 FreeRTOS 队列中取出的单个字符
 */
void Parse_Bluetooth_Protocol(uint8_t bt_char)
{
    static char bt_buffer[64];
    static uint8_t bt_state = 0;
    static uint8_t bt_index = 0;

    // 无论何时，只要看到 '[' (帧头)，无条件重新开始录入
    if (bt_char == '[') 
    {
        bt_state = 1;
        bt_index = 0;
        return; // 直接返回，不要把 '[' 存进去了
    }

    if (bt_state == 1)
    {
        if (bt_char == ']') 
        {
            bt_buffer[bt_index] = '\0'; 
            BT_Execute_Command(bt_buffer); 
            bt_state = 0; 
        }
        else if (bt_index < sizeof(bt_buffer) - 1)
        {
            bt_buffer[bt_index++] = bt_char; 
        }
    }
}


/**
 * @brief 蓝牙模块硬件自检
 * @return 0: 成功, 1: 失败
 * @note 真实的自检应该通过串口发送 "AT" 指令并等待 "OK" 回复。
 *       这需要一个带超时的串口收发框架，这里暂时简化处理。
 */
uint8_t Bluetooth_Check(void)
{
    // 伪代码示例：
    // UART_Transmit("AT\r\n");
    // response = UART_Receive_With_Timeout(1000); // 等待1秒
    // if (strstr(response, "OK")) {
    //     return 0; // 成功
    // }
    return 0; // 暂时假设总是成功
}
