/* BlueTooth.c  ——  完整 HAL 库版本（STM32F4/F1/L4/G4 等全系列通用） */
#include "BlueTooth.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* 外部声明（CubeMX 自动生成的） */
extern UART_HandleTypeDef huart1;       // 注意：你的串口可能是 huart2/huart3，请根据实际改

/* 接收相关全局变量 */
char BlueTooth_RxPacket[100];           // 接收数据包缓冲区，协议格式 "[MSG]"
uint8_t BlueTooth_RxFlag = 0;           // 新数据包到达标志

/* 私有变量（用于中断状态机） */
//static uint8_t  BT_RxState   = 0;       // 0=等待'[', 1=正在接收数据
//static uint16_t BT_RxIndex   = 0;       // 当前写入位置
//static uint8_t  BT_RxTemp;      // 临时接收一个字节（必须是静态或全局）

/*=====================================================================
  1. 发送系列函数（阻塞式，简单可靠）
=====================================================================*/
void BlueTooth_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(&huart1, &Byte, 1, 1000);
}

void BlueTooth_SendArray(uint8_t *Array, uint16_t Length)
{
    HAL_UART_Transmit(&huart1, Array, Length, 1000);
}

void BlueTooth_SendString(const char *String)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)String, (uint16_t)strlen(String), 1000);
}

static uint32_t BlueTooth_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) Result *= X;
    return Result;
}

void BlueTooth_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        uint8_t digit = (Number / BlueTooth_Pow(10, Length - i - 1)) % 10 + '0';
        BlueTooth_SendByte(digit);
    }
}

/* printf 重定向 */
int fputc(int ch, FILE *f)
{
    BlueTooth_SendByte((uint8_t)ch);
    return ch;
}

void BlueTooth_Printf(const char *format, ...)
{
    char buf[128];
    va_list arg;
    va_start(arg, format);
    vsnprintf(buf, sizeof(buf), format, arg);
    va_end(arg);
    BlueTooth_SendString(buf);
}

/*=====================================================================
  2. 接收状态机（HAL 推荐方式：中断 + 回调）
=====================================================================*/

/* 必须在 main.c 的 MX_USART1_UART_Init() 之后调用一次开启接收 */
//void BlueTooth_Init(void)
//{
//    // 每次只接收 1 个字节，收到后自动进入回调函数
//    HAL_UART_Receive_IT(&huart1, &BT_RxTemp, 1);
//}

/* HAL 库串口接收完成回调函数（重要！） */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART1)          // 确认是 USART1（或你的 huart1.Instance）
//    {
//        /* 下面的状态机逻辑和你原来的完全一样 */
//        if (BT_RxState == 0)                 // 正在等包头 '['
//        {
//            if (BT_RxTemp == '[' && BlueTooth_RxFlag == 0)
//            {
//                BT_RxState = 1;
//                BT_RxIndex = 0;
//            }
//        }
//        else if (BT_RxState == 1)            // 正在接收数据
//        {
//            if (BT_RxTemp == ']')            // 收到包尾
//            {
//                BlueTooth_RxPacket[BT_RxIndex] = '\0';  // 字符串结束符
//                BlueTooth_RxFlag = 1;                    // 通知主循环有新包
//                BT_RxState = 0;
//            }
//            else
//            {
//                if (BT_RxIndex < sizeof(BlueTooth_RxPacket) - 1)
//                {
//                    BlueTooth_RxPacket[BT_RxIndex++] = BT_RxTemp;
//                }
//            }
//        }

//        /* 必须再次开启下一次中断接收，否则就收不到了！ */
//        HAL_UART_Receive_IT(&huart1, &BT_RxTemp, 1);
//    }
//	
//}

