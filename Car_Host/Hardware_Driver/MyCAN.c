#include "MyCAN.h"


/**
 * @brief  CAN 初始化（过滤器配置与启动）
 * @note   在使用此函数前，确保 main.c 中已经调用了 MX_CAN_Init()
 */
void MyCAN_Init(void)
{
    // 在 HAL 库与 CubeMX 流程中，引脚配置、时钟使能、波特率和工作模式(Loopback)
    // 都已经由 CubeMX 自动生成的 MX_CAN_Init() 和 HAL_CAN_MspInit() 完成了。
    // 我们在这里只需要配置“过滤器”并“启动”CAN外设即可。
    
    CAN_FilterTypeDef CAN_FilterInitStructure;
    
    CAN_FilterInitStructure.FilterBank = 0;                               // 对应标准库的 CAN_FilterNumber = 0
    CAN_FilterInitStructure.FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.FilterIdLow = 0x0000;
    CAN_FilterInitStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.FilterScale = CAN_FILTERSCALE_32BIT;          // 对应 CAN_FilterScale_32bit
    CAN_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDMASK;           // 对应 CAN_FilterMode_IdMask
    CAN_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO0;          // 对应 CAN_Filter_FIFO0
    CAN_FilterInitStructure.FilterActivation = ENABLE;
    CAN_FilterInitStructure.SlaveStartFilterBank = 14;                    // 单CAN芯片填14即可
    
    // 应用过滤器配置
    if (HAL_CAN_ConfigFilter(&hcan, &CAN_FilterInitStructure) != HAL_OK)
    {
        Error_Handler();
    }
    
    // 启动 CAN
    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  CAN 发送数据帧 (阻塞式等待发送完成)
 */
void MyCAN_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    
    TxHeader.StdId = ID;
    TxHeader.ExtId = ID;
    TxHeader.IDE = CAN_ID_STD;         // 标准帧
    TxHeader.RTR = CAN_RTR_DATA;       // 数据帧
    TxHeader.DLC = Length;             // 数据长度
    TxHeader.TransmitGlobalTime = DISABLE;
    
    // 1. 等待至少有一个发送邮箱空闲 (带有超时机制)
    uint32_t Timeout = 0;
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
    {
        Timeout++;
        if (Timeout > 100000) return; // 超时退出
    }
    
    // 2. 将数据请求加入发送邮箱
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, Data, &TxMailbox) != HAL_OK)
    {
        return; // 请求失败
    }
    
    // 3. 阻塞等待该邮箱的数据发送完成 (完美复刻你原版标准库的 while 等待逻辑)
    Timeout = 0;
    while (HAL_CAN_IsTxMessagePending(&hcan, TxMailbox))
    {
        Timeout++;
        if (Timeout > 100000) break; // 超时退出
    }
}

/**
 * @brief  检测 CAN FIFO0 是否收到数据
 */
uint8_t MyCAN_ReceiveFlag(void)
{
    // HAL_CAN_GetRxFifoFillLevel 返回 FIFO 里挂起的消息数量
    if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0)
    {
        return 1;
    }
    return 0;
}

/**
 * @brief  读取 CAN FIFO0 接收到的数据
 */
void MyCAN_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data)
{
    CAN_RxHeaderTypeDef RxHeader;
    
    // 获取数据，HAL库会自动将提取的数据存入 Data 数组中
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, Data) == HAL_OK)
    {
        // 提取 ID
        if (RxHeader.IDE == CAN_ID_STD)
        {
            *ID = RxHeader.StdId;
        }
        else
        {
            *ID = RxHeader.ExtId;
        }
        
        // 提取数据长度
        if (RxHeader.RTR == CAN_RTR_DATA)
        {
            *Length = RxHeader.DLC;
        }
        else
        {
            *Length = 0;
        }
    }
}
