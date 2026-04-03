/* Flash_HAL.c */
#include "Flash.h"      // 根据你的芯片改成 stm32fxxx_hal.h

/*=============================================================================
  注意事项（重要！）
  1. 必须在 CubeMX 中打开 Write Protection 关闭（或手动解锁）
  2. 必须把要操作的 FLASH 区域设置为“读而执行”或“只读”，不能在运行代码区写自己
  3. STM32F1 的 FLASH 编程是以 16bit（半字）为最小单位，地址必须 2 字节对齐
  4. 页大小为 1KB (F103 小容量) 或 2KB (大容量)，请根据具体型号选择
=============================================================================*/

#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE          0x400U    // 1KB = 1024B，大容量芯片是 2KB
#endif

/**
  * @brief  FLASH 读取一个 32 位字（直接读取，不需要解锁）
  * @param  Address: 要读取的地址（4 字节对齐）
  * @retval 读取到的 32 位数据
  */
uint32_t Flash_ReadWord(uint32_t Address)
{
    return *(__IO uint32_t *)Address;
}

/**
  * @brief  FLASH 读取一个 16 位半字
  * @param  Address: 要读取的地址（2 字节对齐）
  * @retval 读取到的 16 位数据
  */
uint16_t Flash_ReadHalfWord(uint32_t Address)
{
    return *(__IO uint16_t *)Address;
}

/**
  * @brief  FLASH 读取一个 8 位字节
  * @param  Address: 要读取的地址
  * @retval 读取到的 8 位数据
  */
uint8_t Flash_ReadByte(uint32_t Address)
{
    return *(__IO uint8_t *)Address;
}

/**
  * @brief  FLASH 全片擦除（慎用！会擦掉整个用户 Flash，包括当前程序）
  * @param  None
  * @retval None
  */
void Flash_EraseAllPages(void)
{
    FLASH_EraseInitTypeDef eraseinit = {0};
    uint32_t page_error = 0;

    eraseinit.TypeErase = FLASH_TYPEERASE_MASSERASE;   // 质量擦除 = 全擦
    eraseinit.Banks     = FLASH_BANK_1;                // F1 只有 Bank1

    HAL_FLASH_Unlock();

    if (HAL_FLASHEx_Erase(&eraseinit, &page_error) != HAL_OK)
    {
        /* 错误处理 */
        Error_Handler();
    }

    HAL_FLASH_Lock();
}

/**
  * @brief  FLASH 页擦除（F1 系列 1KB 或 2KB 一页）
  * @param  PageAddress: 要擦除的页起始地址（必须是页边界）
  * @retval None
  */
void Flash_ErasePage(uint32_t PageAddress)
{
    FLASH_EraseInitTypeDef eraseinit = {0};
    uint32_t page_error = 0;

    /* 地址必须页对齐 */
    if (PageAddress % FLASH_PAGE_SIZE != 0)
    {
        /* 地址不对齐，错误处理 */
        Error_Handler();
    }

    eraseinit.TypeErase   = FLASH_TYPEERASE_PAGES;
    eraseinit.Banks       = FLASH_BANK_1;
    eraseinit.PageAddress = PageAddress;
    eraseinit.NbPages     = 1;                         // 擦一页

    HAL_FLASH_Unlock();

    if (HAL_FLASHEx_Erase(&eraseinit, &page_error) != HAL_OK)
    {
        /* 擦除出错 */
        Error_Handler();
    }

    HAL_FLASH_Lock();
}

/**
  * @brief  FLASH 写入一个 32 位字（内部会拆成两次半字编程）
  * @param  Address: 目标地址（必须 2 字节对齐）
  * @param  Data:    要写入的 32 位数据
  * @retval None
  */
void Flash_ProgramWord(uint32_t Address, uint32_t Data)
{
    HAL_StatusTypeDef status;

    HAL_FLASH_Unlock();

    /* F1 只能按半字写，所以拆成两次 */
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address,       (uint16_t)Data);
    if (status == HAL_OK)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address + 2, (uint16_t)(Data >> 16));
    }

    if (status != HAL_OK)
    {
        /* 编程错误处理 */
        Error_Handler();
    }

    HAL_FLASH_Lock();
}

/**
  * @brief  FLASH 写入一个 16 位半字（推荐方式）
  * @param  Address: 目标地址（必须 2 字节对齐）
  * @param  Data:    要写入的 16 位数据
  * @retval None
  */
void Flash_ProgramHalfWord(uint32_t Address, uint16_t Data)
{
    HAL_FLASH_Unlock();

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, Data) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_FLASH_Lock();
}

/**
  * @brief  FLASH 写入任意长度数据（推荐在实际项目中使用）
  * @param  Address: 起始地址（2 字节对齐）
  * @param  pData:   数据指针
  * @param   Size:     要写入的字节数（建议偶数）
  * @retval None
  */
void Flash_Program(uint32_t Address, const uint8_t *pData, uint32_t Size)
{
    uint32_t i;
    
    if ((Address % 2) != 0 || (Size == 0)) return;

    HAL_FLASH_Unlock();

    for (i = 0; i < Size; i += 2)
    {
        uint16_t halfword = pData[i] | (pData[i+1] << 8);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address + i, halfword) != HAL_OK)
        {
            HAL_FLASH_Lock();
            Error_Handler();
            return;
        }
    }

    HAL_FLASH_Lock();
}

