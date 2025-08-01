
#include "flash.h"

#define FLASH_TIMEOUT_VALUE       50000U /* 50 s */

/**
  * @brief  Programs a half word at a specified address.
  * @note   This function can be used for all STM32F10x devices.
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, HAL_OK or FLASH_TIMEOUT.
  */
 uint32_t FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data)
 {
    uint32_t status = HAL_OK;
   /* Check the parameters */
   assert_param(IS_FLASH_ADDRESS(Address));
  
   /* Wait for last operation to be completed */
   status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
  
   if(status == HAL_OK)
   {
     /* if the previous operation is completed, proceed to program the new data */
        /* If the previous operation is completed, proceed to program the new data */
        CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);
        FLASH->CR |= FLASH_PSIZE_HALF_WORD;
        FLASH->CR |= FLASH_CR_PG;

        *(__IO uint16_t *)Address = Data;

        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
    }
  
   /* Return the Program Status */
   return status;
 }

// 从指定地址开始写入多个数据
void FLASH_WriteMoreData(uint32_t startAddress, uint16_t *writeData, uint16_t countToWrite)
{
    uint32_t offsetAddress = startAddress - FLASH_BASE;                      // 计算去掉0X08000000后的实际偏移地址
    uint32_t sectorPosition = offsetAddress / SECTOR_SIZE;                   // 计算扇区地址
    //uint32_t sectorStartAddress = sectorPosition * SECTOR_SIZE + FLASH_BASE; // 对应扇区的首地址
    uint16_t dataIndex;

    if (startAddress < FLASH_BASE || ((startAddress + countToWrite * 2) >= FLASH_END))
    {
        return; // 非法地址
    }
    HAL_FLASH_Unlock(); // 解锁写保护

    FLASH_Erase_Sector(sectorPosition, FLASH_VOLTAGE_RANGE_2);       // 擦除这个扇区

    for (dataIndex = 0; dataIndex < countToWrite; dataIndex++)
    {
        FLASH_ProgramHalfWord(startAddress + dataIndex * 2, writeData[dataIndex]);
    }

    HAL_FLASH_Lock(); // 上锁写保护
}

// 读取指定地址的半字(16位数据)
uint16_t FLASH_ReadHalfWord(uint32_t address)
{
    return *(__IO uint16_t *)address;
}

// 从指定地址开始读取多个数据
void FLASH_ReadMoreData(uint32_t startAddress, uint16_t *readData, uint16_t countToRead)
{
    uint16_t dataIndex;
    for (dataIndex = 0; dataIndex < countToRead; dataIndex++)
    {
        readData[dataIndex] = FLASH_ReadHalfWord(startAddress + dataIndex * 2);
    }
}

void Flash_test(void)
{
    uint16_t data[20];

    FLASH_ReadMoreData(CONFIGURATION_BASE_ADDR, data, 20);

    for (int i = 0; i < 20; i++)
    {
        data[i] = i + 0x100;
    }

    FLASH_WriteMoreData(CONFIGURATION_BASE_ADDR, data, 20);
    
    //memset(data, 0, 20 * 2);
    for (int i = 0; i < 20; i++)
    {
        data[i] = 0x0000;
    }

    FLASH_ReadMoreData(CONFIGURATION_BASE_ADDR, data, 20);
}
