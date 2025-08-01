#ifndef __FLASH_H__
#define __FLASH_H__

#include "sys.h"
#include "stm32f4xx_hal.h"

// 对于 STM32F40x 和 STM32F41x，容量高达 1 MB；对于 STM32F42x 和 STM32F43x， 容量高达 2 MB
// #define FLASH_BASE            0x08000000UL /*!< FLASH(up to 2 MB) base address in the alias region                         */

#define SECTOR_SIZE             0x4000

#define CONFIGURATION_BASE_ADDR 0x0800C000
#define CONFIGURATION_SIZE      FLASH_PAGE_SIZE

void FLASH_WriteMoreData(uint32_t startAddress, uint16_t *writeData, uint16_t countToWrite);
void FLASH_ReadMoreData(uint32_t startAddress, uint16_t *readData, uint16_t countToRead);

void Flash_test(void);

#endif
