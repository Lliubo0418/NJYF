/**
 ****************************************************************************************************
 * @file        stmflash.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-28
 * @brief       STM32内部FLASH读写 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 探索者 F407开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211028
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __STMFLASH_H
#define __STMFLASH_H

#include "sys.h"


/* FLASH起始地址 */
#define STM32_FLASH_SIZE        0x100000        /* STM32 FLASH 总大小 */
#define STM32_FLASH_BASE        0x08000000      /* STM32 FLASH 起始地址 */
#define FLASH_TIMEOUT_VALUE         50000           /* FLASH等待超时时间 50s */

/* Define read types (should be in stm32f4xx_hal_flash.h) */
#define FLASH_TYPE_READ_BYTE      ((uint32_t)0x00U)
#define FLASH_TYPE_READ_HALFWORD  ((uint32_t)0x01U)
#define FLASH_TYPE_READ_WORD      ((uint32_t)0x02U)

/* FLASH 扇区的起始地址 */
#define ADDR_FLASH_SECTOR_0     ((uint32_t )0x08000000)     /* 扇区0起始地址, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t )0x08004000)     /* 扇区1起始地址, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t )0x08008000)     /* 扇区2起始地址, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t )0x0800C000)     /* 扇区3起始地址, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t )0x08010000)     /* 扇区4起始地址, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t )0x08020000)     /* 扇区5起始地址, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t )0x08040000)     /* 扇区6起始地址, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t )0x08060000)     /* 扇区7起始地址, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t )0x08080000)     /* 扇区8起始地址, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t )0x080A0000)     /* 扇区9起始地址, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t )0x080C0000)     /* 扇区10起始地址,128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t )0x080E0000)     /* 扇区11起始地址,128 Kbytes */

uint32_t stmflash_read_word(uint32_t faddr);                             /* 读出字 */
void stmflash_write(uint32_t waddr, uint32_t *pbuf, uint32_t length);    /* 从指定地址开始写入指定长度的数据 */
void stmflash_read(uint32_t raddr, uint32_t *pbuf, uint32_t length);     /* 从指定地址开始读出指定长度的数据 */

void test_write(uint32_t waddr, uint32_t wdata);                         /* 测试写入 */

HAL_StatusTypeDef FLASH_Read_Byte(uint32_t Address, uint8_t *Data);
HAL_StatusTypeDef FLASH_Read_HalfWord(uint32_t Address, uint16_t *Data);
HAL_StatusTypeDef FLASH_Read_Word(uint32_t Address, uint32_t *Data);

#endif


















