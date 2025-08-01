/**
 ****************************************************************************************************
 * @file        usart.h
 * @author      
 * @version     V1.1
 * @date        2023-06-05
 * @brief       SRAM口初始化代码
 * @license     Copyright (c) 2020-2032, 常州潞城
 ****************************************************************************************************
 * 修改说明
 * V1.0 20250320
 * 第一次发布
 ****************************************************************************************************
 */

 #ifndef __SRAM_H
 #define __SRAM_H
 
#include "string.h"
#include "sys.h"


#define SRAM_SIZE      (1 << 19)   // 512KB SRAM   256k*16bit
#define HALF_SRAM_SIZE (1 << 18)   // 256KB
#define BUF_WORDS      512         // 512 x uint16_t = 1024 bytes
#define PATTERN_SIZE   2048        // 数据范围 0 到 0x7FF (2048 个值)
#define BUF_BYTES      (BUF_WORDS * 2) // 1024 bytes
#define SAMPLE_COUNT   400         // 采样点数
#define SAMPLE_INTERVAL 50         // 采样间隔
#define SIN_FLAG       0           // 正弦数据标志
#define COS_FLAG       1           // 余弦数据标志



/******************************************************************************************/
/* SRAM WR/RD/CS 引脚 定义 
 * SRAM_D0~D15 及 地址线,由于引脚太多,就不在这里定义了,直接在SRAM_init里面修改.所以在移植的时候,
 * 除了改这3个IO口, 还得改SRAM_init里面的 数据线 和 地址线 所在的IO口.
 */

#define SRAM_WR_GPIO_PORT               GPIOD
#define SRAM_WR_GPIO_PIN                GPIO_PIN_5
#define SRAM_WR_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)     /* 所在IO口时钟使能 */

#define SRAM_RD_GPIO_PORT               GPIOD
#define SRAM_RD_GPIO_PIN                GPIO_PIN_4
#define SRAM_RD_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)     /* 所在IO口时钟使能 */

/* SRAM_CS(需要根据SRAM_FSMC_NEX设置正确的IO口) 引脚 定义 */
#define SRAM_CS_GPIO_PORT                GPIOD
#define SRAM_CS_GPIO_PIN                 GPIO_PIN_7
#define SRAM_CS_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)    /* 所在IO口时钟使能 */

/* FSMC相关参数 定义 
 * 注意: 我们默认是通过FSMC块3来连接SRAM, 块1有4个片选: FSMC_NE1~4
 *
 * 修改SRAM_FSMC_NEX, 对应的SRAM_CS_GPIO相关设置也得改
 */
#define SRAM_FSMC_NEX           1         /* 使用FSMC_NE3接SRAM_CS,取值范围只能是: 1~4 */

#define SRAM_FSMC_BCRX          FSMC_Bank1->BTCR[(SRAM_FSMC_NEX - 1) * 2]       /* BCR寄存器,根据SRAM_FSMC_NEX自动计算 */
#define SRAM_FSMC_BTRX          FSMC_Bank1->BTCR[(SRAM_FSMC_NEX - 1) * 2 + 1]   /* BTR寄存器,根据SRAM_FSMC_NEX自动计算 */
#define SRAM_FSMC_BWTRX         FSMC_Bank1E->BWTR[(SRAM_FSMC_NEX - 1) * 2]      /* BWTR寄存器,根据SRAM_FSMC_NEX自动计算 */

/******************************************************************************************/

/* SRAM基地址, 根据 SRAM_FSMC_NEX 的设置来决定基址地址
 * 我们一般使用FSMC的块1(BANK1)来驱动SRAM, 块1地址范围总大小为256MB,均分成4块:
 * 存储块1(FSMC_NE1)地址范围: 0X6000 0000 ~ 0X63FF FFFF
 * 存储块2(FSMC_NE2)地址范围: 0X6400 0000 ~ 0X67FF FFFF
 * 存储块3(FSMC_NE3)地址范围: 0X6800 0000 ~ 0X6BFF FFFF
 * 存储块4(FSMC_NE4)地址范围: 0X6C00 0000 ~ 0X6FFF FFFF
 */
#define SRAM_BASE_ADDR         (0X60000000 + (0X4000000 * (SRAM_FSMC_NEX - 1)))
//SRAM位置
#define DS_DATA_ADDR_END       0x13880        //0-0x1387E


#define WS_DATA_ADDR_END       0x27100       //0-0x270FE

// 错误码定义
typedef enum {
    SRAM_READ_OK = 0,
    SRAM_ADDR_OUT_OF_BOUNDS = -1,
    SRAM_RESULT_FULL = -2
} SramReadStatus;


extern SRAM_HandleTypeDef g_sram_handler;    /* SRAM句柄 */


void sram_init(void);
void sram_write(uint8_t *pbuf, uint32_t addr, uint32_t datalen);
void sram_read(uint8_t *pbuf, uint32_t addr, uint32_t datalen);
void sram_read_u16(uint16_t *pbuf, uint32_t addr, uint32_t datalen);

uint8_t sram_test_read(uint32_t addr);
void sram_test_write(uint32_t addr, uint8_t data);
void sram_test_rw(uint32_t type);


SramReadStatus read_sram_sincos_data(uint8_t sin_cos_flag, uint8_t side_flag, uint16_t start_pos, uint16_t count);
void swap_uint16_array(uint16_t *data, size_t len);
#endif
