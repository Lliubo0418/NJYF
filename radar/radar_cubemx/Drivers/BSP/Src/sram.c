/**
 ****************************************************************************************************
 * @file        sram.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-11-04
 * @brief       外部SRAM 驱动代码
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
 * V1.0 20211103
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "sram.h"
#include "usart.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
#include "data_process.h"
#include "com_process.h"

#define SRAM_SIZE      (1 << 20)   // 1M
#define HALF_SRAM_SIZE      (1 << 19)   // 512KB
#define BUF_WORDS      512         // 512 x uint16_t = 1024 字节
#define PATTERN_SIZE 2048   /* 0 到 0x7FF 的数据范围（2048 个值） */
#define BUF_BYTES      (BUF_WORDS * 2)
#define SAMPLE_COUNT 200
#define SAMPLE_INTERVAL  50
#define sin 0
#define cos 1


uint8_t write_data[BUF_BYTES];
uint8_t read_data[BUF_BYTES];


uint16_t leftover[SAMPLE_INTERVAL];   // 最多存放不满一组的残留
int leftover_len = 0;
extern  uint16_t sin_data[200], cos_data[200];
extern uint8_t ws_ds_flag;                        /* 0: DS, 1: WS */

SRAM_HandleTypeDef g_sram_handler; /* SRAM句柄 */

/**
 * @brief       初始化 外部SRAM
 * @param       无
 * @retval      无
 */
void sram_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    FMC_NORSRAM_TimingTypeDef fsmc_readwritetim;

    SRAM_CS_GPIO_CLK_ENABLE();    /* SRAM_CS脚时钟使能 */
    SRAM_WR_GPIO_CLK_ENABLE();    /* SRAM_WR脚时钟使能 */
    SRAM_RD_GPIO_CLK_ENABLE();    /* SRAM_RD脚时钟使能 */
    __HAL_RCC_FMC_CLK_ENABLE();  /* 使能FMC时钟 */
    __HAL_RCC_GPIOD_CLK_ENABLE(); /* 使能GPIOD时钟 */
    __HAL_RCC_GPIOE_CLK_ENABLE(); /* 使能GPIOE时钟 */
    __HAL_RCC_GPIOF_CLK_ENABLE(); /* 使能GPIOF时钟 */
    __HAL_RCC_GPIOG_CLK_ENABLE(); /* 使能GPIOG时钟 */

    gpio_init_struct.Pin = SRAM_CS_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(SRAM_CS_GPIO_PORT, &gpio_init_struct); /* SRAM_CS引脚模式设置 */

    gpio_init_struct.Pin = SRAM_WR_GPIO_PIN;
    HAL_GPIO_Init(SRAM_WR_GPIO_PORT, &gpio_init_struct); /* SRAM_WR引脚模式设置 */

    gpio_init_struct.Pin = SRAM_RD_GPIO_PIN;
    HAL_GPIO_Init(SRAM_RD_GPIO_PORT, &gpio_init_struct); /* SRAM_CS引脚模式设置 */

    /* PD0,1,4,5,8~15 */
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 | 
                       GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                       GPIO_PIN_14 | GPIO_PIN_15;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;       /* 推挽复用 */
    gpio_init_struct.Pull = GPIO_PULLUP;           /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH; /* 高速 */
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);

    /* PE0,1,7~15 */
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
                       GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
                       GPIO_PIN_15;
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);

    /* PF0~5,12~15 */
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
                       GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOF, &gpio_init_struct);

    /* PG0~5,10 */
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOG, &gpio_init_struct);

    g_sram_handler.Instance = FMC_NORSRAM_DEVICE;
    g_sram_handler.Extended = FMC_NORSRAM_EXTENDED_DEVICE;

    g_sram_handler.Init.NSBank = (SRAM_FSMC_NEX == 1) ? FMC_NORSRAM_BANK1 : \
                                 (SRAM_FSMC_NEX == 2) ? FMC_NORSRAM_BANK2 : \
                                 (SRAM_FSMC_NEX == 3) ? FMC_NORSRAM_BANK3 : 
                                                        FMC_NORSRAM_BANK4; /* 根据配置选择FSMC_NE1~4 */
    g_sram_handler.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;     /* 地址/数据线不复用 */
    g_sram_handler.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;                 /* SRAM */
    g_sram_handler.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;    /* 16位数据宽度 */
    g_sram_handler.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;   /* 是否使能突发访问,仅对同步突发存储器有效,此处未用到 */
    g_sram_handler.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW; /* 等待信号的极性,仅在突发模式访问下有用 */
    g_sram_handler.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;      /* 存储器是在等待周期之前的一个时钟周期还是等待周期期间使能NWAIT */
    g_sram_handler.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;       /* 存储器写使能 */
    g_sram_handler.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;              /* 等待使能位,此处未用到 */
    g_sram_handler.Init.ExtendedMode = FMC_EXTENDED_MODE_ENABLE;          /* 读写使用相同的时序 */
    g_sram_handler.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;  /* 是否使能同步传输模式下的等待信号,此处未用到 */
    g_sram_handler.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;              /* 禁止突发写 */
    /* FMC读时序控制寄存器 */
    fsmc_readwritetim.AddressSetupTime = 0x02;                              /* 地址建立时间（ADDSET）为2个HCLK 1/168M=6ns*2=12ns */
    fsmc_readwritetim.AddressHoldTime = 0x00;                               /* 地址保持时间（ADDHLD）模式A未用到 */
    fsmc_readwritetim.DataSetupTime = 0x08;                                 /* 数据保存时间为8个HCLK =6*8= 48ns */
    fsmc_readwritetim.BusTurnAroundDuration = 0x00;
    fsmc_readwritetim.AccessMode = FMC_ACCESS_MODE_A;                      /* 模式A */
    HAL_SRAM_Init(&g_sram_handler, &fsmc_readwritetim, &fsmc_readwritetim);
}

/**
 * @brief       往SRAM指定地址写入指定长度数据
 * @param       pbuf    : 数据存储区
 * @param       addr    : 开始写入的地址(最大32bit)
 * @param       datalen : 要写入的字节数(最大32bit)
 * @retval      无
 */
void sram_write(uint8_t *pbuf, uint32_t addr, uint32_t datalen)
{
    for (; datalen != 0; datalen--)
    {
        *(volatile uint8_t *)(SRAM_BASE_ADDR + addr) = *pbuf;
        addr++;
        pbuf++;
    }
}
void sram_write_u16(uint16_t *pbuf, uint32_t addr, uint32_t datalen)
{
    for (; datalen != 0; datalen--)
    {
        *(volatile uint16_t *)(SRAM_BASE_ADDR + addr) = *pbuf;
        addr+=2;
        pbuf++;
    }
}
/**
 * @brief       从SRAM指定地址读取指定长度数据
 * @param       pbuf    : 数据存储区
 * @param       addr    : 开始读取的地址(最大32bit)
 * @param       datalen : 要读取的字节数(最大32bit)
 * @retval      无
 */
void sram_read(uint8_t *pbuf, uint32_t addr, uint32_t datalen)
{
    for (; datalen != 0; datalen--)
    {
        *pbuf++ = *(volatile uint8_t *)(SRAM_BASE_ADDR + addr);
        addr++;
    }
}
void sram_read_u16(uint16_t *pbuf, uint32_t addr, uint32_t datalen)
{
    for (; datalen != 0; datalen--)
    {
        *pbuf++ = *(volatile uint16_t *)(SRAM_BASE_ADDR + addr);
        addr += 2;
    }
}

/*******************测试函数**********************************/

/**
 * @brief       测试函数 在SRAM指定地址写入1个字节
 * @param       addr    : 开始写入的地址(最大32bit)
 * @param       data    : 要写入的字节
 * @retval      无
 */
void sram_test_write(uint32_t addr, uint8_t data)
{
    sram_write(&data, addr, 1); /* 写入1个字节 */
}

/**
 * @brief       测试函数 在SRAM指定地址读取1个字节
 * @param       addr    : 开始读取的地址(最大32bit)
 * @retval      读取到的数据(1个字节)
 */
uint8_t sram_test_read(uint32_t addr)
{
    uint8_t data;
    sram_read(&data, addr, 1); /* 读取1个字节 */
    return data;
}



void sram_test_rw(uint32_t type)
{
    uint32_t addr;

    if (type == 1)
    {
//        for (addr = 0; addr < SRAM_SIZE; addr += BUF_BYTES)                  //写1M（写满）
        for (addr = 0; addr < HALF_SRAM_SIZE; addr += BUF_BYTES)               //512k
        {
            if (addr + BUF_BYTES > SRAM_SIZE) {
                printf("写入操作越界！\n");
                break;
            }
            memset(write_data, 0, sizeof(write_data));
            for (int i = 0; i < BUF_WORDS; i++)
            {
                uint16_t value = ((addr / 2) + i) % PATTERN_SIZE;
                write_data[i * 2] = (value >> 8) & 0xFF;
                write_data[i * 2 + 1] = value & 0xFF;
            }
            sram_write_u16((uint16_t *)write_data, addr, BUF_WORDS);
        }
    }

    if (type == 0)
    {
//        for (addr = 0; addr < SRAM_SIZE; addr += BUF_BYTES)
			  for (addr = 0; addr < HALF_SRAM_SIZE; addr += BUF_BYTES)
        {
            if (addr + BUF_BYTES > SRAM_SIZE) {
                printf("读取操作越界！\n");
                break;
            }
            memset(read_data, 0, sizeof(read_data));
            sram_read_u16((uint16_t *)read_data, addr, BUF_WORDS);
            udbd_cdc_tx_data(read_data, BUF_BYTES);
            delay_us(1000);
        }
    }
}
void my_sram_read(void)
{
    uint32_t addr;

//        for (addr = 0; addr < SRAM_SIZE; addr += BUF_BYTES)
			  for (addr = 0; addr < HALF_SRAM_SIZE; addr += BUF_BYTES)
        {
            if (addr + BUF_BYTES > SRAM_SIZE) {
                printf("读取操作越界！\n");
                break;
            }
            memset(read_data, 0, sizeof(read_data));
            sram_read_u16((uint16_t *)read_data, addr, BUF_WORDS);
            udbd_cdc_tx_data(read_data, BUF_BYTES);
            delay_us(1000);
        }
    
}

        uint16_t sramread_data[BUF_WORDS];
				
				 uint16_t temp_buf[BUF_WORDS + SAMPLE_INTERVAL] = {0};
void sram_side_read(uint8_t sin_cos_flag, uint8_t side_flag) {


    uint32_t start_addr = (side_flag == DEVICE_SIDE_DS) ? 0 : WS_DATA_ADDR_END;
    uint32_t end_addr = (side_flag == DEVICE_SIDE_DS) ? WS_DATA_ADDR_END : DS_DATA_ADDR_END;


    uint32_t result_index = 0;
    uint16_t result_data[SAMPLE_COUNT] = {0};

    uint16_t leftover[SAMPLE_INTERVAL] = {0};
    uint32_t leftover_len = 0;

    // 设置 FPGA RAM 标志
//    FPGA_RAM_FLAG(sin_cos_flag == sin ? 0 : 1);

    // 主循环
    for (uint32_t addr = start_addr; addr < end_addr; addr += BUF_BYTES) {
        // 地址越界检查
        if (addr >= SRAM_SIZE || addr + BUF_BYTES > SRAM_SIZE) {
            printf("Error: Address out of bounds: %u\n", addr);
            return;
        }


        // 读取 SRAM 数据
				sram_read_u16(sramread_data, addr, BUF_WORDS);


        // 拼接数据
        memcpy(temp_buf, leftover, leftover_len * sizeof(uint16_t));
        memcpy(temp_buf + leftover_len, sramread_data, BUF_WORDS * sizeof(uint16_t));
        uint32_t total_len = leftover_len + BUF_WORDS;


				for (uint32_t i = 0; i < total_len; i += SAMPLE_INTERVAL) {
							if (result_index >= SAMPLE_COUNT) {
									printf("Warning: result_data full at index %u\n", result_index);
									break;  // 立即跳出内层循环
							}
							result_data[result_index++] = temp_buf[i];
					}

				if (result_index >= SAMPLE_COUNT) {
									break;  // 跳出外层主循环
					}

        // 保存残留数据
        leftover_len = total_len % SAMPLE_INTERVAL;
        if (leftover_len > 0) {
            memcpy(leftover, temp_buf + total_len - leftover_len, leftover_len * sizeof(uint16_t));
        }
    }

    // 存储结果
				size_t copy_size = result_index * sizeof(uint16_t);

        swap_uint16_array(result_data,SAMPLE_COUNT);

				memcpy(sin_cos_flag == sin ? sin_data : cos_data, result_data, copy_size);
}
void swap_uint16_array(uint16_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint16_t val = data[i];
        data[i] = (val >> 8) | (val << 8);  // 交换高低字节
    }
}

void validate_sram_rw()
{
    uint32_t addr;
    int ret = 1; // 假设验证成功

    // 写入数据
    sram_test_rw(1);

    // 读取数据并验证
    for (addr = 0; addr < SRAM_SIZE; addr += BUF_BYTES)
    {
        memset(read_data, 0, sizeof(read_data));
        sram_read_u16((uint16_t *)read_data, addr, BUF_WORDS);

        // 计算预期的写入数据
        for (int i = 0; i < BUF_WORDS; i++)
        {
            uint16_t expected_value = ((addr / 2) + i) % PATTERN_SIZE;
            uint8_t *expected_data = (uint8_t *)&expected_value;
            if (read_data[i * 2] != expected_data[1] || read_data[i * 2 + 1] != expected_data[0])
            {
                ret = 0; // 发现数据不一致
                break;
            }
        }

        if (ret == 0)
        {
            break;
        }
    }

    if (ret)
    {
        printf("SRAM 读写验证成功！\n");
    }
    else
    {
        printf("SRAM 读写验证失败！\n");
    }
}