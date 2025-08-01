
/**
 ****************************************************************************************************
 * @file        fsmc.c
 * @author
 * @version     V1.1
 * @date        2025-03-20
 * @brief       FSMC口初始化代码
 * @license     Copyright (c) 2020-2032, 常州潞城
 ****************************************************************************************************

*/
#include "sram.h"
#include "fpga.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
#include "data_process.h"
#include "com_process.h"



// 全局缓冲区
uint8_t write_data[BUF_BYTES];    // 保留：用于其他函数写入SRAM数据
uint8_t read_data[BUF_BYTES];     // 保留：用于其他函数读取SRAM数据
uint16_t leftover_buf[SAMPLE_INTERVAL]; // 残留数据缓冲区
uint8_t leftover_len = 0;             // 残留数据长度
extern uint8_t sin_data[SAMPLE_COUNT]; // 存储正弦采样结果（高8位）
extern uint8_t cos_data[SAMPLE_COUNT]; // 存储余弦采样结果（高8位）
extern uint8_t ws_ds_flag;        // 保留：标志位，用于区分WS/DS数据源
uint16_t sramread_data[BUF_WORDS]; // SRAM读取缓冲区
uint16_t temp_buf[BUF_WORDS + SAMPLE_INTERVAL] = {0}; // 临时缓冲区，含余量



SRAM_HandleTypeDef g_sram_handler; /* SRAM句柄 */

/**
 * @brief       初始化 外部SRAM
 * @param       无
 * @retval      无
 */
void sram_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    FSMC_NORSRAM_TimingTypeDef fsmc_readwritetim;

    SRAM_CS_GPIO_CLK_ENABLE();    /* SRAM_CS脚时钟使能 */
    SRAM_WR_GPIO_CLK_ENABLE();    /* SRAM_WR脚时钟使能 */
    SRAM_RD_GPIO_CLK_ENABLE();    /* SRAM_RD脚时钟使能 */
    __HAL_RCC_FSMC_CLK_ENABLE();  /* 使能FSMC时钟 */
    
    __HAL_RCC_GPIOD_CLK_ENABLE(); /* 使能GPIOD时钟 */
    __HAL_RCC_GPIOE_CLK_ENABLE(); /* 使能GPIOE时钟 */
    __HAL_RCC_GPIOF_CLK_ENABLE(); /* 使能GPIOF时钟 */
    __HAL_RCC_GPIOG_CLK_ENABLE(); /* 使能GPIOG时钟 */

    gpio_init_struct.Pin = SRAM_CS_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = GPIO_AF12_FSMC;
    HAL_GPIO_Init(SRAM_CS_GPIO_PORT, &gpio_init_struct); /* SRAM_CS引脚模式设置 */

    gpio_init_struct.Pin = SRAM_WR_GPIO_PIN;
    HAL_GPIO_Init(SRAM_WR_GPIO_PORT, &gpio_init_struct); /* SRAM_WR引脚模式设置 */

    gpio_init_struct.Pin = SRAM_RD_GPIO_PIN;
    HAL_GPIO_Init(SRAM_RD_GPIO_PORT, &gpio_init_struct); /* SRAM_RD引脚模式设置 */

    /*Configure GPIO pins （FSMC_D5~D12）: PD8 PD9 PD10（FSMC_D13~D15）
                             PD14 PD15（FSMC_D0~D1）
                             PD0 PD1（FSMC_D2~D3）
                             PD11 PD12（FSMC_A16~A17） */
    gpio_init_struct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15 |
                          GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_11 | GPIO_PIN_12 |
                          GPIO_PIN_11 | GPIO_PIN_12;
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);

    /*Configure GPIO pins（FSMC_D5~D12） : PE7 PE8 PE9 PE10 PE11 PE12 PE13 PE14 PE15 */
    gpio_init_struct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
                          GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);

    /*Configure GPIO pins（FSMC_A0~A9） : PF0 PF1 PF2 PF3 PF4 PF5 PF12 PF13 PF14 PF15 */
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
                          GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOF, &gpio_init_struct);

    /*Configure GPIO pins（FSMC_A10~A15） :  */
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOG, &gpio_init_struct);

    g_sram_handler.Instance = FSMC_NORSRAM_DEVICE;
    g_sram_handler.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;

    g_sram_handler.Init.NSBank = (SRAM_FSMC_NEX == 1) ? FSMC_NORSRAM_BANK1 : (SRAM_FSMC_NEX == 2) ? FSMC_NORSRAM_BANK2
                                                                         : (SRAM_FSMC_NEX == 3)   ? FSMC_NORSRAM_BANK3
                                                                                                  : FSMC_NORSRAM_BANK4; /* 根据配置选择FSMC_NE1~4 */
    g_sram_handler.Init.DataAddressMux      = FSMC_DATA_ADDRESS_MUX_DISABLE;    /* 地址/数据线不复用 */
    g_sram_handler.Init.MemoryType          = FSMC_MEMORY_TYPE_SRAM;            /* SRAM */
    g_sram_handler.Init.MemoryDataWidth     = FSMC_NORSRAM_MEM_BUS_WIDTH_16;    /* 16位数据宽度 */
    g_sram_handler.Init.BurstAccessMode     = FSMC_BURST_ACCESS_MODE_DISABLE;   /* 是否使能突发访问,仅对同步突发存储器有效,此处未用到 */
    g_sram_handler.Init.WaitSignalPolarity  = FSMC_WAIT_SIGNAL_POLARITY_LOW;    /* 等待信号的极性,仅在突发模式访问下有用 */
    g_sram_handler.Init.WaitSignalActive    = FSMC_WAIT_TIMING_BEFORE_WS;       /* 存储器是在等待周期之前的一个时钟周期还是等待周期期间使能NWAIT */
    g_sram_handler.Init.WriteOperation      = FSMC_WRITE_OPERATION_DISABLE;      /* 存储器写使能禁用 */
    g_sram_handler.Init.WaitSignal          = FSMC_WAIT_SIGNAL_DISABLE;         /* 等待使能位,此处未用到 */
    g_sram_handler.Init.ExtendedMode        = FSMC_EXTENDED_MODE_DISABLE;       /* 读写使用相同的时序 */
    g_sram_handler.Init.AsynchronousWait    = FSMC_ASYNCHRONOUS_WAIT_DISABLE;   /* 是否使能同步传输模式下的等待信号,此处未用到 */
    g_sram_handler.Init.WriteBurst          = FSMC_WRITE_BURST_DISABLE;         /* 禁止突发写 */

    /* FSMC读时序控制寄存器 */
    fsmc_readwritetim.AddressSetupTime  = 0x02;         /* 地址建立时间（ADDSET）为2个HCLK 1/168M = 6ns*2 = 12ns */
    fsmc_readwritetim.AddressHoldTime   = 0x00;         /* 地址保持时间（ADDHLD）模式A未用到 */
    fsmc_readwritetim.DataSetupTime     = 0x08;         /* 数据保存时间为8个HCLK = 6*8 = 48ns */
    fsmc_readwritetim.BusTurnAroundDuration = 0x00;     /* 总线恢复时间 */
    fsmc_readwritetim.AccessMode        = FSMC_ACCESS_MODE_A;   /* 模式A */
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
        addr += 2;
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


/**
 * @brief 从SRAM读取正弦或余弦数据（16位），提取高8位，按采样间隔存储到sin_data或cos_data
 * @param sin_cos_flag 0: 正弦数据, 1: 余弦数据
 * @param side_flag 0: DEVICE_SIDE_DS, 1: WS_DATA_ADDR_END
 * @return SramReadStatus 返回操作状态
 */
SramReadStatus read_sram_sincos_data(uint8_t sin_cos_flag, uint8_t side_flag, uint16_t start_pos, uint16_t count) {
    // 确定SRAM读取范围
    uint32_t base_addr = (side_flag == DEVICE_SIDE_WS) ? 0 : WS_DATA_ADDR_END;
    uint32_t max_addr = (side_flag == DEVICE_SIDE_WS) ? WS_DATA_ADDR_END : DS_DATA_ADDR_END;

    // 计算实际的起始地址（start_pos 以 16 位数据为单位）
    uint32_t start_addr = base_addr + start_pos * sizeof(uint16_t);
    uint32_t end_addr = start_addr + count * SAMPLE_INTERVAL * sizeof(uint16_t);

    // 验证地址合法性
    if (start_addr >= SRAM_SIZE || end_addr > SRAM_SIZE || start_addr >= max_addr || end_addr > max_addr) {
        printf("Error: Invalid address range: start=%u, end=%u\n", start_addr, end_addr);
        return SRAM_ADDR_OUT_OF_BOUNDS;
    }

    // 验证 count 是否合法
    if (count == 0 || count > SAMPLE_COUNT) {
        printf("Error: Invalid count: %u\n", count);
        return SRAM_RESULT_FULL;
    }

    // 验证 start_pos 是否超出数据范围
    if (start_pos >= PATTERN_SIZE) {
        printf("Error: start_pos out of pattern range: %u\n", start_pos);
        return SRAM_ADDR_OUT_OF_BOUNDS;
    }

    uint8_t *target_data = (sin_cos_flag == SIN_FLAG) ? sin_data : cos_data;
    uint32_t result_index = 0;

    // 设置FPGA标志
    FPGA_RAM_FLAG(sin_cos_flag == SIN_FLAG ? 0 : 1);

    // 主循环：按 BUF_BYTES 步长读取 SRAM
    for (uint32_t addr = start_addr; addr < end_addr && result_index < count; addr += BUF_BYTES) {
        // 计算实际需要读取的字数
        uint32_t remaining_bytes = end_addr - addr;
        uint32_t read_words = (remaining_bytes + sizeof(uint16_t) - 1) / sizeof(uint16_t);
        read_words = (read_words > BUF_WORDS) ? BUF_WORDS : read_words;

        // 检查地址越界
        if (addr + read_words * sizeof(uint16_t) > SRAM_SIZE) {
            printf("Error: Address out of bounds: %u\n", addr);
            return SRAM_ADDR_OUT_OF_BOUNDS;
        }

        // 读取SRAM数据（16位）
        sram_read_u16(sramread_data, addr, read_words);

        // 拼接数据：残留数据 + 新读取数据
        memcpy(temp_buf, leftover_buf, leftover_len * sizeof(uint16_t));
        memcpy(temp_buf + leftover_len, sramread_data, read_words * sizeof(uint16_t));
        uint32_t total_len = leftover_len + read_words;

        // 采样数据，提取高8位
        for (uint32_t i = 0; i < total_len && result_index < count; i += SAMPLE_INTERVAL) {
            if (i < total_len) { // 确保不越界
                target_data[result_index++] = (uint8_t)(temp_buf[i] >> 8); // 提取高8位
            }
        }

        // 保存残留数据
        leftover_len = total_len % SAMPLE_INTERVAL;
        if (leftover_len > 0) {
            memcpy(leftover_buf, temp_buf + total_len - leftover_len, leftover_len * sizeof(uint16_t));
        }
    }

    return SRAM_READ_OK;
}

/**
 * @brief 交换uint16_t数组的高低字节（保留以备后续使用）
 * @param data 待处理数组
 * @param len 数组长度
 */

void swap_uint16_array(uint16_t *data, size_t len) {
//    for (size_t i = 0; i < len; i++) {
//        uint16_t val = data[i];
//        data[i] = (val >> 8) | (val << 8);  // 交换高低字节
//    }
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

uint8_t sram_test_read(uint32_t addr)
{
    uint8_t data;
    sram_read(&data, addr, 1); /* 读取1个字节 */
    return data;
}

/**
 * @brief       测试函数 在SRAM指定地址写入数据，然后再依次读出
 * @param       
 * @retval      
 */
void sram_test_rw(uint32_t type)
{
    uint32_t addr;

    if (type == 1)
    {
        for (addr = 0; addr < SRAM_SIZE; addr += BUF_BYTES)
        {

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
        for (addr = 0; addr < SRAM_SIZE; addr += BUF_BYTES)
        {

            memset(read_data, 0, sizeof(read_data));
            sram_read_u16((uint16_t *)read_data, addr, BUF_WORDS);
            udbd_cdc_tx_data(read_data, BUF_BYTES);
            delay_us(1000);
        }
    }
}
