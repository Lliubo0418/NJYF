/**
 ****************************************************************************************************
 * @file        spi.h
 * @author      
 * @version     V1.0
 * @date        2021-10-23
 * @brief       SPI 驱动代码
 * @license     Copyright (c) 2020-2032, 常州潞城
 ****************************************************************************************************
 * @attention
 *
 *
 ****************************************************************************************************
 */

 #ifndef __SPI_H
 #define __SPI_H
 
 #include "sys.h"
 #include "delay.h"
 
 /******************************************************************************************/
 /* SPI2 引脚 定义 */
 
 #define SPI1_SCK_GPIO_PORT              GPIOB
 #define SPI1_SCK_GPIO_PIN               GPIO_PIN_3
 #define SPI1_SCK_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */
 
 #define SPI1_MISO_GPIO_PORT             GPIOB
 #define SPI1_MISO_GPIO_PIN              GPIO_PIN_4
 #define SPI1_MISO_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */
 
 #define SPI1_MOSI_GPIO_PORT             GPIOB
 #define SPI1_MOSI_GPIO_PIN              GPIO_PIN_5
 #define SPI1_MOSI_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */
 
 #define SPI1_CS_GPIO_PORT               GPIOA
 #define SPI1_CS_GPIO_PIN                GPIO_PIN_15
 #define SPI1_CS_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

 #define SPI2_SCK_GPIO_PORT              GPIOB
 #define SPI2_SCK_GPIO_PIN               GPIO_PIN_13
 #define SPI2_SCK_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */
 
 #define SPI2_MOSI_GPIO_PORT             GPIOB
 #define SPI2_MOSI_GPIO_PIN              GPIO_PIN_15
 #define SPI2_MOSI_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */

 #define SPI2_CS_SIN_GPIO_PORT           GPIOB
 #define SPI2_CS_SIN_GPIO_PIN            GPIO_PIN_10
 #define SPI2_CS_SIN_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */

 #define SPI2_CS_COS_GPIO_PORT           GPIOB
 #define SPI2_CS_COS_GPIO_PIN            GPIO_PIN_11
 #define SPI2_CS_COS_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */

 /* SPI1相关定义 */
 #define SPI1_SPI                        SPI1
 #define SPI1_SPI_CLK_ENABLE()           do{ __HAL_RCC_SPI1_CLK_ENABLE(); }while(0)    /* SPI1时钟使能 */
 
 #define SPI2_SPI                        SPI2
 #define SPI2_SPI_CLK_ENABLE()           do{ __HAL_RCC_SPI2_CLK_ENABLE(); }while(0)    /* SPI2时钟使能 */
 
 #define SPI1_CS(x)      do{ x ? \
    HAL_GPIO_WritePin(SPI1_CS_GPIO_PORT, SPI1_CS_GPIO_PIN, GPIO_PIN_SET) : \
    HAL_GPIO_WritePin(SPI1_CS_GPIO_PORT, SPI1_CS_GPIO_PIN, GPIO_PIN_RESET); \
}while(0)

#define SPI2_CS_SIN(x)   do{ x ? \
    HAL_GPIO_WritePin(SPI2_CS_SIN_GPIO_PORT, SPI2_CS_SIN_GPIO_PIN, GPIO_PIN_SET) : \
    HAL_GPIO_WritePin(SPI2_CS_SIN_GPIO_PORT, SPI2_CS_SIN_GPIO_PIN, GPIO_PIN_RESET); \
}while(0)

#define SPI2_CS_COS(x)   do{ x ? \
    HAL_GPIO_WritePin(SPI2_CS_COS_GPIO_PORT, SPI2_CS_COS_GPIO_PIN, GPIO_PIN_SET) : \
    HAL_GPIO_WritePin(SPI2_CS_COS_GPIO_PORT, SPI2_CS_COS_GPIO_PIN, GPIO_PIN_RESET); \
}while(0)

/******************************************************************************************/

/* SPI总线速度设置 */
#define SPI_SPEED_2 0
#define SPI_SPEED_4 1
#define SPI_SPEED_8 2
#define SPI_SPEED_16 3
#define SPI_SPEED_32 4
#define SPI_SPEED_64 5
#define SPI_SPEED_128 6
#define SPI_SPEED_256 7

void spi1_init(void);
void spi1_set_speed(uint8_t speed);
uint16_t spi1_write_bit16(uint16_t txdata);
uint16_t spi1_read_bit16(void);

void spi2_init(void);
uint8_t spi2_write_byte(uint8_t txdata);

#endif
