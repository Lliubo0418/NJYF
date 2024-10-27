/**
 * *****************************************************************************
 * @file    bsp_mcu.c
 * @brief   mcu 模块文件
 * *****************************************************************************
 */

/** Includes ---------------------------------------------------------------- */
#include "bsp_mcu.h"

#if (BSP_MCU_EN)

/** Debug ------------------------------------------------------------------- */
#define DEBUG_EN                    0

#if (DEBUG_EN) && (GLOBAL_DEBUG_EN)
    #define PRINTF(...)             printf(__VA_ARGS__)
#else
    #define PRINTF(...)
#endif

/** Defines ----------------------------------------------------------------- */

/** Types ------------------------------------------------------------------- */

/** Variables --------------------------------------------------------------- */

/** Functions --------------------------------------------------------------- */
// static void bsp_mcu_clock_config(void);

/**
 * @brief   mcu 初始化
 * @param   none
 * @return  0 - OK; other - Error
 */
int bsp_mcu_init(void)
{
    // HAL_Init();
    // bsp_mcu_clock_config();
    return 0;
}

/**
 * @brief   mcu 中断使能
 * @param   none
 * @return  none
 */
void bsp_mcu_irq_enable(void)
{
    __set_PRIMASK(0);
}

/**
 * @brief   mcu 中断禁能
 * @param   none
 * @return  none
 */
void bsp_mcu_irq_disable(void)
{
    __set_PRIMASK(1);
}

/**
 * @brief   mcu 时钟配置
 * @param   none
 * @return  none
 */



/** ------------------------------------------------------------------------- */
#endif /* BSP_MCU_EN */
/******************************* (END OF FILE) ********************************/
