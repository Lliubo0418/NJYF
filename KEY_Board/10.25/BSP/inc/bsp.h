/**
 * *****************************************************************************
 * @file    bsp.h
 * @brief   模块头文件
 * *****************************************************************************
 */

/** Define to prevent recursive inclusion ----------------------------------- */
#ifndef __BSP_H
#define __BSP_H

#ifdef __cplusplus
    extern "C" {
#endif

/** Includes ---------------------------------------------------------------- */
#include "bsp_mcu.h"

#ifndef BSP_EN
    #define BSP_EN                  1
#endif
#if (BSP_EN)

/** Defines ----------------------------------------------------------------- */
#define GLOBAL_DEBUG_EN             0

/** Functions --------------------------------------------------------------- */
int bsp_init(void);


/** ------------------------------------------------------------------------- */
#endif  /* BSP_EN */

#ifdef __cplusplus
    }
#endif

#endif  /* __BSP_H */

/******************************* (END OF FILE) ********************************/