/**
 * *****************************************************************************
 * @file    bsp_mb_crc.h
 * @brief   mb_crc 模块头文件
 * *****************************************************************************
 */

/** Define to prevent recursive inclusion ----------------------------------- */
#ifndef __BSP_MB_CRC_H
#define __BSP_MB_CRC_H

#ifdef __cplusplus
    extern "C" {
#endif

/** Includes ---------------------------------------------------------------- */
#include "bsp.h"

#ifndef BSP_MB_CRC_EN
    #define BSP_MB_CRC_EN           1
#endif
#if (BSP_MB_CRC_EN)

/** Functions --------------------------------------------------------------- */
uint16_t bsp_mb_crc16_calculate(uint8_t *buf, uint32_t len);

/** ------------------------------------------------------------------------- */
#endif  /* BSP_MB_CRC_EN */

#ifdef __cplusplus
    }
#endif

#endif  /* __BSP_MB_CRC_H */

/******************************* (END OF FILE) ********************************/
