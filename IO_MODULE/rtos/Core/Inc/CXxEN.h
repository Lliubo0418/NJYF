#ifndef __CXXEN_H__
#define __CXXEN_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif
	
//static uint16_t MCU_CX1P_Pulse_width=8;
//static uint8_t MCU_CX1P_enable=0;
//static uint16_t MCU_CX2P_Pulse_width=8;
//static uint8_t MCU_CX2P_enable=0;

void Set_MCU_CXCON_ON_OFF(unsigned char status);
void Set_MCU_CX1EN_ON_OFF(unsigned char flag);	
void Set_MCU_CX2EN_ON_OFF(unsigned char flag);	
	
	

	
	
	
#ifdef __cplusplus
}
#endif

#endif

