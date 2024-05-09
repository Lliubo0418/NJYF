#ifndef __CXXEN_H__
#define __CXXEN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"	
	
void Set_MCU_CXCON_ON_OFF(unsigned char status);
void Set_MCU_CX1EN_ON_OFF(unsigned char flag);	
void Set_MCU_CX2EN_ON_OFF(unsigned char flag);	

void Set_MCU_CX1P(uint8_t Pulse_width);	
void Set_MCU_CX2P(uint8_t Pulse_width);
	
	
	
#ifdef __cplusplus
}
#endif

#endif

