//------------------------------------------------------------------------------
// Copyright 2014 Silicon Laboratories, Inc.
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the Silicon Laboratories End User
// License Agreement which accompanies this distribution, and is available at
// http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt
// Original content and implementation provided by Silicon Laboratories.
//------------------------------------------------------------------------------
//Supported Devices:
//  C8051F380_GQ
//  C8051F381_GM
//  C8051F381_GQ
//  C8051F382_GQ
//  C8051F383_GM
//  C8051F383_GQ
//  C8051F384_GQ
//  C8051F385_GM
//  C8051F385_GQ
//  C8051F386_GQ
//  C8051F387_GM
//  C8051F387_GQ
//  C8051F38C_GM
//  C8051F38C_GQ

#ifndef SI_C8051F380_DEFS_H
#define SI_C8051F380_DEFS_H

#include <si_toolchain.h>

//-----------------------------------------------------------------------------
// Register Definitions
//-----------------------------------------------------------------------------
//SI_SFR(ACC,      0xE0); ///< Accumulator
//SI_SFR(ADC0CF,   0xBC); ///< ADC0 Configuration
//SI_SFR(ADC0CN,   0xE8); ///< ADC0 Control
//SI_SFR(ADC0GTH,  0xC4); ///< ADC0 Greater-Than High Byte
//SI_SFR(ADC0GTL,  0xC3); ///< ADC0 Greater-Than Low Byte
//SI_SFR(ADC0H,    0xBE); ///< ADC0 Data Word High Byte
//SI_SFR(ADC0L,    0xBD); ///< ADC0 Data Word Low Byte
//SI_SFR(ADC0LTH,  0xC6); ///< ADC0 Less-Than High Byte
//SI_SFR(ADC0LTL,  0xC5); ///< ADC0 Less-Than Low Byte
//SI_SFR(AMX0N,    0xBA); ///< AMUX0 Negative Multiplexer Selection
//SI_SFR(AMX0P,    0xBB); ///< AMUX0 Positive Multiplexer Selection
//SI_SFR(B,        0xF0); ///< B Register
//SI_SFR(CKCON,    0x8E); ///< Clock Control
//SI_SFR(CKCON1,   0xE4); ///< Clock Control 1
//SI_SFR(CLKMUL,   0xB9); ///< Clock Multiplier Control
//SI_SFR(CLKSEL,   0xA9); ///< Clock Select
//SI_SFR(CPT0CN,   0x9B); ///< Comparator 0 Control
//SI_SFR(CPT0MD,   0x9D); ///< Comparator 0 Mode
//SI_SFR(CPT0MX,   0x9F); ///< Comparator 0 Multiplexer Selection
//SI_SFR(CPT1CN,   0x9A); ///< Comparator 1 Control
//SI_SFR(CPT1MD,   0x9C); ///< Comparator 1 Mode
//SI_SFR(CPT1MX,   0x9E); ///< Comparator 1 Multiplexer Selection
//SI_SFR(DPH,      0x83); ///< Data Pointer High
//SI_SFR(DPL,      0x82); ///< Data Pointer Low
//SI_SFR(EIE1,     0xE6); ///< Extended Interrupt Enable 1
//SI_SFR(EIE2,     0xE7); ///< Extended Interrupt Enable 2
//SI_SFR(EIP1,     0xF6); ///< Extended Interrupt Priority 1
//SI_SFR(EIP2,     0xF7); ///< Extended Interrupt Priority 2
//SI_SFR(EMI0CF,   0x85); ///< External Memory Configuration
//SI_SFR(EMI0CN,   0xAA); ///< External Memory Interface Control
//SI_SFR(EMI0TC,   0x84); ///< External Memory Timing Control
//SI_SFR(FLKEY,    0xB7); ///< Flash Lock and Key
//SI_SFR(FLSCL,    0xB6); ///< Flash Scale
//SI_SFR(IE,       0xA8); ///< Interrupt Enable
//SI_SFR(IP,       0xB8); ///< Interrupt Priority
//SI_SFR(IT01CF,   0xE4); ///< INT0/INT1 Configuration
//SI_SFR(OSCICL,   0xB3); ///< High Frequency Oscillator Calibration
//SI_SFR(OSCICN,   0xB2); ///< High Frequency Oscillator Control
//SI_SFR(OSCLCN,   0x86); ///< Internal Low Frequency Oscillator Control
//SI_SFR(OSCXCN,   0xB1); ///< External Oscillator Control
//SI_SFR(P0,       0x80); ///< Port 0 Pin Latch
//SI_SFR(P0MDIN,   0xF1); ///< Port 0 Input Mode
//SI_SFR(P0MDOUT,  0xA4); ///< Port 0 Output Mode
//SI_SFR(P0SKIP,   0xD4); ///< Port 0 Skip
//SI_SFR(P1,       0x90); ///< Port 1 Pin Latch
//SI_SFR(P1MDIN,   0xF2); ///< Port 1 Input Mode
//SI_SFR(P1MDOUT,  0xA5); ///< Port 1 Output Mode
//SI_SFR(P1SKIP,   0xD5); ///< Port 1 Skip
//SI_SFR(P2,       0xA0); ///< Port 2 Pin Latch
//SI_SFR(P2MDIN,   0xF3); ///< Port 2 Input Mode
//SI_SFR(P2MDOUT,  0xA6); ///< Port 2 Output Mode
//SI_SFR(P2SKIP,   0xD6); ///< Port 2 Skip
//SI_SFR(P3,       0xB0); ///< Port 3 Pin Latch
//SI_SFR(P3MDIN,   0xF4); ///< Port 3 Input Mode
//SI_SFR(P3MDOUT,  0xA7); ///< Port 3 Output Mode
//SI_SFR(P3SKIP,   0xDF); ///< Port 3 Skip
//SI_SFR(P4,       0xC7); ///< Port 4 Pin Latch
//SI_SFR(P4MDIN,   0xF5); ///< Port 4 Input Mode
//SI_SFR(P4MDOUT,  0xAE); ///< Port 4 Output Mode
//SI_SFR(PCA0CN,   0xD8); ///< PCA Control
//SI_SFR(PCA0CPH0, 0xFC); ///< PCA Channel 0 Capture Module High Byte
//SI_SFR(PCA0CPH1, 0xEA); ///< PCA Channel 1 Capture Module High Byte
//SI_SFR(PCA0CPH2, 0xEC); ///< PCA Channel 2 Capture Module High Byte
//SI_SFR(PCA0CPH3, 0xEE); ///< PCA Channel 3 Capture Module High Byte
//SI_SFR(PCA0CPH4, 0xFE); ///< PCA Channel 4 Capture Module High Byte
//SI_SFR(PCA0CPL0, 0xFB); ///< PCA Channel 0 Capture Module Low Byte
//SI_SFR(PCA0CPL1, 0xE9); ///< PCA Channel 1 Capture Module Low Byte
//SI_SFR(PCA0CPL2, 0xEB); ///< PCA Channel 2 Capture Module Low Byte
//SI_SFR(PCA0CPL3, 0xED); ///< PCA Channel 3 Capture Module Low Byte
//SI_SFR(PCA0CPL4, 0xFD); ///< PCA Channel 4 Capture Module Low Byte
//SI_SFR(PCA0CPM0, 0xDA); ///< PCA Channel 0 Capture/Compare Mode 0
//SI_SFR(PCA0CPM1, 0xDB); ///< PCA Channel 1 Capture/Compare Mode
//SI_SFR(PCA0CPM2, 0xDC); ///< PCA Channel 2 Capture/Compare Mode
//SI_SFR(PCA0CPM3, 0xDD); ///< PCA Channel 3 Capture/Compare Mode
//SI_SFR(PCA0CPM4, 0xDE); ///< PCA Channel 4 Capture/Compare Mode
//SI_SFR(PCA0H,    0xFA); ///< PCA Counter/Timer High Byte
//SI_SFR(PCA0L,    0xF9); ///< PCA Counter/Timer Low Byte
//SI_SFR(PCA0MD,   0xD9); ///< PCA Mode
//SI_SFR(PCON,     0x87); ///< Power Control
//SI_SFR(PFE0CN,   0xAF); ///< Prefetch Engine Control
//SI_SFR(PSCTL,    0x8F); ///< Program Store Control
//SI_SFR(PSW,      0xD0); ///< Program Status Word
//SI_SFR(REF0CN,   0xD1); ///< Voltage Reference Control
//SI_SFR(REG01CN,  0xC9); ///< Voltage Regulator Control
//SI_SFR(RSTSRC,   0xEF); ///< Reset Source
//SI_SFR(SBCON1,   0xAC); ///< UART1 Baud Rate Generator Control
//SI_SFR(SBRLH1,   0xB5); ///< UART1 Baud Rate Generator High Byte
//SI_SFR(SBRLL1,   0xB4); ///< UART1 Baud Rate Generator Low Byte
//SI_SFR(SBUF0,    0x99); ///< UART0 Serial Port Data Buffer
//SI_SFR(SBUF1,    0xD3); ///< UART1 Serial Port Data Buffer
//SI_SFR(SCON0,    0x98); ///< UART0 Serial Port Control
//SI_SFR(SCON1,    0xD2); ///< UART1 Serial Port Control
//SI_SFR(SFRPAGE,  0xBF); ///< SFR Page
//SI_SFR(SMB0ADM,  0xCE); ///< SMBus 0 Slave Address Mask
//SI_SFR(SMB0ADR,  0xCF); ///< SMBus 0 Slave Address
//SI_SFR(SMB0CF,   0xC1); ///< SMBus 0 Configuration
//SI_SFR(SMB0CN,   0xC0); ///< SMBus 0 Control
//SI_SFR(SMB0DAT,  0xC2); ///< SMBus 0 Data
//SI_SFR(SMB1ADM,  0xCE); ///< SMBus 1 Slave Address Mask
//SI_SFR(SMB1ADR,  0xCF); ///< SMBus 1 Slave Address
//SI_SFR(SMB1CF,   0xC1); ///< SMBus 1 Configuration
//SI_SFR(SMB1CN,   0xC0); ///< SMBus 1 Control
//SI_SFR(SMB1DAT,  0xC2); ///< SMBus 1 Data
//SI_SFR(SMBTC,    0xB9); ///< SMBus Timing and Pin Control
//SI_SFR(SMOD1,    0xE5); ///< UART1 Mode
//SI_SFR(SP,       0x81); ///< Stack Pointer
//SI_SFR(SPI0CFG,  0xA1); ///< SPI0 Configuration
//SI_SFR(SPI0CKR,  0xA2); ///< SPI0 Clock Rate
//SI_SFR(SPI0CN,   0xF8); ///< SPI0 Control
//SI_SFR(SPI0DAT,  0xA3); ///< SPI0 Data
//SI_SFR(TCON,     0x88); ///< Timer 0/1 Control
//SI_SFR(TH0,      0x8C); ///< Timer 0 High Byte
//SI_SFR(TH1,      0x8D); ///< Timer 1 High Byte
//SI_SFR(TL0,      0x8A); ///< Timer 0 Low Byte
//SI_SFR(TL1,      0x8B); ///< Timer 1 Low Byte
//SI_SFR(TMOD,     0x89); ///< Timer 0/1 Mode
//SI_SFR(TMR2CN,   0xC8); ///< Timer 2 Control
//SI_SFR(TMR2H,    0xCD); ///< Timer 2 High Byte
//SI_SFR(TMR2L,    0xCC); ///< Timer 2 Low Byte
//SI_SFR(TMR2RLH,  0xCB); ///< Timer 2 Reload High Byte
//SI_SFR(TMR2RLL,  0xCA); ///< Timer 2 Reload Low Byte
//SI_SFR(TMR3CN,   0x91); ///< Timer 3 Control
//SI_SFR(TMR3H,    0x95); ///< Timer 3 High Byte
//SI_SFR(TMR3L,    0x94); ///< Timer 3 Low Byte
//SI_SFR(TMR3RLH,  0x93); ///< Timer 3 Reload High Byte
//SI_SFR(TMR3RLL,  0x92); ///< Timer 3 Reload Low Byte
//SI_SFR(TMR4CN,   0x91); ///< Timer 4 Control
//SI_SFR(TMR4H,    0x95); ///< Timer 4 High Byte
//SI_SFR(TMR4L,    0x94); ///< Timer 4 Low Byte
//SI_SFR(TMR4RLH,  0x93); ///< Timer 4 Reload High Byte
//SI_SFR(TMR4RLL,  0x92); ///< Timer 4 Reload Low Byte
//SI_SFR(TMR5CN,   0xC8); ///< Timer 5 Control
//SI_SFR(TMR5H,    0xCD); ///< Timer 5 High Byte
//SI_SFR(TMR5L,    0xCC); ///< Timer 5 Low Byte
//SI_SFR(TMR5RLH,  0xCB); ///< Timer 5 Reload High Byte
//SI_SFR(TMR5RLL,  0xCA); ///< Timer 5 Reload Low Byte
//SI_SFR(USB0ADR,  0x96); ///< USB0 Indirect Address
//SI_SFR(USB0DAT,  0x97); ///< USB0 Data
//SI_SFR(USB0XCN,  0xD7); ///< USB0 Transceiver Control
//SI_SFR(VDM0CN,   0xFF); ///< VDD Supply Monitor Control
//SI_SFR(XBR0,     0xE1); ///< Port I/O Crossbar 0
//SI_SFR(XBR1,     0xE2); ///< Port I/O Crossbar 1
//SI_SFR(XBR2,     0xE3); ///< Port I/O Crossbar 2
//
////------------------------------------------------------------------------------
//// 16-bit Register Definitions (may not work on all compilers)
////------------------------------------------------------------------------------
//SI_SFR16(ADC0GT,  0xC3); ///< ADC0 Greater-Than
//SI_SFR16(ADC0,    0xBD); ///< ADC0 Data Word
//SI_SFR16(ADC0LT,  0xC5); ///< ADC0 Less-Than
//SI_SFR16(DP,      0x82); ///< Data Pointer
//SI_SFR16(PCA0CP0, 0xFB); ///< PCA Channel 0 Capture Module
//SI_SFR16(PCA0CP1, 0xE9); ///< PCA Channel 1 Capture Module
//SI_SFR16(PCA0CP2, 0xEB); ///< PCA Channel 2 Capture Module
//SI_SFR16(PCA0CP3, 0xED); ///< PCA Channel 3 Capture Module
//SI_SFR16(PCA0CP4, 0xFD); ///< PCA Channel 4 Capture Module
//SI_SFR16(PCA0,    0xF9); ///< PCA Counter/Timer
//SI_SFR16(SBRL1,   0xB4); ///< UART1 Baud Rate Generator
//SI_SFR16(TMR2,    0xCC); ///< Timer 2
//SI_SFR16(TMR2RL,  0xCA); ///< Timer 2 Reload
//SI_SFR16(TMR3,    0x94); ///< Timer 3
//SI_SFR16(TMR3RL,  0x92); ///< Timer 3 Reload
//SI_SFR16(TMR4,    0x94); ///< Timer 4
//SI_SFR16(TMR4RL,  0x92); ///< Timer 4 Reload
//SI_SFR16(TMR5,    0xCC); ///< Timer 5
//SI_SFR16(TMR5RL,  0xCA); ///< Timer 5 Reload

//------------------------------------------------------------------------------
// Indirect Register Definitions
//------------------------------------------------------------------------------
#define CLKREC   0x0F ///< USB0 Clock Recovery Control       
#define CMIE     0x0B ///< USB0 Common Interrupt Enable      
#define CMINT    0x06 ///< USB0 Common Interrupt             
#define E0CNT    0x16 ///< USB0 Endpoint0 Data Count         
#define E0CSR    0x11 ///< USB0 Endpoint0 Control            
#define EENABLE  0x1E ///< USB0 Endpoint Enable              
#define EINCSRH  0x12 ///< USB0 IN Endpoint Control High     
#define EINCSRL  0x11 ///< USB0 IN Endpoint Control          
#define EOUTCNTH 0x17 ///< USB0 OUT Endpoint Count High      
#define EOUTCNTL 0x16 ///< USB0 OUT Endpoint Count           
#define EOUTCSRH 0x15 ///< USB0 OUT Endpoint Control High    
#define EOUTCSRL 0x14 ///< USB0 OUT Endpoint Control         
#define FADDR    0x00 ///< USB0 Function Address             
#define FIFO0    0x20 ///< USB0 Endpoint 0 FIFO Access       
#define FIFO1    0x21 ///< USB0 Endpoint 1 FIFO Access       
#define FIFO2    0x22 ///< USB0 Endpoint 2 FIFO Access       
#define FIFO3    0x23 ///< USB0 Endpoint 3 FIFO Access       
#define FRAMEH   0x0D ///< USB0 Frame Number High            
#define FRAMEL   0x0C ///< USB0 Frame Number                 
#define IN1IE    0x07 ///< USB0 IN Endpoint Interrupt Enable 
#define IN1INT   0x02 ///< USB0 IN Endpoint Interrupt        
#define INDEX    0x0E ///< USB0 Endpoint Index               
#define OUT1IE   0x09 ///< USB0 OUT Endpoint Interrupt Enable
#define OUT1INT  0x04 ///< USB0 OUT Endpoint Interrupt       
#define POWER    0x01 ///< USB0 Power                        

//------------------------------------------------------------------------------
// Bit Definitions
//------------------------------------------------------------------------------

// ACC (Accumulator)
#define SFR_ACC 0xE0
SI_SBIT(ACC_ACC0, SFR_ACC, 0); ///< Accumulator Bit 0
SI_SBIT(ACC_ACC1, SFR_ACC, 1); ///< Accumulator Bit 1
SI_SBIT(ACC_ACC2, SFR_ACC, 2); ///< Accumulator Bit 2
SI_SBIT(ACC_ACC3, SFR_ACC, 3); ///< Accumulator Bit 3
SI_SBIT(ACC_ACC4, SFR_ACC, 4); ///< Accumulator Bit 4
SI_SBIT(ACC_ACC5, SFR_ACC, 5); ///< Accumulator Bit 5
SI_SBIT(ACC_ACC6, SFR_ACC, 6); ///< Accumulator Bit 6
SI_SBIT(ACC_ACC7, SFR_ACC, 7); ///< Accumulator Bit 7

// ADC0CN (ADC0 Control)
#define SFR_ADC0CN 0xE8
SI_SBIT(ADC0CN_ADCM0,  SFR_ADC0CN, 0); ///< Start of Conversion Mode Select Bit 0
SI_SBIT(ADC0CN_ADCM1,  SFR_ADC0CN, 1); ///< Start of Conversion Mode Select Bit 1
SI_SBIT(ADC0CN_ADCM2,  SFR_ADC0CN, 2); ///< Start of Conversion Mode Select Bit 2
SI_SBIT(ADC0CN_ADWINT, SFR_ADC0CN, 3); ///< Window Compare Interrupt Flag        
SI_SBIT(ADC0CN_ADBUSY, SFR_ADC0CN, 4); ///< ADC Busy                             
SI_SBIT(ADC0CN_ADINT,  SFR_ADC0CN, 5); ///< Conversion Complete Interrupt Flag   
SI_SBIT(ADC0CN_ADTM,   SFR_ADC0CN, 6); ///< Track Mode                           
SI_SBIT(ADC0CN_ADEN,   SFR_ADC0CN, 7); ///< ADC Enable                           

// B (B Register)
#define SFR_B 0xF0
SI_SBIT(B_B0, SFR_B, 0); ///< B Register Bit 0
SI_SBIT(B_B1, SFR_B, 1); ///< B Register Bit 1
SI_SBIT(B_B2, SFR_B, 2); ///< B Register Bit 2
SI_SBIT(B_B3, SFR_B, 3); ///< B Register Bit 3
SI_SBIT(B_B4, SFR_B, 4); ///< B Register Bit 4
SI_SBIT(B_B5, SFR_B, 5); ///< B Register Bit 5
SI_SBIT(B_B6, SFR_B, 6); ///< B Register Bit 6
SI_SBIT(B_B7, SFR_B, 7); ///< B Register Bit 7

// IE (Interrupt Enable)
#define SFR_IE 0xA8
SI_SBIT(IE_EX0,   SFR_IE, 0); ///< External Interrupt 0 Enable
SI_SBIT(IE_ET0,   SFR_IE, 1); ///< Timer 0 Interrupt Enable   
SI_SBIT(IE_EX1,   SFR_IE, 2); ///< External Interrupt 1 Enable
SI_SBIT(IE_ET1,   SFR_IE, 3); ///< Timer 1 Interrupt Enable   
SI_SBIT(IE_ES0,   SFR_IE, 4); ///< UART0 Interrupt Enable     
SI_SBIT(IE_ET2,   SFR_IE, 5); ///< Timer 2 Interrupt Enable   
SI_SBIT(IE_ESPI0, SFR_IE, 6); ///< SPI0 Interrupt Enable      
SI_SBIT(IE_EA,    SFR_IE, 7); ///< All Interrupts Enable      

// IP (Interrupt Priority)
#define SFR_IP 0xB8
SI_SBIT(IP_PX0,   SFR_IP, 0); ///< External Interrupt 0 Priority Control                        
SI_SBIT(IP_PT0,   SFR_IP, 1); ///< Timer 0 Interrupt Priority Control                           
SI_SBIT(IP_PX1,   SFR_IP, 2); ///< External Interrupt 1 Priority Control                        
SI_SBIT(IP_PT1,   SFR_IP, 3); ///< Timer 1 Interrupt Priority Control                           
SI_SBIT(IP_PS0,   SFR_IP, 4); ///< UART0 Interrupt Priority Control                             
SI_SBIT(IP_PT2,   SFR_IP, 5); ///< Timer 2 Interrupt Priority Control                           
SI_SBIT(IP_PSPI0, SFR_IP, 6); ///< Serial Peripheral Interface (SPI0) Interrupt Priority Control

// P0 (Port 0 Pin Latch)
#define SFR_P0 0x80
SI_SBIT(P0_B0, SFR_P0, 0); ///< Port 0 Bit 0 Latch
SI_SBIT(P0_B1, SFR_P0, 1); ///< Port 0 Bit 1 Latch
SI_SBIT(P0_B2, SFR_P0, 2); ///< Port 0 Bit 2 Latch
SI_SBIT(P0_B3, SFR_P0, 3); ///< Port 0 Bit 3 Latch
SI_SBIT(P0_B4, SFR_P0, 4); ///< Port 0 Bit 4 Latch
SI_SBIT(P0_B5, SFR_P0, 5); ///< Port 0 Bit 5 Latch
SI_SBIT(P0_B6, SFR_P0, 6); ///< Port 0 Bit 6 Latch
SI_SBIT(P0_B7, SFR_P0, 7); ///< Port 0 Bit 7 Latch

// P1 (Port 1 Pin Latch)
#define SFR_P1 0x90
SI_SBIT(P1_B0, SFR_P1, 0); ///< Port 1 Bit 0 Latch
SI_SBIT(P1_B1, SFR_P1, 1); ///< Port 1 Bit 1 Latch
SI_SBIT(P1_B2, SFR_P1, 2); ///< Port 1 Bit 2 Latch
SI_SBIT(P1_B3, SFR_P1, 3); ///< Port 1 Bit 3 Latch
SI_SBIT(P1_B4, SFR_P1, 4); ///< Port 1 Bit 4 Latch
SI_SBIT(P1_B5, SFR_P1, 5); ///< Port 1 Bit 5 Latch
SI_SBIT(P1_B6, SFR_P1, 6); ///< Port 1 Bit 6 Latch
SI_SBIT(P1_B7, SFR_P1, 7); ///< Port 1 Bit 7 Latch

// P2 (Port 2 Pin Latch)
#define SFR_P2 0xA0
SI_SBIT(P2_B0, SFR_P2, 0); ///< Port 2 Bit 0 Latch
SI_SBIT(P2_B1, SFR_P2, 1); ///< Port 2 Bit 1 Latch
SI_SBIT(P2_B2, SFR_P2, 2); ///< Port 2 Bit 2 Latch
SI_SBIT(P2_B3, SFR_P2, 3); ///< Port 2 Bit 3 Latch
SI_SBIT(P2_B4, SFR_P2, 4); ///< Port 2 Bit 4 Latch
SI_SBIT(P2_B5, SFR_P2, 5); ///< Port 2 Bit 5 Latch
SI_SBIT(P2_B6, SFR_P2, 6); ///< Port 2 Bit 6 Latch
SI_SBIT(P2_B7, SFR_P2, 7); ///< Port 2 Bit 7 Latch

// P3 (Port 3 Pin Latch)
#define SFR_P3 0xB0
SI_SBIT(P3_B0, SFR_P3, 0); ///< Port 3 Bit 0 Latch
SI_SBIT(P3_B1, SFR_P3, 1); ///< Port 3 Bit 1 Latch
SI_SBIT(P3_B2, SFR_P3, 2); ///< Port 3 Bit 2 Latch
SI_SBIT(P3_B3, SFR_P3, 3); ///< Port 3 Bit 3 Latch
SI_SBIT(P3_B4, SFR_P3, 4); ///< Port 3 Bit 4 Latch
SI_SBIT(P3_B5, SFR_P3, 5); ///< Port 3 Bit 5 Latch
SI_SBIT(P3_B6, SFR_P3, 6); ///< Port 3 Bit 6 Latch
SI_SBIT(P3_B7, SFR_P3, 7); ///< Port 3 Bit 7 Latch

// PCA0CN (PCA Control)
#define SFR_PCA0CN 0xD8
SI_SBIT(PCA0CN_CCF0, SFR_PCA0CN, 0); ///< PCA Module 0 Capture/Compare Flag
SI_SBIT(PCA0CN_CCF1, SFR_PCA0CN, 1); ///< PCA Module 1 Capture/Compare Flag
SI_SBIT(PCA0CN_CCF2, SFR_PCA0CN, 2); ///< PCA Module 2 Capture/Compare Flag
SI_SBIT(PCA0CN_CCF3, SFR_PCA0CN, 3); ///< PCA Module 3 Capture/Compare Flag
SI_SBIT(PCA0CN_CCF4, SFR_PCA0CN, 4); ///< PCA Module 4 Capture/Compare Flag
SI_SBIT(PCA0CN_CR,   SFR_PCA0CN, 6); ///< PCA Counter/Timer Run Control    
SI_SBIT(PCA0CN_CF,   SFR_PCA0CN, 7); ///< PCA Counter/Timer Overflow Flag  

// PSW (Program Status Word)
#define SFR_PSW 0xD0
SI_SBIT(PSW_PARITY, SFR_PSW, 0); ///< Parity Flag               
SI_SBIT(PSW_F1,     SFR_PSW, 1); ///< User Flag 1               
SI_SBIT(PSW_OV,     SFR_PSW, 2); ///< Overflow Flag             
SI_SBIT(PSW_RS0,    SFR_PSW, 3); ///< Register Bank Select Bit 0
SI_SBIT(PSW_RS1,    SFR_PSW, 4); ///< Register Bank Select Bit 1
SI_SBIT(PSW_F0,     SFR_PSW, 5); ///< User Flag 0               
SI_SBIT(PSW_AC,     SFR_PSW, 6); ///< Auxiliary Carry Flag      
SI_SBIT(PSW_CY,     SFR_PSW, 7); ///< Carry Flag                

// SCON0 (UART0 Serial Port Control)
#define SFR_SCON0 0x98
SI_SBIT(SCON0_RI,    SFR_SCON0, 0); ///< Receive Interrupt Flag             
SI_SBIT(SCON0_TI,    SFR_SCON0, 1); ///< Transmit Interrupt Flag            
SI_SBIT(SCON0_RB8,   SFR_SCON0, 2); ///< Ninth Receive Bit                  
SI_SBIT(SCON0_TB8,   SFR_SCON0, 3); ///< Ninth Transmission Bit             
SI_SBIT(SCON0_REN,   SFR_SCON0, 4); ///< Receive Enable                     
SI_SBIT(SCON0_MCE,   SFR_SCON0, 5); ///< Multiprocessor Communication Enable
SI_SBIT(SCON0_SMODE, SFR_SCON0, 7); ///< Serial Port 0 Operation Mode       

// SMB0CN (SMBus 0 Control)
#define SFR_SMB0CN 0xC0
SI_SBIT(SMB0CN_SI,      SFR_SMB0CN, 0); ///< SMBus Interrupt Flag            
SI_SBIT(SMB0CN_ACK,     SFR_SMB0CN, 1); ///< SMBus Acknowledge               
SI_SBIT(SMB0CN_ARBLOST, SFR_SMB0CN, 2); ///< SMBus Arbitration Lost Indicator
SI_SBIT(SMB0CN_ACKRQ,   SFR_SMB0CN, 3); ///< SMBus Acknowledge Request       
SI_SBIT(SMB0CN_STO,     SFR_SMB0CN, 4); ///< SMBus Stop Flag                 
SI_SBIT(SMB0CN_STA,     SFR_SMB0CN, 5); ///< SMBus Start Flag                
SI_SBIT(SMB0CN_TXMODE,  SFR_SMB0CN, 6); ///< SMBus Transmit Mode Indicator   
SI_SBIT(SMB0CN_MASTER,  SFR_SMB0CN, 7); ///< SMBus Master/Slave Indicator    

// SMB1CN (SMBus 1 Control)
#define SFR_SMB1CN 0xC0
SI_SBIT(SMB1CN_SI,      SFR_SMB1CN, 0); ///< SMBus Interrupt Flag            
SI_SBIT(SMB1CN_ACK,     SFR_SMB1CN, 1); ///< SMBus Acknowledge               
SI_SBIT(SMB1CN_ARBLOST, SFR_SMB1CN, 2); ///< SMBus Arbitration Lost Indicator
SI_SBIT(SMB1CN_ACKRQ,   SFR_SMB1CN, 3); ///< SMBus Acknowledge Request       
SI_SBIT(SMB1CN_STO,     SFR_SMB1CN, 4); ///< SMBus Stop Flag                 
SI_SBIT(SMB1CN_STA,     SFR_SMB1CN, 5); ///< SMBus Start Flag                
SI_SBIT(SMB1CN_TXMODE,  SFR_SMB1CN, 6); ///< SMBus Transmit Mode Indicator   
SI_SBIT(SMB1CN_MASTER,  SFR_SMB1CN, 7); ///< SMBus Master/Slave Indicator    

// SPI0CN (SPI0 Control)
#define SFR_SPI0CN 0xF8
SI_SBIT(SPI0CN_SPIEN,  SFR_SPI0CN, 0); ///< SPI0 Enable            
SI_SBIT(SPI0CN_TXBMT,  SFR_SPI0CN, 1); ///< Transmit Buffer Empty  
SI_SBIT(SPI0CN_NSSMD0, SFR_SPI0CN, 2); ///< Slave Select Mode Bit 0
SI_SBIT(SPI0CN_NSSMD1, SFR_SPI0CN, 3); ///< Slave Select Mode Bit 1
SI_SBIT(SPI0CN_RXOVRN, SFR_SPI0CN, 4); ///< Receive Overrun Flag   
SI_SBIT(SPI0CN_MODF,   SFR_SPI0CN, 5); ///< Mode Fault Flag        
SI_SBIT(SPI0CN_WCOL,   SFR_SPI0CN, 6); ///< Write Collision Flag   
SI_SBIT(SPI0CN_SPIF,   SFR_SPI0CN, 7); ///< SPI0 Interrupt Flag    

// TCON (Timer 0/1 Control)
#define SFR_TCON 0x88
SI_SBIT(TCON_IT0, SFR_TCON, 0); ///< Interrupt 0 Type Select
SI_SBIT(TCON_IE0, SFR_TCON, 1); ///< External Interrupt 0   
SI_SBIT(TCON_IT1, SFR_TCON, 2); ///< Interrupt 1 Type Select
SI_SBIT(TCON_IE1, SFR_TCON, 3); ///< External Interrupt 1   
SI_SBIT(TCON_TR0, SFR_TCON, 4); ///< Timer 0 Run Control    
SI_SBIT(TCON_TF0, SFR_TCON, 5); ///< Timer 0 Overflow Flag  
SI_SBIT(TCON_TR1, SFR_TCON, 6); ///< Timer 1 Run Control    
SI_SBIT(TCON_TF1, SFR_TCON, 7); ///< Timer 1 Overflow Flag  

// TMR2CN (Timer 2 Control)
#define SFR_TMR2CN 0xC8
SI_SBIT(TMR2CN_T2XCLK,  SFR_TMR2CN, 0); ///< Timer 2 External Clock Select    
SI_SBIT(TMR2CN_T2CSS,   SFR_TMR2CN, 1); ///< Timer 2 Capture Source Select    
SI_SBIT(TMR2CN_TR2,     SFR_TMR2CN, 2); ///< Timer 2 Run Control              
SI_SBIT(TMR2CN_T2SPLIT, SFR_TMR2CN, 3); ///< Timer 2 Split Mode Enable        
SI_SBIT(TMR2CN_TF2CEN,  SFR_TMR2CN, 4); ///< Timer 2 Capture Enable           
SI_SBIT(TMR2CN_TF2LEN,  SFR_TMR2CN, 5); ///< Timer 2 Low Byte Interrupt Enable
SI_SBIT(TMR2CN_TF2L,    SFR_TMR2CN, 6); ///< Timer 2 Low Byte Overflow Flag   
SI_SBIT(TMR2CN_TF2H,    SFR_TMR2CN, 7); ///< Timer 2 High Byte Overflow Flag  

// TMR5CN (Timer 5 Control)
#define SFR_TMR5CN 0xC8
SI_SBIT(TMR5CN_T5XCLK,  SFR_TMR5CN, 0); ///< Timer 5 External Clock Select    
SI_SBIT(TMR5CN_TR5,     SFR_TMR5CN, 2); ///< Timer 5 Run Control              
SI_SBIT(TMR5CN_T5SPLIT, SFR_TMR5CN, 3); ///< Timer 5 Split Mode Enable        
SI_SBIT(TMR5CN_TF5LEN,  SFR_TMR5CN, 5); ///< Timer 5 Low Byte Interrupt Enable
SI_SBIT(TMR5CN_TF5L,    SFR_TMR5CN, 6); ///< Timer 5 Low Byte Overflow Flag   
SI_SBIT(TMR5CN_TF5H,    SFR_TMR5CN, 7); ///< Timer 5 High Byte Overflow Flag  

//------------------------------------------------------------------------------
// Interrupt Definitions
//------------------------------------------------------------------------------
#define INT0_IRQn    0  ///< External Interrupt 0  
#define TIMER0_IRQn  1  ///< Timer 0 Overflow      
#define INT1_IRQn    2  ///< External Interrupt 1  
#define TIMER1_IRQn  3  ///< Timer 1 Overflow      
#define UART0_IRQn   4  ///< UART 0                
#define TIMER2_IRQn  5  ///< Timer 2 Overflow      
#define SPI0_IRQn    6  ///< SPI0                  
#define SMBUS0_IRQn  7  ///< SMBus 0               
#define USB0_IRQn    8  ///< USB0                  
#define ADC0WC_IRQn  9  ///< ADC0 Window Compare   
#define ADC0EOC_IRQn 10 ///< ADC0 End of Conversion
#define PCA0_IRQn    11 ///< PCA0                  
#define CMP0_IRQn    12 ///< Comparator 0          
#define CMP1_IRQn    13 ///< Comparator 1          
#define TIMER3_IRQn  14 ///< Timer 3 Overflow      
#define VBUSLVL_IRQn 15 ///< VBUS Level            
#define UART1_IRQn   16 ///< UART 1                
#define SMBUS1_IRQn  18 ///< SMBus 1               
#define TIMER4_IRQn  19 ///< Timer 4 Overflow      
#define TIMER5_IRQn  20 ///< Timer 5 Overflow      

//------------------------------------------------------------------------------
// SFR Page Definitions
//------------------------------------------------------------------------------
#define LEGACY_PAGE 0x00 ///< Legacy SFR Page                   
#define CONFIG_PAGE 0x0F ///< System and Port Configuration Page
#define SMBUS1_PAGE 0x0F ///< SMBus 1 Page                      

//-----------------------------------------------------------------------------
// SDCC PDATA External Memory Paging Support
//-----------------------------------------------------------------------------

#if defined __SDCC

SI_SFR(_XPAGE, 0xAA); // Point to the EMI0CN register

#endif

#endif // SI_C8051F380_DEFS_H
//-eof--------------------------------------------------------------------------

