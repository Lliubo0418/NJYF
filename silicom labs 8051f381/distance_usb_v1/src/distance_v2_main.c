/********************************************************************************
* FILE NAME           : LMD(按键式:单点/范围输出RS232 和RS485）.C
* Copyright           : 2012--2020 Lucheng Sensor Corporation,All Rights Reserved.
* Module Name         : 激光测距仪
* CPU                 : C8051F381
* Create Date         : 2013/09/05
* Author              : Kerry_Sun
* Abstract Description:
*

**------------------------ Revision History ----------------------------------**

* No     Version       Date          Revised By     Item    Description
* 1       V1.41         2015/03/13   Kerry                   first release

* 更改输出为三端式
*                       2015/04/13    Kerry                  更改激光机芯初始化 ：去掉PR
2         V1.42         2015/06/19    Kerry                  更改串口中断为高优先级
3         V1.351        2015/09/11    Kerry                  去掉DIR0，增加继电器和PNP的输出选择：=0 距离报警输出，=1 错误报警输出
4         V2.0          2015/10/19    kerry                  将RS232和RS485合并在一个程序里面
5         V2.2          2016/4/21    kerry                  修改RS485通讯多机死机问题
6.        V3.0          2020.06.16    kerry                  修改先按enter键，再按SET键出现错误显示的bug
7         V3.1          2021/04/09    Mark Dresser          Correct comm lockup bug, rename b -> bx, up->upx for KEIL toolset
8         V3.2      2021/06/21    Mark Dresser          Correct distance overflow error above 65 m.
*********************************************************************************/
//#include <config_381.h>

#include <math.h>
#include <stdio.h>
#include <absacc.h>
#include "compiler_defs.h"
#include "C8051F380_defs.h"
#include "SI_C8051F380_Defs.h"


#include "c8051f3xx.h"
#include "F3xx_USB0_Register.h"
#include "F3xx_USB0_Descriptor.h"
#include "F3xx_USB0_InterruptServiceRoutine.h"
#include "F3xx_USB0_Main.h"
#include "F3xx_USB0_Bulk.h"
#include "F3xx_Flash.h"

#define  uchar  unsigned char
#define  uint  unsigned int
#define  ulint  unsigned long int
#define lint  long int

#define version_major '3'
#define version_minor '2'

//----------------usb-----------------------
SI_INTERRUPT_PROTO(Usb_ISR, USB0_IRQn);

SI_SEGMENT_VARIABLE(In_Packet[IN_EP1_PACKET_SIZE], uint8_t, SI_SEG_XDATA);
SI_SEGMENT_VARIABLE(Out_Packet[OUT_EP1_PACKET_SIZE], uint8_t, SI_SEG_XDATA);

uint8_t In_Packet_Ready = 0;
uint8_t Out_Packet_Ready = 0;
uint8_t AsyncResetState = 0;

// State machine state
static uint8_t State = ST_IDLE;

// State variables for Read Page
static uint8_t TxBlock;    // Current block to send to host
static uint8_t TxPage;     // Current flash page to send to host
static uint8_t TxValid;    // Current flash page to send to host is valid

// State variables for Write Page
static uint8_t RxBlock;    // Current block to receive from host
static uint8_t RxPage;     // Current flash page to receive from host
static uint8_t RxValid;    // Current flash page to receive from host is valid
//------------------------------------------

uchar code par0[38] ={    0x00,0x00,0x02,0x00,0x00, //RB00.200---0,1,2,3,4
                          0x03,0x00,0x00,0x00,0x00, //RE30.000---5,6,7,8,9,
                0x01,0x00,0x00,0x00,0x00, //UP10.000----10,11,12,13,14
                          0x00,0x02,0x00,0x00,0x00, //LO02.000---15,16,17,18,19
                0x00,           //ADD--20,
                0x00,0x00,0x02,0x00,0x00, //AH ---21,22,23,24,25
                0x01,0x02,0x03, //上电识别代码 26,27,28
                              0x00,0x02,0x05,0x00,0x00, // AC ---29,30,31,32,33,
                0x01,                      // OPT---34  1:<AL ;2:AL--AH;3:>AH
                              0x00,                      //继电器的输出形式，=0： 距离输出；=1：出错输出----35
                0x00,                        //PNP的输出形式，=0： 距离输出；=1：出错输出------36
                0x01                          //0=rs232,1 =rs485
                         };
uchar idata par_buf[38];//从单片机内部的flash读出数据到XRAM

//flash
uchar xdata *pwrite;//外部数据缓冲区
uchar code *pread;

uchar *par;

sbit ALARM  = P2^0;
sbit m_clk  = P1^0;
sbit m_load = P1^2;
sbit m_din  =   P1^1;
sbit enter    = P1^7;
sbit shift_right= P1^6;
sbit upx      = P1^5;
sbit function = P1^4;
sbit Power = P1^3;
sbit OUT0 = P2^6;
sbit OUT1   =   P2^5;
//sbit OUT3 = P2^2;
sbit MODE         = P0^7;
sbit RXEN        = P0^3;
sbit DXEN        = P0^6;
//sbit ON           = P2^4;
//sbit SLEW         = P0^2;



ulint data UP;
ulint data LO;
ulint data AH;

uchar data CTR;
ulint data now_val;
uchar idata a,bx,c,d,e;



uchar idata receive0_temp[10]={' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};


uchar idata function_flag;

uchar idata up_flag;
uchar idata shift_right_flag=0;
uchar idata enter_flag;
bit   idata   find_key;                   //功能按键按下  两下确认


//----------------------------UART1-PROFIBUS----------------------------------------
uchar idata receive1[12]={' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
uchar idata send_buf[13]={' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};

volatile uchar data tl0timer0=0;//20ms的时间基准
bit b_head_sure= 0;
bit send_enable=0;
//uchar idata rcv1_end_flag = 0;
uchar idata rcv_count = 0;
bit uart1_receve_end = 0;
volatile uchar data txcount0=0;
volatile uchar data txptr0=0;//指针指向下一个地址

//---------------------------------------------------------------------------

void trans0(char *str);

void display(void);

void display_add(void);
void  shift0_5(void);
void  shift_add(void);
void disp_emissivity_5(void);
void disp_emissivity0_5(void);
void disp_emissivity_2(void);
void disp_emissivity0_2(void);
//void display_uart0(void);
void rb1(void);
void send_rb(void);
void send_rb3();
void send_rb0();
void send_dt(void);
void send_dt1(void);
//void dt(void);
void send_re(void);
void re1();
void send_re3();
void send_re0();

void send_ah(void);
void ah1(void);

void send_dl();
void dl1();
void pa(void);
//void pr(void);
void send_dh();
void dh1();

void send_opt();
void opt();
void po();
void send_po();
void rl();
void send_rl();
void alarm_state(void);
void direct_uart1();
void direct_uart0(void);
void send_ver();
//void send_error_code1();
//void send_error_code2();
//void display_dt();
void send_crc_buf(void);

//------------------------usb--------------------------------
//-----------------------------------------------------------------------------
// Static Function Prototypes - State Machine
//-----------------------------------------------------------------------------

static void StateIdle (void);
static void StateSetFlashKey (void);
static void StateTxPageInfo (void);
static void StateReadPage (void);
static void StateTxBlock (void);
static void StateWritePage (void);
static void StateRxBlock (void);
static void StateTxSuccess (void);
static void StateTxInvalid (void);
static void StateMachine (void);
//-----------------------------------------------------------
//-----------------------------------------------------------------------------
// SiLabs_Startup() Routine
// ----------------------------------------------------------------------------
// This function is called immediately after reset, before the initialization
// code is run in SILABS_STARTUP.A51 (which runs before main() ). This is a
// useful place to disable the watchdog timer, which is enable by default
// and may trigger before main() in some instances.
//-----------------------------------------------------------------------------
void SiLabs_Startup (void)
{
  PCA0MD &= ~0x40;    //clear WD Timer Enable at beginning of initialization code or it may fire.
}

/////////////////////////////////////
//  Generated Initialization File  //

void PCA_Init()                             //Programmable Counter Array
{
    PCA0MD    &= ~0x40;
    PCA0MD    = 0x00;
    PCA0CPL4  = 0xC2;
    PCA0MD    |= 0x20;
}

void UART0_Init()
{
    SCON0     = 0x10;

    TCON      = 0x50; //timer1 enable
    TMOD      = 0x21; //8bit counter/timer autoreload
    CKCON     = 0x01; //timer1 clock = sysclock
    TH1       = 0x64; //bandrate 9600bit             //根据定时器1高位
  TL1       = TH1;
  ES0       = 1;

  // IP |= 0x10;                         // Make UART high priority
}

void UART1_Init()
{

   SMOD1 = 0x0C;

   SCON1 = 0x10;                       // SCON1: 8-bit variable bit rate
                                       //        level of STOP bit is ignored
                                       //        RX enabled
                                       //        ninth bits are zeros
                                       //       clear RI0 and TI0 bits
//       SBRLH1    = 0xFF;
//       SBRLL1    = 0xCC;  // 115200

   SBRLH1    = 0xFD;
   SBRLL1    = 0x8F;  // 9600


    SBCON1 |= 0x43;                     // enable baud rate generator


  SCON1 |= 0x02;                      // indicate ready for TX

   EIE2 |= 0x02;  //enable UART1 interrupt
     EIP2 |= 0x02;   // Make UART high priority


}
void timer0_init(void)
{
    TCON      = 0x00;
    TMOD      = 0x01;
   CKCON=0x00;        //timer0使用sysclk/12
   ET0=1;         //ET0=1;
   TH0=(65536-20000)/256;//20ms定时   ?
   TL0=(65535-20000)%256;
   TR0=1;
 //  PT0=1;             //为默认优先级

}
void Port_IO_Init()
{
 // P0.0  -  TX1 (UART1), Push-Pull,  Digital
    // P0.1  -  RX1 (UART1), Open-Drain, Digital
    // P0.2  -  Unassigned,  Open-Drain, Digital
    // P0.3  -  Unassigned,  Open-Drain, Digital
    // P0.4  -  TX0 (UART0), Push-Pull,  Digital
    // P0.5  -  RX0 (UART0), Open-Drain, Digital
    // P0.6  -  Unassigned,  Open-Drain, Digital
    // P0.7  -  Unassigned,  Open-Drain, Digital

    // P1.0  -  Unassigned,  Open-Drain, Digital
    // P1.1  -  Unassigned,  Open-Drain, Digital
    // P1.2  -  Unassigned,  Open-Drain, Digital
    // P1.3  -  Unassigned,  Open-Drain, Digital
    // P1.4  -  Unassigned,  Open-Drain, Digital
    // P1.5  -  Unassigned,  Open-Drain, Digital
    // P1.6  -  Unassigned,  Open-Drain, Digital
    // P1.7  -  Unassigned,  Open-Drain, Digital

    // P2.0  -  Unassigned,  Open-Drain, Digital
    // P2.1  -  Unassigned,  Open-Drain, Digital
    // P2.2  -  Unassigned,  Open-Drain, Digital
    // P2.3  -  Unassigned,  Open-Drain, Digital
    // P2.4  -  Unassigned,  Open-Drain, Digital
    // P2.5  -  Unassigned,  Open-Drain, Digital
    // P2.6  -  Unassigned,  Open-Drain, Digital
    // P2.7  -  Unassigned,  Open-Drain, Digital

    // P3.0  -  Unassigned,  Open-Drain, Digital

    P0MDOUT   = 0x11;    //TX0 TX1 push-pull
    XBR0      = 0x01;    //UART0 ENABLE
    XBR1      = 0x40;    //Crossbar Enable
    XBR2      = 0x01;    //UART1 ENABLE


}

void Oscillator_Init()
{
    int i = 0;          //去掉
    OSCICN    = 0xC3;
}
// Initialization function for device,
// Call Init_Device() from your main program
void Init_Device(void)
{
    PCA_Init();              //Programmable Counter Array
    timer0_init();
    UART0_Init();
  UART1_Init();
    Port_IO_Init();
    Oscillator_Init();
}

void trans0(char *str)
{
  uchar *p;
  p = str;
  while(*p!='\0')
  {
    PCA0CPH4 = 0xa0;
    SBUF0 = *p++;
    while(!TI0);
    TI0 = 0;
  }
}

/////////////////////////////////////////////////////
/*                      DELAY                      */
/////////////////////////////////////////////////////

void delay(int  n)
{
  int i;
  int j;

  for (i=0;i<n;i++)
  {

  for(j=0;j<1;j++)
  {PCA0CPH4 = 0xa0;}
  }
}



void delay20ms(void)
{
    uchar j;
    for(j=0;j<100;j++)
    {
      PCA0CPH4 = 0xa0;
        delay(40);

    }
}

void delay1s(void)
{
  uchar l;
  for(l=0;l<10;l++)
  {
    PCA0CPH4 = 0xa0;
    delay20ms();
  }
}
/////////////////////////////////////////////////////
/*                    m_display                      */
/////////////////////////////////////////////////////
static  struct
{
  uchar ascii;
  uchar stroke;
}code led_stroke[]=
  {
    {0,0x7e},{1,0x30},{2,0x6d},{3,0x79},{4,0x33},
    {5,0x5b},{6,0x5f},{7,0x70},{8,0x7f},{9,0x7b},
    {'0',0x7e},{'1',0x30},{'2',0x6d},{'3',0x79},{'4',0x33},
    {'5',0x5b},{'6',0x5f},{'7',0x70},{'8',0x7f},{'9',0x7b},
    {'A',0x77},{'B',0x1f},{'C',0x4e},{'D',0x3d},{'E',0x4f},
    {'F',0x47},{'H',0x37},{'I',0X06},{'L',0x0e},{'M',0x15},
    {'O',0x1d},{'P',0x67},{'R',0x05},{'S',0x5b},{'T',0x46},
    {'Y',0x3b},{'U',0x3E},{'-',0x01},{' ',0x00},{'\n',0x00},{'\r',0x00}
  };



void  m_send(uchar addr,uchar da)
{
  uchar i,byte_out;
  byte_out=addr;
  m_clk=0;
  m_load=0;
  for(i=0;i<8;i++)                              //address
    {
      m_din=byte_out&0x80;
        m_clk = 0;
          PCA0CPH4 = 0xa0;    //160
      m_clk = 1;
      byte_out=byte_out<<1;
    }
  byte_out=da;
  for(i=0;i<8;i++)                             //data
    {
      m_din=byte_out&0x80;
      m_clk=0;

        PCA0CPH4 = 0xa0;
      m_clk = 1;
      byte_out=byte_out<<1;
    }

  m_load=1;

}

void  m_init(void)                      //led显示
{
  m_send(0x09,0x00);//no dcode
  m_send(0x0a,0x02);//light
  m_send(0x0b,0x04);//number 5
  m_send(0x0c,0x01);//start
  m_send(0x0f,0x00);

}

uchar get_stroke(uchar c)
{
  uchar i=0;
  while(led_stroke[i].ascii!=c)
  {
    i++;
      PCA0CPH4 = 0xa0;
    if(i>40)
    {
      return(' ');
    }
  }
  return(led_stroke[i].stroke);
}


/////////////////////////////////////////////////////
/*               C8051F38X flash                   */
/////////////////////////////////////////////////////
/*
PSCTL:
位0 PSWE - 置位，允许写FLASH；
位1 PSEE - 置位，允许擦除FLASH；
位2 SFLE - 置位，允许用户软件访问FLASH的128B的临时存储器扇区
其他位置0

FLSCL：
位0 FLWE - 置位，允许写FLASH；
位1-5 保留，0000；
位6 FRAE - 置位，闪存总是处于读方式
位7 FOSE - 置位，允许闪存单稳态定时器
*/

void erase_par(void)  //首次上电初始化,将外部数据初始化为0xff
{
  EA=0;
    FLKEY=0xA5;
  FLKEY=0xF1;
  PSCTL=0x03; //  bit1 PSEE =1 : permit erase; bit0 PSWE = 1 :permit write
  pwrite=0x7E00;
  *pwrite=0x00;
    PSCTL=0x00;   // 复位，用户软件访问FLASH的64KB的程序/数据FLASH扇区，很重要！！！

}

void read_par(void)
{
  uchar data i;

  FLSCL=0xcf;       //禁止flash写
  PSCTL=0x00;
  pread=0x7E00;     //初始化code读指针为字符串起始变量
  for(i=0;i<38;i++)
  {
    par_buf[i]=*pread++;    //将FLASH内数据读到XRAM
   PCA0CPH4 = 0xa0;
  }

}
void init_par(void)
{

  uchar data i;
   PFE0CN =0x00;        // select single-byte write mode.
  PSCTL=0x01;           //允许写入flash
  par=&par0[0];
  pwrite=0x7E00;
  for (i=0;i<38;i++)
  {
         FLKEY=0xA5;            //Flash Lock and Key 顺序要对
       FLKEY=0xF1;
      *pwrite=*par;
      pwrite++;
      par++;
      PCA0CPH4 = 0xa0;              //feed dog
    }
    PSCTL=0x00;      //禁止写入flash

    read_par();

}



void write_par(void)
{
  uchar data i;
   PFE0CN =0x00;
  PSCTL=0x01;
  pwrite=0x7E00;
  for (i=0;i<38;i++)
  {
      FLKEY=0xA5;
       FLKEY=0xF1;
    *pwrite++=par_buf[i];
    PCA0CPH4 = 0xa0;
  }
    PSCTL=0x00;
    read_par();

}

void timer0_isr(void) interrupt 1 //using 1

{
   uchar data key_value_temp;

  if(tl0timer0<5)
  {tl0timer0++;}//与PC通信空闲的时间*/

   key_value_temp=(~P1)&0xf0; // 0x00101110  P1^4:SET; P1^5:ADD; P1^6:REDUCE; P1^7:OK     //REDUCE or shift？

  if (key_value_temp!=0)
   {
       delay(10);

    if (key_value_temp!=0)
    {
     delay(10);

      switch(key_value_temp)
    {
//        key_value_temp= (~P1)&0xf0;   // this isn't even executed (but even if it was, it wouldn't retroactively change the result of switch statement above?
      case 0x10:  function_flag++;
                     find_key=1;
                  if (function_flag==12)
              {
                function_flag=1;
              }
            do{ PCA0CPH4 = 0xa0;}
              while (function==0);
              delay(10);

            break;
      case 0x40:  shift_right_flag++;

            if (shift_right_flag==6)
            {
              shift_right_flag=1;
                  }
              do{ PCA0CPH4 = 0xa0;}
            while (shift_right==0);
            delay(10);;

            break;
      case 0x20:  up_flag++;

            if (up_flag==10)
            {
              up_flag=0;
                }
              do{PCA0CPH4 = 0xa0; }
            while (upx==0);
            delay(10);;

            break;
      case  0x80: if(function_flag!=0)
                   {enter_flag=1;}
              do{PCA0CPH4 = 0xa0; }
              while (enter==0);
              delay(10);;

            break;
      default:  break;
      }
   }
  }
    TH0=(65535-20000)/256;    //20ms定时重载65536-18432
    TL0=(65535-20000)%256;



}

void key_flag_init(void)
{
  function_flag=0;
  up_flag=0;
  shift_right_flag=0;
  enter_flag=0;
}
void scan_keyboard(void)
{
 // if(function_flag==0) return ;

  switch(function_flag)
  {
    case 1:

                m_init();
          m_send(0x01,get_stroke('1'));
          m_send(0x02,get_stroke('-'));
          m_send(0x03,get_stroke('-'));
          m_send(0x04,get_stroke('R'));
          m_send(0x05,get_stroke('B'));

            find_key=1;

              a=par_buf[0];
                 bx=par_buf[1];
                 c=par_buf[2];
               d=par_buf[3];
               e=par_buf[4];
        if  (enter_flag==1)
        {
          shift_right_flag=1;
                  up_flag=0;
                  enter_flag=0;
          display();
           do
                      {
                    PCA0CPH4 = 0xa0;
                        shift0_5(); //显示5位

                     }
                     while  (enter_flag==0);
                    {

                      key_flag_init();

                   display();
                      par_buf[0]=a;
                      par_buf[1]=bx;
                      par_buf[2]=c;
                    par_buf[3]=d;
                    par_buf[4]=e;
               EA=0;
                     erase_par();
                   write_par();     //将修改好后的内容写入flash
                    EA=1;

                      SBUF0 = 'R';  while(!TI0);TI0 = 0;
               SBUF0 = 'B';  while(!TI0);TI0 = 0;
                SBUF0 = par_buf[0]+0x30;while(!TI0);TI0 = 0;
                SBUF0 = par_buf[1]+0x30;while(!TI0);TI0 = 0;
                SBUF0 = '.';            while(!TI0);TI0 = 0;
                SBUF0 = par_buf[2]+0x30;while(!TI0);TI0 = 0;
                SBUF0 = par_buf[3]+0x30;while(!TI0);TI0 = 0;
                SBUF0 = par_buf[4]+0x30;while(!TI0);TI0 = 0;
                SBUF0 = 0x0d;           while(!TI0);TI0 = 0;
                SBUF0 = 0x0a;           while(!TI0);TI0 = 0;
                delay1s();
                delay1s();

                trans0("DT\r\n");
                delay1s();
                delay1s();

                find_key = 0;
                delay1s();



                  }


        }
        break;
    case 2:

                m_init();
          m_send(0x01,get_stroke('2'));
          m_send(0x02,get_stroke('-'));
          m_send(0x03,get_stroke('-'));
          m_send(0x04,get_stroke('R'));
          m_send(0x05,get_stroke('E'));

           find_key=1;
               a=par_buf[5];
                 bx=par_buf[6];
                 c=par_buf[7];
               d=par_buf[8];
               e=par_buf[9];

        if  (enter_flag==1)
        {
          shift_right_flag=1;
                  up_flag=0;
                  enter_flag=0;
          display();
           do
                      {
                    PCA0CPH4 = 0xa0;
                        shift0_5(); //显示5位

                     }
                     while  (enter_flag==0);
                    {

                      key_flag_init();

                   display();
                     par_buf[5]=a;
                      par_buf[6]=bx;
                      par_buf[7]=c;
                    par_buf[8]=d;
                    par_buf[9]=e;

                       EA=0;
                     erase_par();
                   write_par();     //将修改好后的内容写入flash
                      EA=1;
            SBUF0 = 'R';           while(!TI0);TI0 = 0;
                SBUF0 = 'E';             while(!TI0);TI0 = 0;
                SBUF0 = par_buf[5]+0x30; while(!TI0);TI0 = 0;
                SBUF0 = par_buf[6]+0x30; while(!TI0);TI0 = 0;
                SBUF0 = '.';             while(!TI0);TI0 = 0;
                SBUF0 = par_buf[7]+0x30; while(!TI0);TI0 = 0;
                SBUF0 = par_buf[8]+0x30; while(!TI0);TI0 = 0;
                SBUF0 = par_buf[9]+0x30; while(!TI0);TI0 = 0;
                SBUF0 = 0x0d;            while(!TI0);TI0 = 0;
                SBUF0 = 0x0a;            while(!TI0);TI0 = 0;
                delay1s();
                delay1s();
                trans0("DT\r\n");
                  delay1s();
                delay1s();

                  find_key = 0;
                  delay1s();


                  }

        }
        break;
           case 3:     //输出控制方式（单点或范围）

          m_init();
          m_send(0x01,get_stroke('3'));
          m_send(0x02,get_stroke('-'));
          m_send(0x03,get_stroke('O'));
          m_send(0x04,get_stroke('P'));
          m_send(0x05,get_stroke('T'));
            find_key=1;

        if  (enter_flag==1)
        {
           a=par_buf[34];
                 up_flag = a;
                  enter_flag=0;

                 do
           {
             PCA0CPH4 = 0xa0;
                  if (up_flag==1)
                      {
                        a=1;
                      }
                 else if (up_flag==2)
                  {
                    a=2;
                    }
            else if (up_flag==3)
                  {
                    a=3;
                    }
                      else if (up_flag==4)
                  {
                    a=1;up_flag=1;
                    }


                     m_init();
                     m_send(0x01,get_stroke(a));
                     m_send(0x02,get_stroke(' '));
                     m_send(0x03,get_stroke(' '));
                     m_send(0x04,get_stroke(' '));
                     m_send(0x05,get_stroke(' '));
                     }
          while(enter_flag==0);
          {


                     key_flag_init();

             par_buf[34]=a;

              EA=0;
             erase_par();
                   write_par();     //将修改好后的内容写入flash

            EA=1;
                  find_key = 0;
                        delay1s();//等待UART0接收数据，否则会显示乱码

                  }

        }
        break;

        case 4:


              m_init();
          m_send(0x01,get_stroke('4'));
          m_send(0x02,get_stroke('-'));
          m_send(0x03,get_stroke('-'));
          m_send(0x04,get_stroke('D'));
          m_send(0x05,get_stroke('L'));
         find_key=1;
               a=par_buf[15];
                 bx=par_buf[16];
                 c=par_buf[17];
               d=par_buf[18];
               e=par_buf[19];
        if  (enter_flag==1)
        {
          shift_right_flag=1;
                  up_flag=0;
                  enter_flag=0;
          display();
           do
                      {
                   PCA0CPH4 = 0xa0;
                        shift0_5(); //显示5位

                     }
                     while  (enter_flag==0);
                    {

                     key_flag_init();

                   display();
                      par_buf[15]=a;
                      par_buf[16]=bx;
                      par_buf[17]=c;
                    par_buf[18]=d;
                    par_buf[19]=e;

               EA=0;
                     erase_par();
                   write_par();     //将修改好后的内容写入flash
            EA=1;
                  find_key = 0;
                      delay1s();
                  }

        }
        break;

    case 5:

          m_init();
          m_send(0x01,get_stroke('5'));
          m_send(0x02,get_stroke('-'));
          m_send(0x03,get_stroke('-'));
          m_send(0x04,get_stroke('D'));
          m_send(0x05,get_stroke('H'));
          find_key=1;
               a=par_buf[10];
                 bx=par_buf[11];
                 c=par_buf[12];
               d=par_buf[13];
               e=par_buf[14];
        if  (enter_flag==1)
        {
          shift_right_flag=1;
                  up_flag=0;
                  enter_flag=0;
          display();
           do
                      {
                   PCA0CPH4 = 0xa0;
                        shift0_5(); //显示5位

                     }
                     while  (enter_flag==0);
                    {

                      key_flag_init();

                   display();
                      par_buf[10]=a;
                      par_buf[11]=bx;
                      par_buf[12]=c;
                    par_buf[13]=d;
                    par_buf[14]=e;

               EA=0;
                     erase_par();
                   write_par();     //将修改好后的内容写入flash


              EA=1;
                       find_key = 0;
                   delay1s();
                  }

        }
        break;


           case 6:

          m_init();
          m_send(0x01,get_stroke('6'));
          m_send(0x02,get_stroke('-'));
          m_send(0x03,get_stroke('-'));
          m_send(0x04,get_stroke('A'));
          m_send(0x05,get_stroke('H'));
              find_key = 1;

               a=par_buf[21];
                 bx=par_buf[22];
                 c=par_buf[23];
               d=par_buf[24];
               e=par_buf[25];

        if  (enter_flag==1)
        {
          shift_right_flag=1;
                  up_flag=0;
                  enter_flag=0;
          display();
           do
                      {
                    PCA0CPH4 = 0xa0;
                        shift0_5(); //显示5位

                     }
                     while  (enter_flag==0);
                    {

                      key_flag_init();

                   display();
                      par_buf[21]=a;
                      par_buf[22]=bx;
                      par_buf[23]=c;
                    par_buf[24]=d;
                    par_buf[25]=e;

              EA=0;
                     erase_par();
                   write_par();     //将修改好后的内容写入flash
              EA=1;
                         find_key = 0;
                           delay1s();


                  }

        }
        break;
       case 7:    //继电器输出选择

          m_init();
          m_send(0x01,get_stroke('7'));
          m_send(0x02,get_stroke('-'));
          m_send(0x03,get_stroke('R'));
          m_send(0x04,get_stroke('L'));
          m_send(0x05,get_stroke('Y'));

               find_key=1;
        if  (enter_flag==1)
        {
           a=par_buf[35];
                 up_flag = a;
                  enter_flag=0;

                 do
           {
             PCA0CPH4 = 0xa0;
                  if (up_flag==0)
                      {
                        a=0;
                      }
                 else if (up_flag==1)
                  {
                    a=1;
                    }

                      else if (up_flag==2)
                  {
                    a=0;up_flag=0;
                    }


                      m_init();
          m_send(0x01,get_stroke(a));
          m_send(0x02,get_stroke(' '));
          m_send(0x03,get_stroke(' '));
          m_send(0x04,get_stroke(' '));
          m_send(0x05,get_stroke(' '));
                     }
          while(enter_flag==0);
          {
             par_buf[35]=a;

                     key_flag_init();

             EA=0;
             erase_par();
                   write_par();     //将修改好后的内容写入flash
                      EA=1;
              find_key = 0;
                        delay1s();



                  }

        }
        break;
        case 8:    //PNP输出选择

          m_init();
          m_send(0x01,get_stroke('8'));
          m_send(0x02,get_stroke('-'));
          m_send(0x03,get_stroke('-'));
          m_send(0x04,get_stroke('P'));
          m_send(0x05,get_stroke('0'));
          find_key=1;


        if  (enter_flag==1)
        {
           a=par_buf[36];
                 up_flag = a;
                  enter_flag=0;

                 do
           {
             PCA0CPH4 = 0xa0;
                  if (up_flag==0)
                      {
                        a=0;
                      }
                 else if (up_flag==1)
                  {
                    a=1;
                    }

                      else if (up_flag==2)
                  {
                    a=0;up_flag=0;
                    }


                      m_init();
          m_send(0x01,get_stroke(a));
          m_send(0x02,get_stroke(' '));
          m_send(0x03,get_stroke(' '));
          m_send(0x04,get_stroke(' '));
          m_send(0x05,get_stroke(' '));
                     }
          while(enter_flag==0);
          {
             par_buf[36]=a;

                     key_flag_init();

             EA=0;
             erase_par();
                   write_par();     //将修改好后的内容写入flash
              EA=1;
            find_key = 0;
                        delay1s();


                  }

        }
        break;
        case 9:    //RS232/RS485选择

          m_init();
          m_send(0x01,get_stroke('9'));
          m_send(0x02,get_stroke('-'));
          m_send(0x03,get_stroke('-'));
          m_send(0x04,get_stroke('C'));
          m_send(0x05,get_stroke('P'));
            find_key=1;

        if  (enter_flag==1)
        {
           a=par_buf[37];
                 up_flag = a;
                  enter_flag=0;

                 do
           {
             PCA0CPH4 = 0xa0;
                  if (up_flag==0)
                      {
                        a=0;
                      }
                 else if (up_flag==1)
                  {
                    a=1;
                    }

                      else if (up_flag==2)
                  {
                    a=0;up_flag=0;
                    }


                      m_init();
          m_send(0x01,get_stroke(a));
          m_send(0x02,get_stroke(' '));
          m_send(0x03,get_stroke(' '));
          m_send(0x04,get_stroke(' '));
          m_send(0x05,get_stroke(' '));
                     }
          while(enter_flag==0);
          {
             par_buf[37]=a;

                     key_flag_init();

              EA=0;
             erase_par();
                   write_par();     //将修改好后的内容写入flash
            EA=1;
                find_key = 0;
                        delay1s();

                  }

        }
        break;
        case 10:    //ADD

          m_init();
          m_send(0x01,get_stroke('1'));
          m_send(0x02,get_stroke('0'));
          m_send(0x03,get_stroke('-'));
          m_send(0x04,get_stroke('-'));
          m_send(0x05,get_stroke('A'));

          find_key=1;

        if  (enter_flag==1)
        {
           a=par_buf[20];
           bx=par_buf[20]/10;
           c=par_buf[20]%10;
                  shift_right_flag=1;
                  up_flag=0;
                  enter_flag=0;
          display_add();
           do
                      {
                    PCA0CPH4 = 0xa0;
                        shift_add();  //显示2位

                     }
                     while  (enter_flag==0);
                    {

                      key_flag_init();

                   display_add();
                     a=bx*10+c;

                     if (a<=25)//A-Z(65-90)
                   {
                  par_buf[20]=a;
             EA=0;
                     erase_par();
                   write_par();     //将修改好后的内容写入flash
                      EA=1;
                     }

                        find_key = 0;
              delay1s();



                  }

        }
        break;
         case 11:

          m_init();
          m_send(0x01,get_stroke('1'));
          m_send(0x02,get_stroke('1'));
          m_send(0x03,get_stroke('-'));
          m_send(0x04,get_stroke('U'));
          m_send(0x05,get_stroke('R'));

         find_key=1;

        if  (enter_flag==1)
        {

                 enter_flag=0;

                 do
           {
                      m_init();
          m_send(0x01,get_stroke(' '));
          m_send(0x02,(get_stroke(version_major-0x30)|0x80));  //Display Version Number
          m_send(0x03,get_stroke(version_minor-0x30));
          m_send(0x04,get_stroke(' '));
          m_send(0x05,get_stroke(' '));
                     }
          while(enter_flag==0);
          {
                     key_flag_init();

            find_key = 0;
             delay1s();




                  }

        }
        break;


    default: break;
  }
}

void display(void)
{
 m_init();
m_send(0x01,get_stroke(a));
m_send(0x02,(get_stroke(bx)|0x80));
m_send(0x03,get_stroke(c));
m_send(0x04,get_stroke(d));
m_send(0x05,get_stroke(e));
}

void display_add(void)
{
 m_init();
m_send(0x01,get_stroke(bx));
m_send(0x02,get_stroke(c));
m_send(0x03,get_stroke(' '));
m_send(0x04,get_stroke(' '));
m_send(0x05,get_stroke(' '));
}
void  shift_add(void)
{
    if (shift_right_flag==1)      //第一位闪烁
    {

      up_flag=bx;
    do
    { PCA0CPH4 = 0xa0;
      switch(up_flag)
      {
          case 0: bx=0; break;
        case 1: bx=1; break;
        case 2: bx=2; break;
        case 3: bx=3; break;
        case 4: bx=4; break;
        case 5: bx=5; break;
        case 6: bx=6; break;
        case 7: bx=7; break;
        case 8: bx=8; break;
        case 9: bx=9; break;
        case 10: bx=0; break;
            }
      disp_emissivity_2();
         }
      while ( (shift_right_flag==1)&&(enter_flag==0) );
    }
    else if (shift_right_flag==2)
    {
       up_flag=c;
       do
     {
      PCA0CPH4 = 0xa0;
      switch(up_flag)
      {
          case 0: c=0; break;
        case 1: c=1; break;
        case 2: c=2; break;
        case 3: c=3; break;
        case 4: c=4; break;
        case 5: c=5; break;
        case 6: c=6; break;
        case 7: c=7; break;
        case 8: c=8; break;
        case 9: c=9; break;
        case 10: c=0; break;
            }
      disp_emissivity_2();
         }
      while ( (shift_right_flag==2)&&(enter_flag==0) );
   }

   else if (shift_right_flag==3)
   {
    shift_right_flag=1;
   }
}
void disp_emissivity_2(void)
{
   if (shift_right_flag==1)     //第一位闪烁显示
   {
       disp_emissivity0_2();
      // delay(100);
     delay1s();

    m_init();
    m_send(0x01,get_stroke(' '));
    m_send(0x02,get_stroke(c));
    m_send(0x03,get_stroke(' '));
    m_send(0x04,get_stroke(' '));
    m_send(0x05,get_stroke(' '));
    // delay(100);
     delay1s();

      }
  else if (shift_right_flag==2)       //第二位闪烁显示
  {
       disp_emissivity0_2();
    // delay(100);
     delay1s();

    m_init();
    m_send(0x01,get_stroke(bx));
    m_send(0x02,get_stroke(' '));
    m_send(0x03,get_stroke(' '));
    m_send(0x04,get_stroke(' '));
    m_send(0x05,get_stroke(' '));
  //   delay(100);
    delay1s();

      }

    else if (shift_right_flag==3)
    {
             shift_right_flag=1;
       }
}


void disp_emissivity0_2(void)     //正常显示，不闪
{
    m_init();
  m_send(0x01,get_stroke(bx));
  m_send(0x02,get_stroke(c));
  m_send(0x03,get_stroke(' '));
  m_send(0x04,get_stroke(' '));
  m_send(0x05,get_stroke(' '));
}
void  shift0_5(void)
{
    if (shift_right_flag==1)      //第一位闪烁
    {
       up_flag=a;
       do
     {
        PCA0CPH4 = 0xa0;
      switch(up_flag)
      {
          case 0: a=0; break;
        case 1: a=1; break;
        case 2: a=2; break;
        case 3: a=3; break;
        case 4: a=4; break;
        case 5: a=5; break;
        case 6: a=6; break;
        case 7: a=7; break;
        case 8: a=8; break;
        case 9: a=9; break;
        case 10: a=0; break;
            }
      disp_emissivity_5();
      }
          while ( (shift_right_flag==1)&&(enter_flag==0) );
    }
    else if (shift_right_flag==2)     //第二位闪烁
    {
      up_flag=bx;
    do
    { PCA0CPH4 = 0xa0;
      switch(up_flag)
      {
          case 0: bx=0; break;
        case 1: bx=1; break;
        case 2: bx=2; break;
        case 3: bx=3; break;
        case 4: bx=4; break;
        case 5: bx=5; break;
        case 6: bx=6; break;
        case 7: bx=7; break;
        case 8: bx=8; break;
        case 9: bx=9; break;
        case 10: bx=0; break;
            }
      disp_emissivity_5();
         }
      while ( (shift_right_flag==2)&&(enter_flag==0) );
    }
    else if (shift_right_flag==3)
    {
       up_flag=c;
       do
     {
      PCA0CPH4 = 0xa0;
      switch(up_flag)
      {
          case 0: c=0; break;
        case 1: c=1; break;
        case 2: c=2; break;
        case 3: c=3; break;
        case 4: c=4; break;
        case 5: c=5; break;
        case 6: c=6; break;
        case 7: c=7; break;
        case 8: c=8; break;
        case 9: c=9; break;
        case 10: c=0; break;
            }
      disp_emissivity_5();
         }
      while ( (shift_right_flag==3)&&(enter_flag==0) );
   }
  else if (shift_right_flag==4)
  {
       up_flag=d;
       do
     {
      PCA0CPH4 = 0xa0;
      switch(up_flag)
      {
          case 0: d=0; break;
        case 1: d=1; break;
        case 2: d=2; break;
        case 3: d=3; break;
        case 4: d=4; break;
        case 5: d=5; break;
        case 6: d=6; break;
        case 7: d=7; break;
        case 8: d=8; break;
        case 9: d=9; break;
        case 10: d=0; break;
            }
      disp_emissivity_5();
         }
      while ( (shift_right_flag==4)&&(enter_flag==0) );
   }
    else if (shift_right_flag==5)
  {
       up_flag=e;
       do
     {
        PCA0CPH4 = 0xa0;
      switch(up_flag)
      {
          case 0: e=0; break;
        case 1: e=1; break;
        case 2: e=2; break;
        case 3: e=3; break;
        case 4: e=4; break;
        case 5: e=5; break;
        case 6: e=6; break;
        case 7: e=7; break;
        case 8: e=8; break;
        case 9: e=9; break;
        case 10: e=0; break;
            }
      disp_emissivity_5();
         }
      while ( (shift_right_flag==5)&&(enter_flag==0) );
   }
   else if (shift_right_flag==6)
   {
    shift_right_flag=1;
   }
}


void disp_emissivity_5(void)
{
   if (shift_right_flag==1)     //第一位闪烁显示
   {
       disp_emissivity0_5();
      // delay(100);
     delay1s();

    m_init();
    m_send(0x01,get_stroke(' '));
    m_send(0x02,(get_stroke(bx)|0x80));
    m_send(0x03,get_stroke(c));
    m_send(0x04,get_stroke(d));
    m_send(0x05,get_stroke(e));
    // delay(100);
     delay1s();

      }
  else if (shift_right_flag==2)       //第二位闪烁显示
  {
       disp_emissivity0_5();
    // delay(100);
     delay1s();

    m_init();
    m_send(0x01,get_stroke(a));
    m_send(0x02,(get_stroke(' ')|0x80));
    m_send(0x03,get_stroke(c));
    m_send(0x04,get_stroke(d));
    m_send(0x05,get_stroke(e));
  //   delay(100);
    delay1s();

      }
    else if (shift_right_flag==3)     //第三位闪烁显示
    {
         disp_emissivity0_5();
     //  delay(100);
       delay1s();
       m_init();

    m_send(0x01,get_stroke(a));
    m_send(0x02,(get_stroke(bx)|0x80));
    m_send(0x03,get_stroke(' '));
    m_send(0x04,get_stroke(d));
    m_send(0x05,get_stroke(e));
     //  delay(100);
       delay1s();

  }
  else if (shift_right_flag==4)     //第4位闪烁显示
   {
         disp_emissivity0_5();
      // delay(100);
      delay1s();

       m_init();
    m_send(0x01,get_stroke(a));
    m_send(0x02,(get_stroke(bx)|0x80));
    m_send(0x03,get_stroke(c));
    m_send(0x04,get_stroke(' '));
    m_send(0x05,get_stroke(e));
     //  delay(100);
       delay1s();

  }
  else if (shift_right_flag==5)
  {
        disp_emissivity0_5();
    //   delay(100);
      delay1s();

       m_init();
    m_send(0x01,get_stroke(a));
    m_send(0x02,(get_stroke(bx)|0x80));
    m_send(0x03,get_stroke(c));
    m_send(0x04,get_stroke(d));
    m_send(0x05,get_stroke(' '));
     //  delay(100);
      delay1s();

  }
    else if (shift_right_flag==6)
    {
             shift_right_flag=1;
       }
}


void disp_emissivity0_5(void)     //正常显示，不闪
{
    m_init();
  m_send(0x01,get_stroke(a));
  m_send(0x02,(get_stroke(bx)|0x80));
  m_send(0x03,get_stroke(c));
  m_send(0x04,get_stroke(d));
  m_send(0x05,get_stroke(e));
}

char idata receive0[10]={' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};

bit nors=0;
bit err_flag=0;
bit uart0_rcv_ok= 0;
void uart0(void) interrupt 4
{
  uchar  i;


  if(RI0==1)
    {
    for(i=0;i<10;i++)
    {
      while(!RI0)
      { PCA0CPH4 = 0xa0; }

      receive0[i] = SBUF0;
      RI0 = 0;

      if(receive0[i]==0x0a)
      {
         uart0_rcv_ok =1;
      //  ES0 = 0;
        break;
      }
        }
 }
}

void direct_uart0(void)
{
    uchar  i, rd1,rd2,rd3,rd4,rd5;
   if(find_key!=0)return;
   if(uart0_rcv_ok==0)return;
    uart0_rcv_ok=0;



    if((receive0[3]=='.')&&(receive0[0]>=0x30)&&(receive0[0]<=0x39)&&(receive0[1]>=0x30)&&(receive0[1]<=0x39)&&(receive0[2]>=0x30)&&(receive0[2]<=0x39)&&(receive0[4]>=0x30)&&(receive0[4]<=0x39)&&(receive0[5]>=0x30)&&(receive0[5]<=0x39)&&(receive0[6]>=0x30)&&(receive0[6]<=0x39) )
    {
      err_flag=0;


      receive0[0]=receive0[0]-0x30;
      receive0[1]=receive0[1]-0x30;
      receive0[2]=receive0[2]-0x30;
      receive0[4]=receive0[4]-0x30;
      receive0[5]=receive0[5]-0x30;
      receive0[6]=receive0[6]-0x30;


            rd1 = receive0[1];
            rd2 = receive0[2];
            rd3 = receive0[4];
            rd4 = receive0[5];
            rd5 = receive0[6];

//            now_val = rd1*10000+rd2*1000+rd3*100+rd4*10+rd5;  // original code-- intermediate values are not unsigned long int!
            now_val = rd1;
            now_val = now_val*10 + rd2;
            now_val = now_val*10 + rd3;
            now_val = now_val*10 + rd4;
            now_val = now_val*10 + rd5;

            rd1 = rd1+0x30;
            rd2 = rd2+0x30;
            rd3 = rd3+0x30;
            rd4 = rd4+0x30;
            rd5 = rd5+0x30;

             m_init();
                 m_send(0x01,get_stroke(rd1));
                 m_send(0x02,(get_stroke(rd2))|0x80);
                 m_send(0x03,get_stroke(rd3));
                 m_send(0x04,get_stroke(rd4));
                 m_send(0x05,get_stroke(rd5));

    }

         else
      {

          for(i=0;i<10;i++)
          {
           receive0_temp[i]=receive0[i];
         }
         if(receive0_temp[0]=='+')
        {

        m_init();
        m_send(0x01,get_stroke(receive0_temp[1]));
        m_send(0x02,get_stroke(receive0_temp[2]));
        m_send(0x03,0x63);
        m_send(0x04,0x00);
        m_send(0x05,0x00);


        }
      else if(receive0_temp[0]=='E')
      {
         err_flag=1;
        m_init();
        m_send(0x01,get_stroke(receive0_temp[0]));
        m_send(0x02,get_stroke(receive0_temp[1]));
        m_send(0x03,get_stroke(receive0_temp[2]));
        m_send(0x04,get_stroke(receive0_temp[3]));
        m_send(0x05,get_stroke(receive0_temp[4]));
         }
         }



    alarm_state();


}




void alarm_state(void)

{

  ulint idata alarm_over_vhh,alarm_over_vll;
  ulint idata alarm_low_vhh,alarm_low_vll;
  ulint idata upper_val,lower_val;

  UP=(par_buf[10]*1000+par_buf[11]*100+par_buf[12]*10+par_buf[13])*10+par_buf[14];
  LO=(par_buf[15]*1000+par_buf[16]*100+par_buf[17]*10+par_buf[18])*10+par_buf[19];
  AH = (par_buf[21]*1000+par_buf[22]*100+par_buf[23]*10+par_buf[24])*10+par_buf[25];

  CTR = par_buf[34];



//继电器作为出错时报警
   if(par_buf[35]==1)
      {
      if(err_flag==1)
        {

       ALARM  = 0;
       delay1s();
       ALARM  = 1;
         delay1s();
         OUT0=1;
       }
     else
     {
       OUT0=0;
         }
    }


//继电器作为距离报警
 else if(par_buf[35]==0)
  {
  if(err_flag==1)
        {

       ALARM  = 0;
       delay1s();
       ALARM  = 1;
         delay1s();
        OUT0 =1;
       }
   else if (err_flag==0)
   {
    if(CTR ==1)
   {
     upper_val = LO+(AH/2);
     lower_val = LO-(AH/2);

   if(now_val>=upper_val)
   {
    OUT0 = 1;
   // OUT1 = 1;
   ALARM  = 1;
   }

   else if(now_val<=lower_val)
    {
      OUT0 = 0;
    //  OUT1 = 0;
      ALARM = 0;
  //   OUT3=!OUT3;
    }

  }


  else if(CTR ==2)
  {
  alarm_over_vhh = UP+AH/2;
  alarm_over_vll = UP-AH/2;
  alarm_low_vhh  = LO+AH/2;
  alarm_low_vll  = LO-AH/2;

     if((now_val>=alarm_low_vhh)&&(now_val<=alarm_over_vll))
     {
      ALARM = 0;
    OUT0 = 0;
   // OUT1 = 0;
    }
     else if((now_val<alarm_low_vll)||(now_val>alarm_over_vhh))
     {
      ALARM = 1;
    OUT0 = 1;
  //  OUT1 = 1;
    }

  }
else if (CTR ==3)
{
   upper_val = UP+(AH/2);
   lower_val = UP-(AH/2);


   if(now_val>=upper_val)
   {
    OUT0 = 0;
   // OUT1 = 0;
   ALARM  = 0;
   }

   else if(now_val<=lower_val)
    {
      OUT0 = 1;
    //  OUT1 = 1;
      ALARM = 1;

    }
  }
  }
}

 //PNP作为出错时报警
 if(par_buf[36]==1)
  {

   if(err_flag==1)
        {

       ALARM  = 0;
       delay1s();
       ALARM  = 1;
         delay1s();
         OUT1=0;
       }
     else
     {
       OUT1=1;
         }
    }



//PNP作为距离报警
else if(par_buf[36]==0)
{
 if(err_flag==1)
        {

       ALARM  = 0;
       delay1s();
       ALARM  = 1;
         delay1s();
          OUT1 =0;
       }

 else if (err_flag==0)
 {
  if(CTR ==1)
 {
   upper_val = LO+(AH/2);
   lower_val = LO-(AH/2);

   if(now_val>=upper_val)
   {
    //OUT0 = 1;
    OUT1 = 0;
   ALARM  = 1;
   }

   else if(now_val<=lower_val)
    {
    //  OUT0 = 0;
      OUT1 = 1;
      ALARM = 0;

    }

  }


else if(CTR ==2)
{
  alarm_over_vhh = UP+AH/2;
  alarm_over_vll = UP-AH/2;
  alarm_low_vhh  = LO+AH/2;
  alarm_low_vll  = LO-AH/2;

     if((now_val>=alarm_low_vhh)&&(now_val<=alarm_over_vll))
     {
      ALARM = 0;
   // OUT0 = 0;
    OUT1 = 1;
    }
     else if((now_val<alarm_low_vll)||(now_val>alarm_over_vhh))
     {
      ALARM = 1;
   // OUT0 = 1;
    OUT1 = 0;
    }

  }
else if (CTR ==3)
{
   upper_val = UP+(AH/2);
   lower_val = UP-(AH/2);


   if(now_val>=upper_val)
   {
    //OUT0 = 0;
    OUT1 = 1;
   ALARM  = 0;
   }

   else if(now_val<=lower_val)
    {
    //  OUT0 = 1;
      OUT1 = 0;
      ALARM = 1;

    }
  }
  }
 }
}



void uart1(void) interrupt 16                          //中断函数 中断向量号
{
  uchar data i;
  uint8_t j=0;

  if((SCON1 & 0x01)==1)
  {
      tl0timer0=0;//定时器0累加清0
      SCON1 &= ~0x10;//REN1=0;RI1=0;关接收

    i=SBUF1;//读数据


    if(uart1_receve_end==0)//上次的数据还没有处理，不再接收
    {
      if(i==0x23)
      {

        b_head_sure=1;//数据帧正确
        rcv_count=1;//重新计数
        receive1[0]=i;//保存数据
      }
      else if(b_head_sure==1)
      {
        if(rcv_count<12)//只接收12个字节
        {

          receive1[rcv_count]=i;//保存数据

          rcv_count++;
          if(rcv_count==12)
          {uart1_receve_end=1;
          b_head_sure=0;}//接收完成
        }
      }
    }
   SCON1 |= 0x10;//开启接收REN1=1
   SCON1 &= ~0x01;//清零RI1

  }

    if((SCON1&0x02)==0x02)//发送数据
  {

    if(txcount0!=0)//还有数据需要发送
    {
//      SBUF1=send_buf[txptr0];//装数据
      SBUF1=send_buf[txptr0];
      txptr0++;//指针指向下一个地址
      txcount0--;//已发送数据数量+1
    }
      else//已经发送完的处理
    {
      if(par_buf[37]==1)
       {
      RXEN=1;//开启485接收
            DXEN=0;//
      }
    tl0timer0=0;//定时器0累加清0
    }
    SCON1 &= ~0x02;//清零TI1;
  }
}
void direct_uart1()
{
  uchar i;
  uchar data ADD;
  // uchar check_sum = 0;
   bit  check_ok  = 0;
   uchar order_val1 = 0;
   uchar order_val2 = 0;

    if(tl0timer0<5)return;
  if(find_key!=0)return;
  if(uart1_receve_end==0)return;
     uart1_receve_end = 0;
//check_sum=receive1[1]^receive1[2]^receive1[3]^receive1[4]^receive1[5]^receive1[6]^receive1[7]^receive1[8]^receive1[9];
    ADD = par_buf[20]+0x41;
   if((receive1[11]==0x0a)&&(receive1[1]==ADD))
   {
     check_ok=1;

   }
   else
   {
      check_ok=0;
  }


 if(check_ok==1)
   {
      order_val1=receive1[2];
      order_val2=receive1[3];

      if (( order_val1=='D')&&(order_val2=='T'))
    {

       send_enable = 1;
           if(par_buf[37]==1)
       {
      send_dt();
         }

    }
    else if ((order_val1=='R')&&(order_val2=='B'))
    {

          send_enable = 0;
      rb1();
      send_rb();
//          send_crc_buf1();

    }
    else if ((order_val1=='R')&&(order_val2=='E'))
    {

        send_enable = 0;
      re1();
        send_re();

    }
    else if ((order_val1=='O')&&(order_val2=='P'))
    {

      send_enable = 0;
      opt();
      send_opt();



    }
    else if ((order_val1=='D')&&(order_val2=='L'))
    {

      send_enable = 0;
      dl1();
      send_dl();



    }
    else if ((order_val1=='D')&&(order_val2=='H'))
    {

      send_enable = 0;
      dh1();
      send_dh();
    }

    else if ((order_val1=='A')&&(order_val2=='H'))
    {

      send_enable = 0;
      ah1();
      send_ah();

    }
       else if ((order_val1=='R')&&(order_val2=='L'))
    {

      send_enable = 0;
      rl();
      send_rl();

    }
    else if ((order_val1=='P')&&(order_val2=='O'))
    {

      send_enable = 0;
      po();
      send_po();

    }
    else if ((order_val1=='P')&&(order_val2=='A'))  //显示参数
    {

      send_enable = 0;

      pa();


    }
      else if ((order_val1=='P')&&(order_val2=='R'))  //复位参数
    {

          send_enable = 0;

            EA=0;
          erase_par();
           init_par();
       EA =1;

    }
         for(i=0;i<10;i++)
   {
     receive1[i]=' ';

      }

  }
}
void pa(void)                                  //显示参数
{

      send_rb();
          delay1s();
      send_re();
          delay1s();
      send_opt();
          delay1s();
      send_dl();
          delay1s();
      send_dh();
          delay1s();
      send_ah();
          delay1s();
      send_rl();
          delay1s();
      send_po();
          delay1s();
      send_ver();

}

void send_crc_buf(void)//发固定的12字节的数据
{

  send_buf[0]=0x05;                     //本机地址    //modified
  send_buf[1]='#';                      // 在send_buf数组的第一个位置放入字符 '#'   //modified
    send_buf[2]=par_buf[20]+0x41;       // 根据par_buf数组中的某个元素，计算出要发送的第二个字节  //modified
  send_buf[11]=0x0d;                    // 设置数据包的倒数第二个字节为 0x0d（回车符）//modified
    //send_buf[10]=send_buf[1]^send_buf[2]^send_buf[3]^send_buf[4]^send_buf[5]^send_buf[6]^send_buf[7]^send_buf[8]^send_buf[9];
  send_buf[12]=0x0a;                    // 设置数据包的最后一个字节为 0x0a（换行符）  //modified
//   if(par_buf[37]==0x01)                // 如果par_buf数组中的第37个元素等于 0x01
//  {
//    DXEN = 1;                           // 将 DXEN 置 1，发送使能标志位
//    RXEN = 0;                           // 将 RXEN 置 0，接收使能标志位
//   }

  txcount0=12;                              //modified
  txptr0=1;//从第1个字节开始发送
  SBUF1=send_buf[0];//发数据
}



 void send_dt(void)
{
  uchar idata  rd1,rd2,rd3,rd4,rd5;

  ulint data now_val_temp;

  if(find_key!=0) return;

    now_val_temp = now_val;

  rd1 = now_val_temp/10000;
  rd2 = (now_val_temp%10000)/1000;
  rd3 = (now_val_temp%1000)/100;
  rd4 = (now_val_temp%100)/10;
  rd5 = now_val_temp%10;

    rd1 = rd1+0x30;
    rd2 = rd2+0x30;
    rd3 = rd3+0x30;
    rd4 = rd4+0x30;
    rd5 = rd5+0x30;


if(send_enable==1)
{

      send_buf[3]= 'D';            //modified
      send_buf[4]= 'T';            //modified
   if(err_flag==0)
   {

      send_buf[5]=rd1;             //modified
      send_buf[6]=rd2;
      send_buf[7]='.';
      send_buf[8]= rd3;
      send_buf[9]=rd4;
      send_buf[10]=rd5;
  }
  else if(err_flag==1)
    {
      send_buf[5]=receive0_temp[0];
      send_buf[6]= receive0_temp[1];
      send_buf[7]=receive0_temp[2];
      send_buf[8]=' ';
      send_buf[9]=' ';
      send_buf[10]=' ';


    }
  send_crc_buf();




  }



}

 void send_dt1(void)
{
  uchar idata  rd1,rd2,rd3,rd4,rd5;

  ulint data now_val_temp;
    if(tl0timer0<5)return;//此值为5比较好，太小了，发送值太快，会影响UART0的接收。
  if(find_key!=0) return;

    now_val_temp = now_val;

  rd1 = now_val_temp/10000;
  rd2 = (now_val_temp%10000)/1000;
  rd3 = (now_val_temp%1000)/100;
  rd4 = (now_val_temp%100)/10;
  rd5 = now_val_temp%10;

    rd1 = rd1+0x30;
    rd2 = rd2+0x30;
    rd3 = rd3+0x30;
    rd4 = rd4+0x30;
    rd5 = rd5+0x30;


if(send_enable==1)
{


   if(err_flag==0)
   {

      send_buf[1]=rd1;
      send_buf[2]=rd2;
      send_buf[3]='.';
      send_buf[4]= rd3;
      send_buf[5]=rd4;
      send_buf[6]=rd5;
  }
  else if(err_flag==1)
    {
      send_buf[1]=receive0_temp[0];
      send_buf[2]= receive0_temp[1];
      send_buf[3]=receive0_temp[2];
      send_buf[4]=' ';
      send_buf[5]=' ';
      send_buf[6]=' ';


    }
    send_buf[7]=0x0d;
    send_buf[8]=0x0a;
  txcount0=7;
  txptr0=1;//从第1个字节开始发送
  SBUF1=send_buf[0];//发数据
  delay1s();



  }



}
void rb1(void)
{


      if((receive1[6]=='.')&&(receive1[4]>=0x30)&&(receive1[4]<=0x39)&&(receive1[5]>=0x30)&&(receive1[5]<=0x39)&&(receive1[7]>=0x30)&&(receive1[7]<=0x39)&&(receive1[8]>=0x30)&&(receive1[8]<=0x39)&&(receive1[9]>=0x30)&&(receive1[9]<=0x39))
        {
          receive1[4]=receive1[4]-0x30;
          receive1[5]=receive1[5]-0x30;
          receive1[7]=receive1[7]-0x30;
          receive1[8]=receive1[8]-0x30;
          receive1[9]=receive1[9]-0x30;

          par_buf[0]=receive1[4];
          par_buf[1]=receive1[5];
          par_buf[2]=receive1[7];
          par_buf[3]=receive1[8];
          par_buf[4]=receive1[9];



        EA=0;

        erase_par();
        write_par();  //将修改好后的内容写入flash
         EA =1;

          send_rb0();
        }


}

void send_rb(void)
{
      send_buf[3]= 'R';                      //modified
      send_buf[4]= 'B';
      send_buf[5]=par_buf[0]+0x30;
      send_buf[6]=par_buf[1]+0x30;
      send_buf[7]='.';
      send_buf[8]=par_buf[2]+0x30;
      send_buf[9]=par_buf[3]+0x30;
      send_buf[10]=par_buf[4]+0x30;


      send_crc_buf();
}
void send_rb0()
{
  // SCON0 = 0x00;
  ES0=0;
   PCA0CPH4 = 0xa0;
  SBUF0 = 'R';  while(!TI0);TI0 = 0;
SBUF0 = 'B';  while(!TI0);TI0 = 0;
SBUF0 = par_buf[0]+0x30;while(!TI0);TI0 = 0;
SBUF0 = par_buf[1]+0x30;while(!TI0);TI0 = 0;
SBUF0 = '.';            while(!TI0);TI0 = 0;
SBUF0 = par_buf[2]+0x30;while(!TI0);TI0 = 0;
SBUF0 = par_buf[3]+0x30;while(!TI0);TI0 = 0;
SBUF0 = par_buf[4]+0x30;while(!TI0);TI0 = 0;
SBUF0 = 0x0d;           while(!TI0);TI0 = 0;
SBUF0 = 0x0a;           while(!TI0);TI0 = 0;
delay1s();
delay1s();
SBUF0 = 'D';while(!TI0);TI0 = 0;
SBUF0 = 'T';while(!TI0);TI0 = 0;
SBUF0 = 0x0d;           while(!TI0);TI0 = 0;
SBUF0 = 0x0a;           while(!TI0);TI0 = 0;

delay1s();
delay1s();

  // SCON0 = 0x10;
   ES0=1;
}
void send_rb3()                         //模拟量3起始值
{
  // SCON0 = 0x00;
  ES0=0;
   PCA0CPH4 = 0xa0;
  SBUF0 = 'R';  while(!TI0);TI0 = 0;
SBUF0 = 'B';  while(!TI0);TI0 = 0;
SBUF0 = par_buf[0]+0x30;while(!TI0);TI0 = 0;
SBUF0 = par_buf[1]+0x30;while(!TI0);TI0 = 0;
SBUF0 = '.';            while(!TI0);TI0 = 0;
SBUF0 = par_buf[2]+0x30;while(!TI0);TI0 = 0;
SBUF0 = par_buf[3]+0x30;while(!TI0);TI0 = 0;
SBUF0 = par_buf[4]+0x30;while(!TI0);TI0 = 0;
SBUF0 = 0x0d;           while(!TI0);TI0 = 0;
SBUF0 = 0x0a;           while(!TI0);TI0 = 0;
delay1s();
delay1s();

   ES0=1;
}
void re1(void)                           //模拟量1的终点值
{



    if((receive1[6]=='.')&&(receive1[4]>=0x30)&&(receive1[4]<=0x39)&&(receive1[5]>=0x30)&&(receive1[5]<=0x39)&&(receive1[7]>=0x30)&&(receive1[7]<=0x39)&&(receive1[8]>=0x30)&&(receive1[8]<=0x39)&&(receive1[9]>=0x30)&&(receive1[9]<=0x39))
      {
        receive1[4]=receive1[4]-0x30;
        receive1[5]=receive1[5]-0x30;
        receive1[7]=receive1[7]-0x30;
        receive1[8]=receive1[8]-0x30;
        receive1[9]=receive1[9]-0x30;

        par_buf[5]=receive1[4];
        par_buf[6]=receive1[5];
        par_buf[7]=receive1[7];
        par_buf[8]=receive1[8];
        par_buf[9]=receive1[9];






        EA=0;

        erase_par();
        write_par();  //将修改好后的内容写入flash
          EA =1;
          send_re0();
      }


}
void send_re(void)
{


      send_buf[3]= 'R';
      send_buf[4]= 'E';
      send_buf[5]=par_buf[5]+0x30;
      send_buf[6]=par_buf[6]+0x30;
      send_buf[7]='.';
      send_buf[8]=par_buf[7]+0x30;
      send_buf[9]=par_buf[8]+0x30;
      send_buf[10]=par_buf[9]+0x30;

      send_crc_buf();

}
void send_re0()
{
  // SCON0 = 0x00;
   ES0=0;
   PCA0CPH4 = 0xa0;
SBUF0 = 'R';           while(!TI0);TI0 = 0;
SBUF0 = 'E';             while(!TI0);TI0 = 0;
SBUF0 = par_buf[5]+0x30; while(!TI0);TI0 = 0;
SBUF0 = par_buf[6]+0x30; while(!TI0);TI0 = 0;
SBUF0 = '.';             while(!TI0);TI0 = 0;
SBUF0 = par_buf[7]+0x30; while(!TI0);TI0 = 0;
SBUF0 = par_buf[8]+0x30; while(!TI0);TI0 = 0;
SBUF0 = par_buf[9]+0x30; while(!TI0);TI0 = 0;
SBUF0 = 0x0d;            while(!TI0);TI0 = 0;
SBUF0 = 0x0a;            while(!TI0);TI0 = 0;
delay1s();
delay1s();
SBUF0 = 'D';while(!TI0);TI0 = 0;
SBUF0 = 'T';while(!TI0);TI0 = 0;
SBUF0 = 0x0d;           while(!TI0);TI0 = 0;
SBUF0 = 0x0a;           while(!TI0);TI0 = 0;
delay1s();
delay1s();
// SCON0 = 0x10;
 ES0=1;
}
void send_re3()
{
  // SCON0 = 0x00;
   ES0=0;
   PCA0CPH4 = 0xa0;
SBUF0 = 'R';           while(!TI0);TI0 = 0;
SBUF0 = 'E';             while(!TI0);TI0 = 0;
SBUF0 = par_buf[5]+0x30; while(!TI0);TI0 = 0;
SBUF0 = par_buf[6]+0x30; while(!TI0);TI0 = 0;
SBUF0 = '.';             while(!TI0);TI0 = 0;
SBUF0 = par_buf[7]+0x30; while(!TI0);TI0 = 0;
SBUF0 = par_buf[8]+0x30; while(!TI0);TI0 = 0;
SBUF0 = par_buf[9]+0x30; while(!TI0);TI0 = 0;
SBUF0 = 0x0d;            while(!TI0);TI0 = 0;
SBUF0 = 0x0a;            while(!TI0);TI0 = 0;
delay1s();
delay1s();

 ES0=1;
}

void dl1(void)
{



    if((receive1[6]=='.')&&(receive1[4]>=0x30)&&(receive1[4]<=0x39)&&(receive1[5]>=0x30)&&(receive1[5]<=0x39)&&(receive1[7]>=0x30)&&(receive1[7]<=0x39)&&(receive1[8]>=0x30)&&(receive1[8]<=0x39)&&(receive1[9]>=0x30)&&(receive1[9]<=0x39))
      {
        receive1[4]=receive1[4]-0x30;
        receive1[5]=receive1[5]-0x30;
        receive1[7]=receive1[7]-0x30;
        receive1[8]=receive1[8]-0x30;
        receive1[9]=receive1[9]-0x30;

        par_buf[15]=receive1[4];
        par_buf[16]=receive1[5];
        par_buf[17]=receive1[7];
        par_buf[18]=receive1[8];
        par_buf[19]=receive1[9];




        EA=0;

        erase_par();
        write_par();  //将修改好后的内容写入flash
          EA=1;

  }

}

void send_dl(void)                                    //报警点1值
{

      send_buf[3]= 'D';
      send_buf[4]= 'L';
      send_buf[5]=par_buf[15]+0x30;
      send_buf[6]=par_buf[16]+0x30;
      send_buf[7]='.';
      send_buf[8]=par_buf[17]+0x30;
      send_buf[9]=par_buf[18]+0x30;                   //modified
      send_buf[10]=par_buf[19]+0x30;

        send_crc_buf();

}
void dh1(void)
{


    if((receive1[6]=='.')&&(receive1[4]>=0x30)&&(receive1[4]<=0x39)&&(receive1[5]>=0x30)&&(receive1[5]<=0x39)&&(receive1[7]>=0x30)&&(receive1[7]<=0x39)&&(receive1[8]>=0x30)&&(receive1[8]<=0x39)&&(receive1[9]>=0x30)&&(receive1[9]<=0x39))
      {
        receive1[4]=receive1[4]-0x30;
        receive1[5]=receive1[5]-0x30;
        receive1[7]=receive1[7]-0x30;
        receive1[8]=receive1[8]-0x30;
        receive1[9]=receive1[9]-0x30;

        par_buf[10]=receive1[4];
        par_buf[11]=receive1[5];
        par_buf[12]=receive1[7];
        par_buf[13]=receive1[8];
        par_buf[14]=receive1[9];
        EA=0;

        erase_par();
        write_par();  //将修改好后的内容写入flash
          EA=1;
      }




}
void send_dh(void)                                   //报警点2值
{

      send_buf[3]= 'D';
      send_buf[4]= 'H';
      send_buf[5]=par_buf[10]+0x30;
      send_buf[6]=par_buf[11]+0x30;                            //modified
      send_buf[7]='.';
      send_buf[8]=par_buf[12]+0x30;
      send_buf[9]=par_buf[13]+0x30;
      send_buf[10]=par_buf[14]+0x30;
        send_crc_buf();

}

void ah1(void)
{



    if((receive1[6]=='.')&&(receive1[4]>=0x30)&&(receive1[4]<=0x39)&&(receive1[5]>=0x30)&&(receive1[5]<=0x39)&&(receive1[7]>=0x30)&&(receive1[7]<=0x39)&&(receive1[8]>=0x30)&&(receive1[8]<=0x39)&&(receive1[9]>=0x30)&&(receive1[9]<=0x39))
      {
        receive1[4]=receive1[4]-0x30;
        receive1[5]=receive1[5]-0x30;
        receive1[7]=receive1[7]-0x30;
        receive1[8]=receive1[8]-0x30;
        receive1[9]=receive1[9]-0x30;

        par_buf[21]=receive1[4];
        par_buf[22]=receive1[5];
        par_buf[23]=receive1[7];
        par_buf[24]=receive1[8];
        par_buf[25]=receive1[9];





        EA=0;

        erase_par();
        write_par();  //将修改好后的内容写入flash
         EA=1;
  }




}
void send_ah(void)                                //读报警迟滞值
{

      send_buf[3]= 'A';
      send_buf[4]= 'H';
      send_buf[5]=par_buf[21]+0x30;
      send_buf[6]=par_buf[22]+0x30;
      send_buf[7]='.';
      send_buf[8]=par_buf[23]+0x30;
      send_buf[9]=par_buf[24]+0x30;                        //modified
      send_buf[10]=par_buf[25]+0x30;
        send_crc_buf();
}


void opt(void)
{

    if((receive1[4]>=0x31)&&(receive1[4]<=0x33)&&(receive1[5]==0x30)&&(receive1[6]==0x30)&&(receive1[7]==0x30)&&(receive1[8]==0x30)&&(receive1[9]==0x30))
      {
        receive1[4]=receive1[4]-0x30;

        par_buf[34]=receive1[4];



         EA=0;

        erase_par();
        write_par();  //将修改好后的内容写入flash
        EA=1;

        }
}


void send_opt(void)                      //报警点选择
{
      send_buf[3]= 'O';
      send_buf[4]= 'P';
      send_buf[5]=par_buf[34]+0x30;                 //modified
      send_buf[6]=' ';
      send_buf[7]=' ';
      send_buf[8]=' ';
      send_buf[9]=' ';
      send_buf[10]=' ';
        send_crc_buf();
}
 void rl(void)                    //RELAY
{

   if((receive1[4]>=0x30)&&(receive1[4]<=0x31)&&(receive1[5]==0x30)&&(receive1[6]==0x30)&&(receive1[7]==0x30)&&(receive1[8]==0x30)&&(receive1[9]==0x30))
     {
     receive1[4]=receive1[4]-0x30;

     par_buf[35]=receive1[4];



         EA=0;

        erase_par();
        write_par();  //将修改好后的内容写入flash
        EA=1;

        }
}


void send_rl(void)                    //继电器
{
      send_buf[3]= 'R';
      send_buf[4]= 'L';
      send_buf[5]=par_buf[35]+0x30;                    //modified
      send_buf[6]=' ';
      send_buf[7]=' ';
      send_buf[8]=' ';
      send_buf[9]=' ';
      send_buf[10]=' ';
        send_crc_buf();
}
void po(void)                       //PNP
{

  if((receive1[4]>=0x30)&&(receive1[4]<=0x31)&&(receive1[5]==0x30)&&(receive1[6]==0x30)&&(receive1[7]==0x30)&&(receive1[8]==0x30)&&(receive1[9]==0x30))
    {
    receive1[4]=receive1[4]-0x30;

    par_buf[36]=receive1[4];



         EA=0;

        erase_par();
        write_par();  //将修改好后的内容写入flash

           EA=1;
        }
}


void send_po(void)                   //PNP
{
      send_buf[3]= 'P';
      send_buf[4]= 'O';
      send_buf[5]=par_buf[36]+0x30;               //modified
      send_buf[6]=' ';
      send_buf[7]=' ';
      send_buf[8]=' ';
      send_buf[9]=' ';
      send_buf[10]=' ';
        send_crc_buf();
}
void send_ver()                    //version
{
      send_buf[3]= 'V';
      send_buf[4]= 'e';
      send_buf[5]='r';
      send_buf[6]=version_major;                    //modified
      send_buf[7]='.';
      send_buf[8]=version_minor;
      send_buf[9]='0';
      send_buf[10]='0';
      send_crc_buf();

}

//-----------------------------------------------------------------------------
// Static Functions - State Machine
//-----------------------------------------------------------------------------

// Ready to receive a command from the host
static void StateIdle (void)
{
   // Recieved an OUT packet
   if (Out_Packet_Ready)
   {
      // Decode the command
      switch (Out_Packet[0])
      {
         case CMD_SET_FLASH_KEY:
            State = ST_SET_FLASH_KEY;
            break;

         case CMD_GET_PAGE_INFO:
            // Done processing Out_Packet
            Out_Packet_Ready = 0;
            State = ST_TX_PAGE_INFO;
            break;

         case CMD_READ_PAGE:
            State = ST_READ_PAGE;
            break;

         case CMD_WRITE_PAGE:
            State = ST_WRITE_PAGE;
            break;

         default:
            // Done processing Out_Packet
            Out_Packet_Ready = 0;
            State = ST_TX_INVALID;
            break;
      }
   }
}

// Receive flash key from host
static void StateSetFlashKey (void)
{
   // Set the flash key
   SetFlashKey (&Out_Packet[1]);

   // Done processing Out_Packet
   Out_Packet_Ready = 0;

   State = ST_TX_SUCCESS;
}

// Send number of flash pages and flash page size to host
static void StateTxPageInfo (void)
{
   // If able to send an IN packet
   if (!In_Packet_Ready)
   {
      In_Packet[0] = RSP_SUCCESS;
      In_Packet[1] = FLASH_NUM_PAGES;
      In_Packet[2] = LOBYTE(FLASH_PAGE_SIZE);
      In_Packet[3] = HIBYTE(FLASH_PAGE_SIZE);

      // Ready to send IN packet
      In_Packet_Ready = 1;

      State = ST_IDLE;
   }
}

// Send flash page to host in blocks (command stage)
static void StateReadPage (void)
{
   // Turn on LED1 while reading page
   SetLed (1, 1);

   // Store the page number
   TxPage = Out_Packet[1];

   // Done processing Out_Packet
   Out_Packet_Ready = 0;

   // Reset the block counter
   //
   // There are 8, 64-byte blocks in a 512-byte
   // flash page
   TxBlock = 0;

   // Check if the requested page number is valid.
   //
   // If the page is invalid, we'll send dummy blocks
   // back and return an error in the response packet.
   if (TxPage < FLASH_NUM_PAGES)
      TxValid = 1;
   else
      TxValid = 0;

   State = ST_TX_BLOCK;
}

// Send flash page to host in blocks (data/response stage)
static void StateTxBlock (void)
{
   // Response stage:
   // Finished sending last block to host
   if (TxBlock == 8)
   {
      // Turn off LED1 after finished reading page
      SetLed (1, 0);

      // Send appropriate response
      if (TxValid)
         State = ST_TX_SUCCESS;
      else
         State = ST_TX_INVALID;
   }
   // Data stage:
   // If able to send an IN packet
   else if (!In_Packet_Ready)
   {
      // Calculate the flash address based on current page and block number
      uint16_t address = FLASH_START + (FLASH_PAGE_SIZE * TxPage) + (IN_EP1_PACKET_SIZE * TxBlock);

      // Only read from flash if the flash page is valid.
      // Otherwise, send whatever dummy data is left in In_Packet
      if (TxValid)
      {
         ReadFlashPage (address, In_Packet, IN_EP1_PACKET_SIZE);
      }

      // Ready to send IN packet
      In_Packet_Ready = 1;

      TxBlock++;
   }
}

// Receive flash page from host in blocks and write to flash (command stage)
static void StateWritePage (void)
{
   // Turn on LED2 while writing page
   SetLed (2, 1);

   // Store the page number
   RxPage = Out_Packet[1];

   // Done processing Out_Packet
   Out_Packet_Ready = 0;

   // Reset the block counter
   //
   // There are 8, 64-byte blocks in a 512-byte
   // flash page
   RxBlock = 0;

   // Check if the requested page number is valid.
   //
   // If the page is invalid, we'll send dummy blocks
   // back and return an error in the response packet.
   if (RxPage < FLASH_NUM_PAGES)
   {
      // Flash page is valid
      RxValid = 1;

      // Erase the flash page
      EraseFlashPage (FLASH_START + (RxPage * FLASH_PAGE_SIZE));
   }
   else
   {
      // Flash page is invalid
      RxValid = 0;
   }

   State = ST_RX_BLOCK;
}

// Receive flash page from host in blocks and write to flash (response/data stage)
static void StateRxBlock (void)
{
   // Response stage:
   // Finished receiving last block
   if (RxBlock == 8)
   {
      // Turn off LED2 after finished writing page
      SetLed (2, 0);

      if (RxValid)
         State = ST_TX_SUCCESS;
      else
         State = ST_TX_INVALID;
   }
   // Data stage:
   // If received an OUT packet
   else if (Out_Packet_Ready)
   {
      uint16_t address = FLASH_START + (FLASH_PAGE_SIZE * RxPage) + (OUT_EP1_PACKET_SIZE * RxBlock);

      if (RxValid)
      {
         WriteFlashPage (address, Out_Packet, OUT_EP1_PACKET_SIZE);
      }

      // Finished processing OUT packet
      Out_Packet_Ready = 0;

      RxBlock++;
   }
}

// Send a response (success)
static void StateTxSuccess (void)
{
   // If able to send an IN packet
   if (!In_Packet_Ready)
   {
      In_Packet[0] = RSP_SUCCESS;

      // Ready to send IN packet
      In_Packet_Ready = 1;

      State = ST_IDLE;
   }
}

// Send a response (invalid)
static void StateTxInvalid (void)
{
   // If able to send an IN packet
   if (!In_Packet_Ready)
   {
      In_Packet[0] = RSP_INVALID;

      // Ready to send IN packet
      In_Packet_Ready = 1;

      State = ST_IDLE;
   }
}

// Manage state transitions
static void StateMachine (void)
{
   uint8_t EA_Save;

   // Received a reset state control request
   // from the host. Transition to the idle
   // state after completing the current state
   // function
   if (AsyncResetState)
   {
      // Disable interrupts
      EA_Save = IE_EA;
      IE_EA = 0;

      // Set index to endpoint 1 registers
      POLL_WRITE_BYTE(INDEX, 1);

      // Flush IN/OUT FIFOs
      POLL_WRITE_BYTE(EINCSR1, rbInFLUSH);
      POLL_WRITE_BYTE(EOUTCSR1, rbOutFLUSH);

      // Flush software In_Packet/Out_Packet buffers
      In_Packet_Ready = 0;
      Out_Packet_Ready = 0;

      // Reset state to idle
      State = ST_IDLE;

      // Turn off both LEDs
      SetLed(1, 0);
      SetLed(2, 0);

      // Finished resetting state
      AsyncResetState = 0;

      // Restore interrupts
      IE_EA = EA_Save;
   }

   switch (State)
   {
      case ST_IDLE:
         StateIdle ();
         break;

      case ST_SET_FLASH_KEY:
         StateSetFlashKey ();
         break;

      case ST_TX_PAGE_INFO:
         StateTxPageInfo ();
         break;

      case ST_READ_PAGE:
         StateReadPage ();
         break;

      case ST_TX_BLOCK:
         StateTxBlock ();
         break;

      case ST_WRITE_PAGE:
         StateWritePage ();
         break;

      case ST_RX_BLOCK:
         StateRxBlock ();
         break;

      case ST_TX_SUCCESS:
         StateTxSuccess ();
         break;

      case ST_TX_INVALID:
         StateTxInvalid ();
         break;
   }
}




/////////////////////////////////////////////////////
/*                     主程序                      */
/////////////////////////////////////////////////////
void main(void)
{
    uchar i;


    //--------------------------usb------------------------
    uint8_t EA_Save;

    System_Init ();                     // Initialize Sysclk, Port IO
    USB0_Init ();                       // Initialize USB0

    IE_EA = 1;                             // Enable global interrupts
    //------------------------------------------------------

    Init_Device();

  delay1s();
  delay1s();
  delay1s();

  m_init();
  m_send(0x01,0xff);                  //8.8.8 8 8
  m_send(0x02,0xff);
  m_send(0x03,0x7f);
  m_send(0x04,0x7f);
  m_send(0x05,0x7f);


    Power = 0;
    EA = 0;              //禁用所有中断源
   read_par();

  if (( (par_buf[26]==0x01)&&(par_buf[27]==0x02)&&(par_buf[28]==0x03))==0)
  {
    erase_par();
    init_par();

  }

  for (i=0;i<38;i++)                     //验证
  {
    if ( par_buf[i]>9)
    {
        PCA0CPH4 = 0xa0;
        erase_par();
        init_par();

    }
  }



  //STC1387init
//    SLEW = 1;
//  if(par_buf[37]==0x00)
//  {
//  MODE = 0;//RS232 mode
//  }
//  else if(par_buf[37]==0x01)
//  {MODE = 1;}//RS485 mode


  RXEN = 1;
  DXEN = 0;


  UP = par_buf[10]*10000+par_buf[11]*1000+par_buf[12]*100+par_buf[13]*10+par_buf[14];
  LO = par_buf[15]*10000+par_buf[16]*1000+par_buf[17]*100+par_buf[18]*10+par_buf[19];
  AH = par_buf[21]*10000+par_buf[22]*1000+par_buf[23]*100+par_buf[24]*10+par_buf[25];


  CTR = par_buf[34];



    read_par();


  // 激光测距仪初始化时间

  delay1s();
  delay1s();
    delay1s();
  delay1s();
  delay1s();
  delay1s();


  trans0("ASdt\r\n");
  delay1s();
  delay1s();
  trans0("ASdt\r\n");
  delay1s();
  delay1s();

  trans0("SE2\r\n");
  delay1s();
  delay1s();

  send_rb3();
  send_re3();

  trans0("DT\r\n");

  delay1s();
  delay1s();
    trans0("DT\r\n");
  delay1s();

    function_flag = 0;
  find_key = 0;

  //Added to fix Comm lockup problem MD 20210409
  i=SBUF1;        // flush UART1 input buffer
  i=SBUF1;        // flush UART1 input buffer
  rcv_count = 0;      // state: waiting for '#'
  uart1_receve_end = 0; // state: no complete message received
  b_head_sure=0;      // state: no '#' found
  UART1_Init();     // reinitialize UART1
  RXEN = 1;       // RS-485 driver receive enable
  DXEN = 0;       // RS-485 driver transmit disable

  EA = 1;                 //all interrupt enable
  RI0 = 0;                //receive flag
  ES0 = 1;        //Enable UART0 Interrupt
  SCON1 |= 0x01;//开启一次中断，解决UART1优先级高导致UART0不工作的情况（UART1中断一次后UART0才能进入中断）
  i=SBUF1;        // flush UART1 input buffer
  i=SBUF1;        // flush UART1 input buffer
  // delay1s();

    delay20ms();
  while(1)
  {
       PCA0CPH4 = 0xa0;

     scan_keyboard();

      direct_uart0();

     direct_uart1();
     if(par_buf[37]==1)
     {send_dt1();  }



     StateMachine ();

     EA_Save = IE_EA;
     IE_EA = 0;
     Send_Packet_Foreground ();
     IE_EA = EA_Save;
//       OUT0=1;
//       OUT1=1;
//       delay1s();
//       delay1s();
//       delay1s();
//       delay1s();
//       delay1s();
//       delay1s();
//       delay1s();
//       delay1s();
//       delay1s();
//
//       OUT0=0;
//       OUT1=0;
//       delay1s();
//       delay1s();
//       delay1s();
//       delay1s();
//       delay1s();
//       delay1s();
//       delay1s();
//       delay1s();
//       delay1s();

//      send_crc_buf1();

//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
//      delay1s();
  }
}
