
#ifndef  __COM_PROCESS_H
#define  __COM_PROCESS_H

#include "sys.h"
#include "data_process.h"
#include "cmsis_os.h"

extern osMutexId_t paramMutexHandle;
extern osMutexId_t usbBufferMutexHandle;

#define     RX_BUFFER_LEN   128
#define     TX_BUFFER_LEN   1024
#define     ADC_BUFFER_LEN  800

extern uint32_t t0, t1;
extern float delta_times;
extern uint32_t  g_tx_flag;
extern uint32_t  g_rx_flag;
extern uint8_t g_tx_buf[TX_BUFFER_LEN];
extern uint8_t sin_data[400], cos_data[400];
//			t0 = TIM5->CNT;
//			t1 = TIM5->CNT;
//			delta_times = (t1 - t0) / 84.0f / 1000.0f;
//			printf("time take%f ms\n", delta_times);


void Handle_Parameter_Read(uint8_t cmd_code);
void Handle_ADC_Collect(uint8_t flag, uint16_t start_pos);
void Handle_Setting_Command(uint8_t *buf, uint8_t cmd_code);
uint8_t Verify_Checksum(uint8_t *buf, uint16_t len);
void Load_Parameters(void);
void Save_Parameter();
void Get_ADC_Data(uint16_t start_pos, uint8_t *sin_data, uint8_t *cos_data, uint16_t count);

// ��˽�������������һ�� uint16_t ���ڴ��е��ֽ�˳��
static inline void SwapUint16BE(uint8_t *p) {
    uint8_t tmp = p[0];
    p[0] = p[1];
    p[1] = tmp;
}

#endif  /* __COM_PROCESS_H */
