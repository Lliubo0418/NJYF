
#ifndef  __COM_PROCESS_H
#define  __COM_PROCESS_H

#include "sys.h"
#include "data_process.h"

#define     RX_BUFFER_LEN   128
#define     TX_BUFFER_LEN   1024
#define     ADC_BUFFER_LEN  800



extern uint8_t g_tx_buf[TX_BUFFER_LEN];

extern uint32_t  g_tx_flag;
extern uint32_t  g_rx_flag;
extern uint8_t g_tx_buf[TX_BUFFER_LEN];

uint8_t parse_message(uint8_t *message, uint32_t msg_len);
void receive_message(uint8_t *message, uint32_t msg_len);

void Handle_Parameter_Read(uint8_t cmd_code);
void Handle_ADC_Collect(uint8_t flag, uint16_t start_pos);
void Handle_Setting_Command(uint8_t *buf, uint8_t cmd_code);
uint8_t Verify_Checksum(uint8_t *buf, uint16_t len);
void Load_Parameters(void);
void Save_Parameter(uint8_t param_id, uint32_t value);
void Get_ADC_Data(uint8_t flag, uint16_t start_pos, uint8_t *sin_data, uint8_t *cos_data, uint16_t count);

// 大端交换函数：交换一个 uint16_t 在内存中的字节顺序
static inline void SwapUint16BE(uint8_t *p) {
    uint8_t tmp = p[0];
    p[0] = p[1];
    p[1] = tmp;
}

#endif  /* __COM_PROCESS_H */
