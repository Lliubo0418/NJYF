
#ifndef  __DATA_PROCESS_H
#define  __DATA_PROCESS_H

#include "sys.h"

#define   DEBUG_DATA_PROCESS

#define  DATA_COUNT     200			// 实际显示数据个数：10000个 u16_t；处理数据个数：(gate_right - gate_left).

#define  PEAK_OFFSET            5
#define  PEAK_RANGE_COUNT       (PEAK_OFFSET * 2 + 1)

#define  CONFIGURATION_ADDRESS_BEGIN     0x0800C000

#define     DEVICE_SIDE_DS          0x00
#define     DEVICE_SIDE_WS          0x01

// 距离计算相关宏
#define  PEAK_POS_OFFSET         833         // 零点位置偏移



// 定时器相关宏
#define  HEARTBEAT_INTERVAL      105U        // 心跳定时器间隔（ms）

typedef struct{
    uint16_t    ds_last_phase;
    int16_t     ds_phase_offset_sum;
    uint16_t    ws_last_phase;
    int16_t     ws_phase_offset_sum;

}ST_RSP_PARAMETER;

/* 参数帧结构体（128字节） */
#pragma pack(push, 1) // 确保无填充
typedef struct {
    uint8_t  header[3];                      // 字节0–2: 帧头 (01 01 FE)
    uint16_t ds_strip_edge_position;         // 字节3–4: DS STRIP EDGE POSITION (0.1mm)
    uint16_t ws_strip_edge_position;         // 字节5–6: WS STRIP EDGE POSITION (0.1mm)
    uint16_t ds_base_position;               // 字节7–8: DS BASE POSITION (0.1mm)
    uint16_t ws_base_position;               // 字节9–10: WS BASE POSITION (0.1mm)
    uint16_t ds_theta_base;                  // 字节11–12: DS θ BASE (°)
    uint16_t ws_theta_base;                  // 字节13–14: WS θ BASE (°)
    uint16_t ds_theta_current;               // 字节15–16: DS θ CURRENT (°)
    uint16_t ws_theta_current;               // 字节17–18: WS θ CURRENT (°)
    int16_t ds_rotate_n_data;               // 字节19–20: DS ROTATE N DATA
    int16_t ws_rotate_n_data;               // 字节21–22: WS ROTATE N DATA
    uint16_t ds_iq_data;                     // 字节23–24: DS IQ DATA (0.1mm)
    uint16_t ws_iq_data;                     // 字节25–26: WS IQ DATA (0.1mm)
    uint16_t board_temperature;              // 字节27–28: BOARD TEMPERATURE (0.1°C)
    uint16_t ds_antenna_temperature;         // 字节29–30: DS ANTENNA TEMPERATURE (0.1°C)
    uint16_t ws_antenna_temperature;         // 字节31–32: WS ANTENNA TEMPERATURE (0.1°C)
    uint16_t panel_temperature;              // 字节33–34: PANEL TEMPERATURE (0.1°C)
    uint16_t strip_gate_line_left_1;         // 字节35–36: STRIP GATE LINE LEFT (Data)
    uint16_t strip_gate_line_right_1;        // 字节37–38: STRIP GATE LINE RIGHT (Data)
    uint16_t ds_strip_edge_line;             // 字节39–40: DS STRIP EDGE LINE (高位在前)
    uint16_t ws_strip_edge_line;             // 字节41–42: WS STRIP EDGE LINE (高位在前)
    uint16_t iq_theta_long_time_count;       // 字节43–44: IQ-θ LONG TIME COUNT
    uint16_t error;                          // 字节45–46: 作用未明（疑似错误标志)
    uint16_t receive_command;                // 字节47–48: 接收命令
    uint16_t strip_gate_line_left_2;         // 字节49–50: STRIP GATE LINE LEFT (Data)
    uint16_t strip_gate_line_right_2;        // 字节51–52: STRIP GATE LINE RIGHT (Data)
    uint16_t ws_distance_from_zero_position; // 字节53–54: WS DISTANCE FROM ZERO POSITION (0.1mm)
    uint16_t ds_distance_from_zero_position; // 字节55–56: DS DISTANCE FROM ZERO POSITION (0.1mm)
    uint16_t ws_zero_position;               // 字节57–58: WS ZERO POSITION (Data)
    uint16_t ds_zero_position;               // 字节59–60: DS ZERO POSITION (Data)
    uint16_t ws_roll_out;                    // 字节61–62: WS ROLL OUT
    uint16_t ds_roll_out;                    // 字节63–64: DS ROLL OUT
    uint16_t theta_offset;                   // 字节65–66: θ OFFSET (0.01mm)
    uint16_t filter_a;                       // 字节67–68: FILTER A (0.1mm)
    uint16_t filter_c;                       // 字节69–70: FILTER C
    uint16_t wait_counter;                   // 字节71–72: WAIT COUNTER
    uint16_t iq0_iq1_threshold;              // 字节73–74: IQ0-IQ1 THRESHOLD (0.1mm)
    uint16_t theta0_theta1_threshold;        // 字节75–76: θ0-θ1 THRESHOLD (0.1mm)
    uint16_t iq_theta_threshold;             // 字节77–78: IQ-θ THRESHOLD (0.1mm)
    uint16_t iq_theta_long_time_count2;      // 字节79–80: IQ-θ LONG TIME COUNT（重复？）
    uint16_t max_id_number;                  // 字节81–82: MAX ID NUMBER
    uint16_t emw_number;                     // 字节83–84: EMW NUMBER
    uint8_t  sin_gain;                       // 字节85: SIN GAIN
    uint8_t  cos_gain;                       // 字节86: COS GAIN
    uint16_t data_disposal_count;            // 字节87–88: DATA DISPOSAL COUNT
    uint16_t data_read_count;                // 字节89–90: DATA READ COUNT
    uint16_t dirt_warning_level;             // 字节91–92: DIRT WARNING LEVEL
    uint16_t dirt_detect_average;            // 字节93–94: DIRT DETECT AVERAGE
    uint16_t dirt_detect_sampling;           // 字节95–96: DIRT DETECT SAMPLING
    uint8_t  reserved[30];                   // 字节97–126: 保留字节，填充0
    uint8_t  checksum;                       // 字节127: 校验和（字节0–126的和）
} ParameterFrame;

// ADC采集帧结构体（1024字节）
typedef struct {
    uint8_t  header[3];                      // 字节0–2: 帧头 (01 01 FE)
    uint16_t ds_strip_edge_position;         // 字节3–4: DS STRIP EDGE POSITION (0.1mm)
    uint16_t ws_strip_edge_position;         // 字节5–6: WS STRIP EDGE POSITION (0.1mm)
    uint16_t ds_base_position;               // 字节7–8: DS BASE POSITION (0.1mm)
    uint16_t ws_base_position;               // 字节9–10: WS BASE POSITION (0.1mm)
    uint16_t ds_theta_base;                  // 字节11–12: DS θ BASE (°)
    uint16_t ws_theta_base;                  // 字节13–14: WS θ BASE (°)
    uint16_t ds_theta_current;               // 字节15–16: DS θ CURRENT (°)
    uint16_t ws_theta_current;               // 字节17–18: WS θ CURRENT (°)
    uint16_t ds_rotate_n_data;               // 字节19–20: DS ROTATE N DATA
    uint16_t ws_rotate_n_data;               // 字节21–22: WS ROTATE N DATA
    uint16_t ds_iq_data;                     // 字节23–24: DS IQ DATA (0.1mm)
    uint16_t ws_iq_data;                     // 字节25–26: WS IQ DATA (0.1mm)
    uint16_t board_temperature;              // 字节27–28: BOARD TEMPERATURE (0.1°C)
    uint16_t ds_antenna_temperature;         // 字节29–30: DS ANTENNA TEMPERATURE (0.1°C)
    uint16_t ws_antenna_temperature;         // 字节31–32: WS ANTENNA TEMPERATURE (0.1°C)
    uint16_t panel_temperature;              // 字节33–34: PANEL TEMPERATURE (0.1°C)
    uint16_t strip_gate_line_left_1;         // 字节35–36: STRIP GATE LINE LEFT (Data)
    uint16_t strip_gate_line_right_1;        // 字节37–38: STRIP GATE LINE RIGHT (Data)
    uint16_t ds_strip_edge_line;             // 字节39–40: DS STRIP EDGE LINE (高位在前)
    uint16_t ws_strip_edge_line;             // 字节41–42: WS STRIP EDGE LINE (高位在前)
    uint16_t iq_theta_long_time_count;       // 字节43–44: IQ-θ LONG TIME COUNT
    uint16_t error;                          // 字节45–46: 作用未明（疑似错误标志)
    uint16_t receive_command;                // 字节47–48: 接收命令
    uint8_t  adc_data[800];                  // 字节49–848: ADC数据，400个采样点（sin/cos交替，每个 sin/cos 各 2 字节，交替存放 200 组（共 400 个））
    uint8_t  reserved[174];                  // 字节849–1022: 保留字节，填充0
    uint8_t  checksum;                       // 字节1023: 校验和（字节0–1022的和）
} AdcFrame;
#pragma pack(pop)



extern  ParameterFrame      par_params,par_temp_params;
extern  ST_RSP_PARAMETER    gst_rsp_parameter;
extern  uint32_t            param_changed;

extern uint8_t sin_data[400];
extern uint8_t cos_data[400];
extern AdcFrame g_adc_params;


void data_process_init(void);
static inline uint16_t abs_diff(uint16_t a, uint16_t b);
static void check_param_update(void);
void reset_phase_data(void);
static void apply_threshold_corrections(uint16_t *base_pos, uint16_t *base_phase, uint16_t iq_data, uint16_t cur_phase);
static void update_response_parameters(ParameterFrame *p, uint16_t base_pos, uint16_t base_phase,
                                       uint16_t cur_distance, uint16_t cur_phase, uint16_t strip_pos, uint16_t iq_data);

uint32_t Extract_from_Sram(uint32_t peak_index);
int16_t cal_phase(uint32_t peak_index);
uint16_t distance_by_pseudocode(uint32_t peak_index);
uint16_t distance_by_phase(uint16_t cur_distance, uint16_t cur_theta);
uint16_t distance_filter(uint16_t cur_distance);
void data_process(void);

int16_t wrap_phase_diff(int16_t cur, int16_t base);
int16_t compute_phase_diff(int16_t cur, int16_t base, int16_t last_cur);
#endif  /* __DATA_PROCESS_H */
