#include <math.h>
#include <string.h>
#include "data_process.h"
#include "flash.h"
#include "com_process.h"
#include "arm_math.h"
#include "sram.h"
#include "fpga.h"

#define MAX_SAMPLES_WS 20000
#define MAX_SAMPLES_DS 20000
#define PHASE_UNIT 10
#define IQ_DISTANCE_SCALE 0.6f
#define THETA_TO_MM_RATIO 14.02f
#define FIND_PEAK_MIDDLE_OFFSET 30

int16_t amplitude[PEAK_OFFSET * 2 + 1];

extern volatile uint32_t peak_pos;
extern uint8_t ws_ds_flag;
extern uint8_t sin_data[400], cos_data[400];

uint16_t peak_around_sin[10], peak_around_cos[10];
uint16_t find_peak_middle_sin[61], find_peak_middle_cos[61];

ParameterFrame par_params = { .header = {0x01, 0x01, 0xFE} }, par_temp_params;
ST_RSP_PARAMETER gst_rsp_parameter;
uint32_t param_changed = 0;

static inline uint16_t abs_diff(uint16_t a, uint16_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

// 12-bit ADC ??????????
int16_t adc_12bit_to_signed(uint16_t val) {
    if (val > 0x7FF) {  // 0x7FF = 2047(12-bit ????)
        val -= 0x1000;  // 0x1000 = 4096(????????)
    }
    return (int16_t)val;
}

void data_process_init(void)
{
    memset(&par_params, 0, sizeof(ParameterFrame));
    memset(&gst_rsp_parameter, 0, sizeof(ST_RSP_PARAMETER));

    #if 0
    par_params.strip_gate_line_left_1  = 51712;
    par_params.strip_gate_line_right_1 = 51712 + 9728;
    par_params.ws_distance_from_zero_position    = 200;
    par_params.ds_distance_from_zero_position    = 500;
    par_params.ws_zero_position  = 3401;
    par_params.ds_zero_position  = 1319;
    par_params.ws_roll_out       = 150;
    par_params.ds_roll_out       = 150;
    par_params.theta_offset      = 1409;
    par_params.filter_a          = 100;
    par_params.filter_c          = 2;
    par_params.wait_counter      = 4;
    par_params.iq0_iq1_threshold        = 120;
    par_params.theta0_theta1_threshold  = 70;
    par_params.iq_theta_threshold       = 20;
    par_params.iq_theta_long_time_count  = 3;
    par_params.max_id_number             = 15;
    par_params.emw_number                = 1;
    par_params.sin_gain                  = 60;
    par_params.cos_gain                  = 60;
    par_params.data_disposal_count       = 60000;
    par_params.data_read_count           = 40000;
    #endif

    Load_Parameters();
}

uint32_t peak_pos_old=0;
void data_process(void)
{
    check_param_update();        //可忽略，参数更新已在别处处理过了
    ParameterFrame *p_rsp_param = &par_params;

    uint16_t base_position = (ws_ds_flag == DEVICE_SIDE_DS) ? p_rsp_param->ds_base_position : p_rsp_param->ws_base_position;
    uint16_t base_phase = (ws_ds_flag == DEVICE_SIDE_DS) ? p_rsp_param->ds_theta_base : p_rsp_param->ws_theta_base;

		peak_pos_old =	peak_pos ;
		peak_pos = find_peak_middle_position( peak_pos);
	
    uint16_t cur_phase = cal_phase(peak_pos);
	
    uint16_t iq_data = distance_by_pseudocode(peak_pos);

   
    if (ws_ds_flag == DEVICE_SIDE_DS) {
        if (par_params.ds_theta_base == 0 && cur_phase != 0) {
            par_params.ds_theta_base = cur_phase;
        }
    } else {
        if (par_params.ws_theta_base == 0 && cur_phase != 0) {
            par_params.ws_theta_base = cur_phase;
        }
    }

    calibrate_base_if_needed(&base_position, &base_phase, iq_data, cur_phase);
    apply_threshold_corrections(&base_position, &base_phase, iq_data, cur_phase);

    uint16_t strip_pos = distance_by_phase(iq_data, cur_phase);
    uint16_t cur_distance = distance_filter(strip_pos);

    update_response_parameters(p_rsp_param, base_position, base_phase, cur_distance, cur_phase, strip_pos, iq_data);
}

static void check_param_update(void)
{
    if (param_changed == 1)
    {
        memcpy(&par_params, &par_temp_params, sizeof(ParameterFrame));
        param_changed = 0;
    }
}

static void calibrate_base_if_needed(uint16_t *base_pos, uint16_t *base_phase, uint16_t iq_data, uint16_t cur_phase)
{
    if (*base_pos == 0)  *base_pos = iq_data;
    if (*base_phase == 0) *base_phase = cur_phase;
}

static void reset_phase_data(void)
{
    if (ws_ds_flag == DEVICE_SIDE_DS)
    {
        gst_rsp_parameter.ds_phase_offset_sum = 0;
        par_params.ds_rotate_n_data = 0;
        gst_rsp_parameter.ds_last_phase = 0;
    }
    else
    {
        gst_rsp_parameter.ws_phase_offset_sum = 0;
        par_params.ws_rotate_n_data = 0;
        gst_rsp_parameter.ws_last_phase = 0;
    }
}

static void apply_threshold_corrections(uint16_t *base_pos, uint16_t *base_phase, uint16_t iq_data, uint16_t cur_phase)
{
    uint16_t offset;
    uint16_t strip_pos = distance_by_phase(iq_data, cur_phase);

    offset = abs_diff(iq_data, *base_pos);
    if (offset > par_params.iq0_iq1_threshold)
    {
        *base_pos = iq_data;
        *base_phase = cur_phase;
        reset_phase_data();
    }

    offset = abs_diff(strip_pos, *base_pos);
    if (offset > par_params.theta0_theta1_threshold)
    {
        *base_pos = iq_data;
        *base_phase = cur_phase;
        reset_phase_data();
    }

    offset = abs_diff(strip_pos, iq_data);
    if (offset > par_params.iq_theta_threshold)
    {
        *base_pos = iq_data;
        *base_phase = cur_phase;
        reset_phase_data();
    }
}

static void update_response_parameters(ParameterFrame *p, uint16_t base_pos, uint16_t base_phase,
                                       uint16_t cur_distance, uint16_t cur_phase, uint16_t strip_pos, uint16_t iq_data)
{
    if (ws_ds_flag == DEVICE_SIDE_DS)
    {
        p->ds_strip_edge_position = cur_distance;
        p->ds_base_position = base_pos;
        p->ds_theta_base = base_phase;
        p->ds_theta_current = cur_phase;
        p->ds_iq_data = iq_data;
        p->ds_strip_edge_line = strip_pos;
    }
    else
    {
        p->ws_strip_edge_position = cur_distance;
        p->ws_base_position = base_pos;
        p->ws_theta_base = base_phase;
        p->ws_theta_current = cur_phase;
        p->ws_iq_data = iq_data;
        p->ws_strip_edge_line = strip_pos;
    }
}



uint32_t Extract_from_Sram(uint32_t peak_index)
{
    uint32_t addr, phase_cnt;
    uint32_t start_idx, end_idx;
    uint32_t max_samples = (ws_ds_flag == DEVICE_SIDE_DS) ? MAX_SAMPLES_DS : MAX_SAMPLES_WS;

    start_idx = (peak_index >= PEAK_OFFSET) ? (peak_index - PEAK_OFFSET) : 0;
    end_idx = peak_index + PEAK_OFFSET - 1;
    if (end_idx >= max_samples) end_idx = max_samples - 1;

    phase_cnt = end_idx - start_idx + 1;
    addr = (ws_ds_flag == DEVICE_SIDE_DS) ?  (start_idx * 2) : (DS_DATA_ADDR_END + start_idx * 2) ;

    FPGA_RAM_FLAG(0);
    sram_read_u16(peak_around_sin, addr, phase_cnt);
    FPGA_RAM_FLAG(1);
    sram_read_u16(peak_around_cos, addr, phase_cnt);

    return phase_cnt;
}

int16_t cal_phase(uint32_t peak_index)
{
		uint32_t i, phase_cnt;
    int32_t phase_sum = 0;  // ??int32_t????
    int16_t phase_avr, phase_offset;
    int16_t last_phase;
    uint16_t zero_position = (ws_ds_flag == DEVICE_SIDE_DS) ? par_params.ds_zero_position : par_params.ws_zero_position;
    uint16_t base_position = (ws_ds_flag == DEVICE_SIDE_DS) ? par_params.ds_base_position : par_params.ws_base_position;
    last_phase = (ws_ds_flag == DEVICE_SIDE_DS) ? gst_rsp_parameter.ds_last_phase : gst_rsp_parameter.ws_last_phase;

    phase_cnt = Extract_from_Sram(peak_index);
    if (phase_cnt == 0) return 0;

//    for (i = 0; i < phase_cnt; i++) {
//        // 1. 12-bit ADC????
//        int16_t sin_signed = (int16_t)(peak_around_sin[i] > 0x7FF ? peak_around_sin[i] - 0x1000 : peak_around_sin[i]);
//        int16_t cos_signed = (int16_t)(peak_around_cos[i] > 0x7FF ? peak_around_cos[i] - 0x1000 : peak_around_cos[i]);

//        // 2. ??????(3.3V??)
//        float32_t sin_val = (float32_t)sin_signed * 3.3f / 4096.0f;
//        float32_t cos_val = (float32_t)cos_signed * 3.3f / 4096.0f;

//        // 3. ?????(atan2)
//        float32_t theta;
//        arm_atan2_f32(sin_val, cos_val, &theta);

//        // 4. ????[0?, 360?)???10?
//        if (theta < 0) theta += 2 * PI;
//        phase_sum += (int16_t)(theta * 180.0f / PI * 10.0f);
//    }
    for (i = 0; i < phase_cnt; i++) {
        // 1. adc转换
        int16_t sin_signed = adc_12bit_to_signed(peak_around_sin[i]);  
        int16_t cos_signed = adc_12bit_to_signed(peak_around_cos[i]);

        // 2. (3.3V??)
        float32_t sin_val = (float32_t)sin_signed * 3.3f / 4096.0f; 
        float32_t cos_val = (float32_t)cos_signed * 3.3f / 4096.0f;

        // 3. (atan2)
        float32_t theta;
        arm_atan2_f32(sin_val, cos_val, &theta);

        // 4. 区间范围[0, 360)放大10倍
        if (theta < 0) theta += 2 * PI;  // ????!
        phase_sum += (int16_t)(theta * 180.0f / PI * 10.0f);
    }
    phase_avr = (int16_t)(phase_sum / phase_cnt);

    if (last_phase == 0)
        last_phase = phase_avr;
    phase_offset = phase_avr - last_phase;

	// 环形处理，保持连续性
	if (phase_offset > 1800) phase_offset -= 3600;
	else if (phase_offset < -1800) phase_offset += 3600;

    if (ws_ds_flag == DEVICE_SIDE_DS)
    {
        gst_rsp_parameter.ds_phase_offset_sum += phase_offset;
        if (gst_rsp_parameter.ds_phase_offset_sum >= 3600) {
            par_params.ds_rotate_n_data++;
            gst_rsp_parameter.ds_phase_offset_sum -= 3600;
        } else if (gst_rsp_parameter.ds_phase_offset_sum <= -3600) {
            par_params.ds_rotate_n_data--;
            gst_rsp_parameter.ds_phase_offset_sum += 3600;
        }
        gst_rsp_parameter.ds_last_phase = phase_avr;
    }
    else
    {
        gst_rsp_parameter.ws_phase_offset_sum += phase_offset;
        if (gst_rsp_parameter.ws_phase_offset_sum >= 3600) {
            par_params.ws_rotate_n_data++;
            gst_rsp_parameter.ws_phase_offset_sum -= 3600;
        } else if (gst_rsp_parameter.ws_phase_offset_sum <= -3600) {
            par_params.ws_rotate_n_data--;
            gst_rsp_parameter.ws_phase_offset_sum += 3600;
        }
        gst_rsp_parameter.ws_last_phase = phase_avr;
    }

    return phase_avr;
}

uint16_t distance_by_pseudocode(uint32_t peak_index)
{
    uint16_t dp = 6;
    uint16_t zero_position = (ws_ds_flag == DEVICE_SIDE_DS) ? par_params.ds_zero_position : par_params.ws_zero_position;
    return dp * (peak_index - zero_position);
}

uint16_t distance_by_phase(uint16_t cur_distance, uint16_t cur_theta)
{
    uint32_t distance;
    uint16_t base_position, base_theta, rotate_n, theta_offset;

    theta_offset = par_params.theta_offset;

    if (ws_ds_flag == DEVICE_SIDE_DS)
    {
        base_position = par_params.ds_base_position;
        base_theta = par_params.ds_theta_base;
        rotate_n = par_params.ds_rotate_n_data;
    }
    else
    {
        base_position = par_params.ws_base_position;
        base_theta = par_params.ws_theta_base;
        rotate_n = par_params.ws_rotate_n_data;
    }

    distance = base_position * 10 + theta_offset * ((float)(cur_theta - base_theta) / 3600.0f + rotate_n);
    return distance / 10;
}

uint16_t distance_filter(uint16_t cur_distance)
{
    static uint16_t last_distance = 0;
    static uint16_t filter_cnt = 0;
    uint16_t distance_offset;

    if (last_distance != 0)
    {
        distance_offset = abs_diff(cur_distance, last_distance);
		if (distance_offset > par_params.filter_a)
		{
			filter_cnt++;
			if (filter_cnt > par_params.filter_c)
			{
				last_distance = cur_distance;
				filter_cnt = 0;
			}
			else
				cur_distance = last_distance;
		}
		else
		{
			last_distance = cur_distance;
			filter_cnt = 0;
		}
	}

    return cur_distance;
}

    uint32_t start_idx, end_idx;
	  uint32_t window_start = 0, window_end = 0;
static uint32_t find_peak_middle_position(uint32_t peak_index)
{
    uint32_t addr, phase_cnt;
//    uint32_t start_idx, end_idx;
//	  uint32_t window_start = 0, window_end = 0;
    uint32_t max_samples = (ws_ds_flag == DEVICE_SIDE_DS) ? MAX_SAMPLES_DS : MAX_SAMPLES_WS;
    float32_t max_amplitude = 0.0f;

    uint32_t i;
	
	
    // 左右各30?(61?)
    start_idx = (peak_index >= FIND_PEAK_MIDDLE_OFFSET) ? (peak_index - FIND_PEAK_MIDDLE_OFFSET) : 0;
    end_idx = peak_index + FIND_PEAK_MIDDLE_OFFSET;
    if (end_idx >= max_samples)
        end_idx = max_samples - 1;

    phase_cnt = end_idx - start_idx + 1;
    addr = (ws_ds_flag == DEVICE_SIDE_DS) ?(start_idx * 2) :  (DS_DATA_ADDR_END + start_idx * 2) ;

    // 读取SRAM 
    FPGA_RAM_FLAG(0);
    sram_read_u16(find_peak_middle_sin, addr, phase_cnt);
    FPGA_RAM_FLAG(1);
    sram_read_u16(find_peak_middle_cos, addr, phase_cnt);



    for (i = 0; i < phase_cnt; i++)
    {
        int16_t sin_signed = adc_12bit_to_signed(find_peak_middle_sin[i]);
        int16_t cos_signed = adc_12bit_to_signed(find_peak_middle_cos[i]);
        float32_t sin_val = (float32_t)sin_signed * 3.3f / 4096.0f;
        float32_t cos_val = (float32_t)cos_signed * 3.3f / 4096.0f;
        float32_t amplitude = sqrtf(sin_val * sin_val + cos_val * cos_val);

        if (amplitude > max_amplitude)
        {
            max_amplitude = amplitude;
            window_start = i;

        }
        else if (amplitude == max_amplitude) 
        {
            window_end = i;
        }
    }

    // 查找中间位置
    if (window_end >= window_start)
    {
        uint32_t length = window_end - window_start + 1;
        return (uint16_t)(start_idx + window_start + (length / 2));
    }

    return peak_index; 
}
