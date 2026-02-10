#include <math.h>
#include <stdio.h>
#include <string.h>
#include "data_process.h"
#include "flash.h"
#include "com_process.h"
#include "arm_math.h"
#include "fmc.h"
#include "tim.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include "semphr.h"

#define Test_sram 0

#define MAX_SAMPLES_WS 20000
#define MAX_SAMPLES_DS 20000
#define PHASE_UNIT 10
#define IQ_DISTANCE_SCALE 0.6f
#define THETA_TO_MM_RATIO 14.02f

uint8_t iq0_iq1_over = 0;
uint8_t theta0_theta1_over = 0;
uint8_t iq_theta_over = 0;


uint16_t iq_data_old = 0;
uint16_t strip_pos_old = 0;
uint16_t phase_old = 0;
uint8_t base_update_pending = 0;
uint16_t update_timer_flag = 1;
int16_t amplitude[PEAK_OFFSET * 2 + 1];
uint16_t para_wait_counter = 0;

uint16_t peak_around_sin[10], peak_around_cos[10];

ParameterFrame par_params = {.header = {0x01, 0x01, 0xFE}}, par_temp_params;
ST_RSP_PARAMETER gst_rsp_parameter;
uint32_t param_changed = 0;

static inline uint16_t abs_diff(uint16_t a, uint16_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

// 12-bit ADC 有符号转换
int16_t adc_12bit_to_signed(uint16_t val)
{
    if (val > 0x7FF)
    {                  // 0x7FF = 2047
        val -= 0x1000; // 0x1000 = 4096
    }
    return (int16_t)val;
}

void data_process_init(void)
{
    memset(&par_params, 0, sizeof(ParameterFrame));
    memset(&gst_rsp_parameter, 0, sizeof(ST_RSP_PARAMETER));

    Load_Parameters();
    if (par_params.sin_gain == 0)
    { // 新板子参数为0
        memset(&par_params, 0, sizeof(ParameterFrame));
        par_params.strip_gate_line_left_1 = 0;
        par_params.strip_gate_line_right_1 = 6000;
        par_params.strip_gate_line_left_2 = 0;
        par_params.strip_gate_line_right_2 = 6000; // 是否重复
        par_params.ws_distance_from_zero_position = 5000;
        par_params.ds_distance_from_zero_position = 5000;
        par_params.ws_zero_position = 2272;
        par_params.ds_zero_position = 2272;
        par_params.ws_roll_out = 1500;
        par_params.ds_roll_out = 1500;
        par_params.theta_offset = 1409;
        par_params.filter_a = 1000;
        par_params.filter_c = 10;
        par_params.wait_counter = 2;
        par_params.iq0_iq1_threshold = 120;
        par_params.theta0_theta1_threshold = 120;
        par_params.iq_theta_threshold = 120;
        par_params.iq_theta_long_time_count = 3;
        par_params.iq_theta_long_time_count2 = 3; // count和count2是否重复
        par_params.max_id_number = 15;
        par_params.emw_number = 1;
        par_params.sin_gain = 70;
        par_params.cos_gain = 70;
        par_params.data_disposal_count = 30000;
        par_params.data_read_count = 40000;
        par_params.dirt_warning_level = 200;
        par_params.dirt_detect_average = 100;
        par_params.dirt_detect_sampling = 10;
    }
}

uint32_t peak_pos_old = 0;
uint8_t first_run = 1; // 标记首次运行

void data_process(void)
{
    check_param_update();
    ParameterFrame *p_rsp_param = &par_params;



    // 计算当前相位和 IQ 数据
    uint16_t cur_phase = cal_phase(peak_pos);
    uint16_t iq_data = distance_by_pseudocode(peak_pos);

    // 定义基准位置和基准相位
    uint16_t base_position, base_phase;

    // 首次运行：基于 peak_pos 初始化基准值并写入 Flash

    if (first_run)
    {
        
        if(xSemaphoreTake(paramMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            if (ws_ds_flag == DEVICE_SIDE_DS)
            {
                if (iq_data != 0)
                {
                    p_rsp_param->ds_base_position = iq_data; // 初始化基准位置
//                    printf("[INIT] DS base_position set to %d\n", iq_data);
                }
                if (cur_phase != 0)
                {
                    p_rsp_param->ds_theta_base = cur_phase; // 初始化基准相位
//                    printf("[INIT] DS theta_base set to %d\n", cur_phase);
                }
                base_position = p_rsp_param->ds_base_position;
                base_phase = p_rsp_param->ds_theta_base;
            }
            else
            {
                if (iq_data != 0)
                {
                    p_rsp_param->ws_base_position = iq_data; // 初始化基准位置
//                    printf("[INIT] WS base_position set to %d\n", iq_data);
                }
                if (cur_phase != 0)
                {
                    p_rsp_param->ws_theta_base = cur_phase; // 初始化基准相位
//                    printf("[INIT] WS theta_base set to %d\n", cur_phase);
                }
                base_position = p_rsp_param->ws_base_position;
                base_phase = p_rsp_param->ws_theta_base;
            }

            first_run = 0; // 标记首次运行已完成
            para_wait_counter = par_params.wait_counter;
            par_params.board_temperature = 0;
            par_params.ds_antenna_temperature = 0;
            par_params.ws_antenna_temperature = 0;
            par_params.panel_temperature = 0;
            par_params.error = 0;
            
            xSemaphoreGive(paramMutexHandle);
        }
    }
    else
    {
        // 非首次运行，从 par_params 获取基准值
        base_position = (ws_ds_flag == DEVICE_SIDE_DS) ? p_rsp_param->ds_base_position : p_rsp_param->ws_base_position;
        base_phase = (ws_ds_flag == DEVICE_SIDE_DS) ? p_rsp_param->ds_theta_base : p_rsp_param->ws_theta_base;
    }

    // 校正
    apply_threshold_corrections(&base_position, &base_phase, iq_data, cur_phase);

    // 更新 par_params 中的基准值（如果被校正修改）
    if(xSemaphoreTake(paramMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        if (ws_ds_flag == DEVICE_SIDE_DS)
        {
            if (p_rsp_param->ds_base_position != base_position || p_rsp_param->ds_theta_base != base_phase)
            {
                p_rsp_param->ds_base_position = base_position;
                p_rsp_param->ds_theta_base = base_phase;
            }
        }
        else
        {
            if (p_rsp_param->ws_base_position != base_position || p_rsp_param->ws_theta_base != base_phase)
            {
                p_rsp_param->ws_base_position = base_position;
                p_rsp_param->ws_theta_base = base_phase;
            }
        }
        xSemaphoreGive(paramMutexHandle);
    }

    // 计算带钢位置
    uint16_t strip_pos = distance_by_phase(iq_data, cur_phase);
    if(xSemaphoreTake(paramMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        if (ws_ds_flag == DEVICE_SIDE_DS)
        {
            if (strip_pos < par_params.ds_roll_out)
            {
                par_params.error = par_params.error | (1 << 13); // ds_roll_out
            }
            else
            {
                par_params.error &= ~(1 << 13);
            }
        }
        else
        {
            if (strip_pos < par_params.ws_roll_out)
            {
                par_params.error = par_params.error | (1 << 10); // ws_roll_out
            }
            else
            {
                par_params.error &= ~(1 << 10);
            }
        }
        xSemaphoreGive(paramMutexHandle);
    }
    // 应用滤波
    uint16_t cur_distance = distance_filter(strip_pos);

    // 更新响应参数
    update_response_parameters(p_rsp_param, base_position, base_phase, cur_distance, cur_phase, strip_pos, iq_data);
    iq_data_old = iq_data;
    strip_pos_old = cur_distance;
    phase_old = cur_phase;
}

/**
** update可以去掉，在sram中做了比较更新
**/
static void check_param_update(void)
{
    if (param_changed == 1)
    {
        memcpy(&par_params, &par_temp_params, sizeof(ParameterFrame));
        param_changed = 0;
    }
}

uint16_t over_count_times = 0;
void reset_phase_data(void)
{
    // 获取参数互斥量，保护对par_params的修改
    if(xSemaphoreTake(paramMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
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
        xSemaphoreGive(paramMutexHandle);
    }
//    printf("[RESET] rotate_n + phase offset cleared.\n");
}

void apply_threshold_corrections(uint16_t *base_pos, uint16_t *base_phase, uint16_t iq_data, uint16_t cur_phase)
{
    uint16_t offset1, offset2, offset3;
    uint16_t strip_pos = distance_by_phase(iq_data, cur_phase);
    base_update_pending = 0;

    // 1. 计算两项误差
    // offset1 = abs_diff(iq_data, *base_pos);
    offset1 = abs_diff(iq_data, iq_data_old); // 与上一个扫描周期的数据进行比较
    iq0_iq1_over = (offset1 > par_params.iq0_iq1_threshold);

    offset2 = abs_diff(strip_pos, iq_data);
    iq_theta_over = (offset2 > par_params.iq_theta_threshold);

    offset3 = abs_diff(strip_pos, strip_pos_old);
    theta0_theta1_over = (offset3 > par_params.theta0_theta1_threshold);

    if (par_params.iq_theta_long_time_count == 0)
    {
        uint8_t over_count = iq0_iq1_over + theta0_theta1_over + iq_theta_over;

        if (over_count >= 2)
        {
            over_count_times++;
        }
        
        if (over_count_times >= 3)
        {
            if (base_update_pending == 0)
            {
                base_update_pending = 1;
                if (update_timer_flag)
                {
                    __HAL_TIM_SET_COUNTER(&htim6, 0);
                    HAL_TIM_Base_Start_IT(&htim6); // 启动延时计时
                    update_timer_flag = 0;
                }
            }

            if (para_wait_flag)
            {
                *base_pos = iq_data;
                *base_phase = cur_phase;

                reset_phase_data();
                over_count_times = 0;
                base_update_pending = 0;
                para_wait_flag = 0;
                update_timer_flag = 1;
                HAL_TIM_Base_Stop_IT(&htim6);
                para_wait_counter = par_params.wait_counter;
            }
        }
        else
        {
            
            if (base_update_pending)
            {
                base_update_pending = 0;
                para_wait_flag = 0;
                HAL_TIM_Base_Stop_IT(&htim6);
            }
        }
    }
    else
    {
        uint8_t over_count1 = iq0_iq1_over | iq_theta_over;

        if (over_count1 != 0)
        {
            over_count_times++;
        }
        
        if (over_count_times == par_params.wait_counter)
        {
            if (base_update_pending == 0)
            {
                base_update_pending = 1;
            }
            *base_pos = iq_data;
            *base_phase = cur_phase;

            reset_phase_data();

            over_count_times = 0;
            base_update_pending = 0;
        }
    }
}

static void update_response_parameters(ParameterFrame *p, uint16_t base_pos, uint16_t base_phase, uint16_t cur_distance, uint16_t cur_phase, uint16_t strip_pos, uint16_t iq_data)
{
    
    if(xSemaphoreTake(paramMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        if (ws_ds_flag == DEVICE_SIDE_DS)
        {
            p->ds_strip_edge_position = cur_distance;
            p->ds_base_position = base_pos;
            p->ds_theta_base = base_phase;
            p->ds_theta_current = cur_phase;
            p->ds_iq_data = iq_data;
            p->ds_strip_edge_line = peak_pos;
        }
        else
        {
            p->ws_strip_edge_position = cur_distance;
            p->ws_base_position = base_pos;
            p->ws_theta_base = base_phase;
            p->ws_theta_current = cur_phase;
            p->ws_iq_data = iq_data;
            p->ws_strip_edge_line = peak_pos;
        }
        xSemaphoreGive(paramMutexHandle);
    }
}

uint32_t Extract_from_Sram(uint32_t peak_index)
{
    uint32_t addr, phase_cnt;
    uint32_t start_idx, end_idx;
    uint32_t max_samples = (ws_ds_flag == DEVICE_SIDE_DS) ? MAX_SAMPLES_DS : MAX_SAMPLES_WS;

    start_idx = (peak_index >= PEAK_OFFSET) ? (peak_index - PEAK_OFFSET) : 0;
    end_idx = peak_index + PEAK_OFFSET - 1;
    if (end_idx >= max_samples)
        end_idx = max_samples - 1;

    phase_cnt = end_idx - start_idx + 1;
    addr = (ws_ds_flag == DEVICE_SIDE_DS) ? (start_idx * 2) : (DS_DATA_ADDR_END + start_idx * 2);

    FPGA_RAM_FLAG(0);
    sram_read_u16(peak_around_sin, addr, phase_cnt);
    FPGA_RAM_FLAG(1);
    sram_read_u16(peak_around_cos, addr, phase_cnt);

    return phase_cnt;
}

int16_t cal_phase(uint32_t peak_index)
{
    uint32_t i, phase_cnt;
    int32_t phase_sum = 0;
    int16_t phase_avr_01deg, phase_offset_01deg; // 0.1° 精度
    int16_t phase_avr_1deg, phase_offset_1deg;   // 1° 精度
    int16_t last_phase_01deg;
    uint16_t zero_position = (ws_ds_flag == DEVICE_SIDE_DS) ? par_params.ds_zero_position : par_params.ws_zero_position;
    uint16_t base_position = (ws_ds_flag == DEVICE_SIDE_DS) ? par_params.ds_base_position : par_params.ws_base_position;
    last_phase_01deg = (ws_ds_flag == DEVICE_SIDE_DS) ? gst_rsp_parameter.ds_last_phase : gst_rsp_parameter.ws_last_phase;

    phase_cnt = Extract_from_Sram(peak_index);
    if (phase_cnt == 0)
        return 0;

    for (i = 0; i < phase_cnt; i++)
    {
        int16_t sin_signed = adc_12bit_to_signed(peak_around_sin[i]);
        int16_t cos_signed = adc_12bit_to_signed(peak_around_cos[i]);

        float32_t sin_val = (float32_t)sin_signed * 3.3f / 4096.0f;
        float32_t cos_val = (float32_t)cos_signed * 3.3f / 4096.0f;

        float32_t theta;
        arm_atan2_f32(sin_val, cos_val, &theta);

        if (theta < 0)
            theta += 2 * PI;

        // 保持0.1° 精度
        phase_sum += (int16_t)(theta * 180.0f / PI * 10.0f);
    }

    // 平均相位（0.1°）
    phase_avr_01deg = (int16_t)(phase_sum / phase_cnt);

    if (last_phase_01deg == 0)
        last_phase_01deg = phase_avr_01deg;

    phase_offset_01deg = wrap_phase_diff(phase_avr_01deg, last_phase_01deg);

    if (ws_ds_flag == DEVICE_SIDE_DS)
    {
        gst_rsp_parameter.ds_phase_offset_sum += phase_offset_01deg;
        if (gst_rsp_parameter.ds_phase_offset_sum >= 3600) // 360.0°
        {
            par_params.ds_rotate_n_data++;
            gst_rsp_parameter.ds_phase_offset_sum -= 3600;
        }
        else if (gst_rsp_parameter.ds_phase_offset_sum <= -3600)
        {
            par_params.ds_rotate_n_data--;
            gst_rsp_parameter.ds_phase_offset_sum += 3600;
        }
        gst_rsp_parameter.ds_last_phase = phase_avr_01deg;

        
        phase_avr_1deg = phase_avr_01deg / 10;
        phase_offset_1deg = phase_offset_01deg / 10;


    }
    else
    {
        gst_rsp_parameter.ws_phase_offset_sum += phase_offset_01deg;
        if (gst_rsp_parameter.ws_phase_offset_sum >= 3600)
        {
            par_params.ws_rotate_n_data++;
            gst_rsp_parameter.ws_phase_offset_sum -= 3600;
        }
        else if (gst_rsp_parameter.ws_phase_offset_sum <= -3600)
        {
            par_params.ws_rotate_n_data--;
            gst_rsp_parameter.ws_phase_offset_sum += 3600;
        }
        gst_rsp_parameter.ws_last_phase = phase_avr_01deg;

        phase_avr_1deg = phase_avr_01deg / 10;
        phase_offset_1deg = phase_offset_01deg / 10;


    }

    // 返回1°精度
    return phase_avr_01deg / 10;
}

uint16_t distance_by_pseudocode(uint32_t peak_index)
{
    uint16_t dp = 6;
    uint16_t zero_position = (ws_ds_flag == DEVICE_SIDE_DS) ? par_params.ds_zero_position : par_params.ws_zero_position;
    return dp * (peak_index - zero_position);
}

uint16_t distance_by_phase(uint16_t cur_distance, uint16_t cur_theta)
{
    float distance;
    uint16_t base_position, base_theta, theta_offset;
    int16_t rotate_n;
    int16_t last_phase;
    int16_t phase_sum;

    theta_offset = par_params.theta_offset;

    if (ws_ds_flag == DEVICE_SIDE_DS)
    {
        base_position = par_params.ds_base_position;
        base_theta = par_params.ds_theta_base;
        rotate_n = par_params.ds_rotate_n_data;
        last_phase = gst_rsp_parameter.ds_last_phase;
        phase_sum = gst_rsp_parameter.ds_phase_offset_sum;
    }
    else
    {
        base_position = par_params.ws_base_position;
        base_theta = par_params.ws_theta_base;
        rotate_n = par_params.ws_rotate_n_data;
        last_phase = gst_rsp_parameter.ws_last_phase;
        phase_sum = gst_rsp_parameter.ws_phase_offset_sum;
    }

    if (par_params.iq_theta_long_time_count == 2)
    {

        uint16_t iq_data = distance_by_pseudocode(peak_pos);
        int best_n = (iq_data - iq_data_old) / theta_offset; // 向零取整
        // 更新圈数计数器
        if (best_n != 0)
        {
            // TODO
            distance = base_position * 10.0f +
                       theta_offset * ((float)phase_sum / 3600.0f + best_n);
            return (int16_t)(distance / 10);
        }
        else
        {
            distance = base_position * 10.0f +
                       theta_offset * ((float)phase_sum / 3600.0f + rotate_n);
            return (int16_t)(distance / 10);
        }

    }
    else
    {
        distance = base_position * 10.0f +
                   theta_offset * ((float)phase_sum / 3600.0f + rotate_n);
        return (int16_t)(distance / 10);
    }
}

uint16_t distance_filter(uint16_t cur_distance)
{
    static uint16_t last_distance = 0;
    static uint16_t filter_cnt = 0;
    uint16_t distance_offset;

    if (last_distance == 0)
    {
        last_distance = cur_distance;
        return cur_distance;
    }

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
        {
            cur_distance = last_distance;
        }
    }
    else
    {
        last_distance = cur_distance;
        filter_cnt = 0;
    }

    return cur_distance;
}

int16_t wrap_phase_diff(int16_t cur, int16_t base)
{
    int32_t diff = (int32_t)cur - base; 
    diff = (diff % 3600 + 3600) % 3600; 
    if (diff > 1800)
        diff -= 3600; 
    return (int16_t)diff;
}

/**
 * 改进版相位差计算，自动判断方向
 * cur      : 当前相位
 * base     : 基准相位
 * last_cur : 上一次相位
 */
int16_t compute_phase_diff(int16_t cur, int16_t base, int16_t last_cur)
{
    // 先算最短差值
    int16_t diff = cur - base;
    if (diff > 1800)
        diff -= 3600;
    else if (diff < -1800)
        diff += 3600;

    // 推断方向：cur 相对于 last_cur 的变化
    int16_t delta = cur - last_cur;
    if (delta > 1800) 
        delta -= 3600;
    else if (delta < -1800) 
        delta += 3600;

    // 根据运动方向修正差值
    if (delta > 0 && diff < 0)
        diff += 3600; 
    else if (delta < 0 && diff > 0)
        diff -= 3600; 

    return diff;
}
