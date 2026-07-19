#ifndef ENCODER_HOST_STUBS_H
#define ENCODER_HOST_STUBS_H

#include <stdint.h>
#include <stdio.h>

#define _zf_common_headfile_h_
#define _ACKERMANN_CONTROL_H_

typedef int16_t int16;
typedef uint16_t uint16;

typedef enum
{
    TIM2_ENCODER_CH1_P33_7 = 0,
    TIM3_ENCODER_CH1_P02_6,
    TIM4_ENCODER_CH1_P02_8
} encoder_channel1_enum;

typedef enum
{
    TIM2_ENCODER_CH2_P33_6 = 0,
    TIM3_ENCODER_CH2_P02_7,
    TIM4_ENCODER_CH2_P00_9
} encoder_channel2_enum;

typedef enum
{
    TIM2_ENCODER = 0,
    TIM3_ENCODER,
    TIM4_ENCODER,
    TIM5_ENCODER,
    TIM6_ENCODER
} encoder_index_enum;

int16 encoder_get_count(encoder_index_enum encoder_n);
void encoder_clear_count(encoder_index_enum encoder_n);
void encoder_dir_init(encoder_index_enum encoder_n,
                      encoder_channel1_enum channel1,
                      encoder_channel2_enum channel2);
void encoder_quad_init(encoder_index_enum encoder_n,
                       encoder_channel1_enum channel1,
                       encoder_channel2_enum channel2);
void oid_encoder_init(void);
uint16 oid_encoder_get_angle(void);

#endif
