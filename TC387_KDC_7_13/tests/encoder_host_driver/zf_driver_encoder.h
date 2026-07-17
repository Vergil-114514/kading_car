#ifndef _ENCODER_HOST_ZF_DRIVER_ENCODER_H_
#define _ENCODER_HOST_ZF_DRIVER_ENCODER_H_

#include <stdint.h>

/* Minimal host-only copy of the public types used by code/encoder.c. */
typedef int16_t int16;

typedef enum
{
    TIM2_ENCODER_CH1_P00_7,
    TIM2_ENCODER_CH1_P33_7,
    TIM3_ENCODER_CH1_P02_6,
    TIM4_ENCODER_CH1_P02_8,
    TIM5_ENCODER_CH1_P21_7,
    TIM5_ENCODER_CH1_P10_3,
    TIM6_ENCODER_CH1_P20_3,
    TIM6_ENCODER_CH1_P10_2
} encoder_channel1_enum;

typedef enum
{
    TIM2_ENCODER_CH2_P00_8,
    TIM2_ENCODER_CH2_P33_6,
    TIM3_ENCODER_CH2_P02_7,
    TIM4_ENCODER_CH2_P00_9,
    TIM4_ENCODER_CH2_P33_5,
    TIM5_ENCODER_CH2_P21_6,
    TIM5_ENCODER_CH2_P10_1,
    TIM6_ENCODER_CH2_P20_0
} encoder_channel2_enum;

typedef enum
{
    TIM2_ENCODER,
    TIM3_ENCODER,
    TIM4_ENCODER,
    TIM5_ENCODER,
    TIM6_ENCODER
} encoder_index_enum;

int16 encoder_get_count(encoder_index_enum encoder_n);
void encoder_clear_count(encoder_index_enum encoder_n);
void encoder_quad_init(encoder_index_enum encoder_n,
                       encoder_channel1_enum channel1,
                       encoder_channel2_enum channel2);
void encoder_dir_init(encoder_index_enum encoder_n,
                      encoder_channel1_enum channel1,
                      encoder_channel2_enum channel2);

#endif
