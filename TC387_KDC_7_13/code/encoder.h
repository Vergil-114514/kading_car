#ifndef _CAR_ENCODER_H_
#define _CAR_ENCODER_H_

#include <stdint.h>
#include "zf_common_headfile.h"
/*
 * 三个电机编码器的统一接口。
 *
 * 硬件连接：
 *   1. 左后轮：A/计数=P02.6，B/方向=P02.7，GPT12 TIM3，方向编码器模式；
 *   2. 右后轮：A/计数=P02.8，B/方向=P00.9，GPT12 TIM4，方向编码器模式；
 *   3. 转向轮：A=P33.7，B=P33.6，GPT12 TIM2，正交编码器模式。
 *
 * 本层直接调用 libraries/zf_driver/zf_driver_encoder 中的逐飞驱动，业务代码
 * 不再直接操作 TIM2/TIM3/TIM4。这样编码器引脚、计数累计和每圈数值都只在
 * 一个模块内维护。
 */

//#define     ENCODER_B       (TIM2_ENCODER)
//#define     ENCODER_B_A     (TIM2_ENCODER_CH1_P33_7)
//#define     ENCODER_B_B     (TIM2_ENCODER_CH2_P33_6)
//
//#define     ENCODER_BL       (TIM3_ENCODER)
//#define     ENCODER_BL_A     (TIM3_ENCODER_CH1_P02_6)
//#define     ENCODER_BL_B     (TIM3_ENCODER_CH2_P02_7)
//
//#define     ENCODER_BR       (TIM4_ENCODER)
//#define     ENCODER_BR_A     (TIM4_ENCODER_CH1_P02_8)
//#define     ENCODER_BR_B     (TIM4_ENCODER_CH2_P00_9)

/* 只补充外部调用所需声明，不改变原有 count/car_speed 主体。 */
extern int16 count_num[3];
/* Linear speeds in m/s, calculated from each 5 ms count increment. */
extern float car_speed[3];
extern float car_angle;

void QdInit(void);
void GetSpeed(void);
void GetCount(void);

#endif
