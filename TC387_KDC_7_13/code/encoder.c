#include "zf_common_headfile.h"


#define     ENCODER_B       (TIM2_ENCODER)
#define     ENCODER_B_A     (TIM2_ENCODER_CH1_P33_7)
#define     ENCODER_B_B     (TIM2_ENCODER_CH2_P33_6)

#define     ENCODER_BL       (TIM3_ENCODER)
#define     ENCODER_BL_A     (TIM3_ENCODER_CH1_P02_6)
#define     ENCODER_BL_B     (TIM3_ENCODER_CH2_P02_7)

#define     ENCODER_BR       (TIM4_ENCODER)
#define     ENCODER_BR_A     (TIM4_ENCODER_CH1_P02_8)
#define     ENCODER_BR_B     (TIM4_ENCODER_CH2_P00_9)


int16 count[3]={0};
float  car_speed[3]={0};
float  car_angle   = 0 ;
#define encoder_back      (3413.33f)
#define encoder_back_lr   (1.0f)

#define oid_encoder_zero  (0)
#define oid_encoder       (4096.0f/360.0f)

void QdInit(void)
{
    encoder_quad_init(ENCODER_B, ENCODER_B_A, ENCODER_B_B);
    encoder_dir_init(ENCODER_BL, ENCODER_BL_A, ENCODER_BL_B);
    encoder_dir_init(ENCODER_BR, ENCODER_BR_A, ENCODER_BR_B);                      // 初始化编码器模块与引脚 方向解码编码器模式
    //encoder_dir_init(ENCODER2_TIM, ENCODER2_PLUS, ENCODER2_DIR);                // 初始化编码器采值引脚及定时器
    oid_encoder_init();
}

void GetSpeed(void)
{
    count[0] = encoder_get_count(ENCODER_B);           //获取编码器数值
    encoder_clear_count(ENCODER_B);                 //编码器清零

    count[1] = encoder_get_count(ENCODER_BL);           //获取编码器数值
    encoder_clear_count(ENCODER_BL);                 //编码器清零

    count[2] = encoder_get_count(ENCODER_BR);           //获取右后轮编码器数值
    encoder_clear_count(ENCODER_BR);                 //右后轮编码器清零

    car_angle = ((float)(oid_encoder_get_angle()-oid_encoder_zero))/oid_encoder;

    car_speed[0] = (float)count[0]/encoder_back;
    car_speed[1] = (float)count[1]/encoder_back_lr;
    car_speed[2] = (float)count[2]/encoder_back_lr;

}
void GetCount(void)
{
    count[0] = encoder_get_count(ENCODER_B);           //获取编码器数值

    count[1] = encoder_get_count(ENCODER_BL);           //获取编码器数值

    count[2] = encoder_get_count(ENCODER_BR);           //获取右后轮编码器数值

    car_angle = ((float)(oid_encoder_get_angle()-oid_encoder_zero))/oid_encoder;

    printf("%5d,%5d,%5d,%5d\r\n",
           count[0], count[1], count[2],
           oid_encoder_get_angle());
    printf("%6.2f\r\n",car_angle);
}
