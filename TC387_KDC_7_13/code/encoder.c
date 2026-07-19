#include "zf_common_headfile.h"
#include "ackermann_control.h"


#define     ENCODER_B       (TIM2_ENCODER)
#define     ENCODER_B_A     (TIM2_ENCODER_CH1_P33_7)
#define     ENCODER_B_B     (TIM2_ENCODER_CH2_P33_6)

#define     ENCODER_BL       (TIM3_ENCODER)
#define     ENCODER_BL_A     (TIM3_ENCODER_CH1_P02_6)
#define     ENCODER_BL_B     (TIM3_ENCODER_CH2_P02_7)

#define     ENCODER_BR       (TIM4_ENCODER)
#define     ENCODER_BR_A     (TIM4_ENCODER_CH1_P02_8)
#define     ENCODER_BR_B     (TIM4_ENCODER_CH2_P00_9)


int16 count_num[3]={0};
float  car_speed[3]={0};
float  car_angle   = 0 ;
float ENCODER_B_COUNTS_PER_METER   =    (-3342.0f); /* 6684 counts over 2 m. */
float ENCODER_BL_BR_COUNTS_PER_METER =  (-2565.0f);/* 5130 counts over 2 m. */
#define ENCODER_REPORT_SAMPLE_COUNT       (20U)     /* 20 x 5 ms = 100 ms. */

static int32_t encoder_report_count_sum[3] = {0};
static volatile float encoder_report_speed_mps[3] = {0.0f};
static uint8_t encoder_report_sample_count = 0U;

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
    count_num[0] = encoder_get_count(ENCODER_B);           //获取编码器数值
    count_num[1] = encoder_get_count(ENCODER_BL);           //获取编码器数值
    count_num[2] = encoder_get_count(ENCODER_BR);           //获取右后轮编码器数值
    encoder_clear_count(ENCODER_B);                    //编码器清零
    encoder_clear_count(ENCODER_BL);                   //编码器清零
    encoder_clear_count(ENCODER_BR);                   //右后轮编码器清零

    car_angle = ((float)(oid_encoder_get_angle()-oid_encoder_zero))/oid_encoder;

    /* Convert each 5 ms count increment to linear speed in m/s. */
    car_speed[0] = (float)count_num[0]*-0.0598444045481747f;
    car_speed[1] = (float)count_num[1]*-0.0779727095516569f;
    car_speed[2] = (float)count_num[2]*0.0779727095516569f;

    encoder_report_count_sum[0] += count_num[0];
    encoder_report_count_sum[1] += count_num[1];
    encoder_report_count_sum[2] += count_num[2];
    ++encoder_report_sample_count;

    if(encoder_report_sample_count >= ENCODER_REPORT_SAMPLE_COUNT)
    {
        encoder_report_speed_mps[0] =
            (float)encoder_report_count_sum[0]
            * -0.0598444045481747f
            / (float)ENCODER_REPORT_SAMPLE_COUNT;
        encoder_report_speed_mps[1] =
            (float)encoder_report_count_sum[1]
            * -0.0779727095516569f
            / (float)ENCODER_REPORT_SAMPLE_COUNT;
        encoder_report_speed_mps[2] =
            (float)encoder_report_count_sum[2]
            * 0.0779727095516569f
            / (float)ENCODER_REPORT_SAMPLE_COUNT;
        encoder_report_count_sum[0] = 0;
        encoder_report_count_sum[1] = 0;
        encoder_report_count_sum[2] = 0;
        encoder_report_sample_count = 0U;
    }
}
void GetCount(void)
{
//    count[0] = encoder_get_count(ENCODER_B);           //获取编码器数值
//
//    count[1] = encoder_get_count(ENCODER_BL);           //获取编码器数值
//
//    count[2] = encoder_get_count(ENCODER_BR);           //获取右后轮编码器数值
//
//    car_angle = ((float)(oid_encoder_get_angle()-oid_encoder_zero))/oid_encoder;
//
//    printf("%5d,%5d,%5d,%5d\r\n",
//           count[0], count[1], count[2],
//           oid_encoder_get_angle());
//    printf("%6.2f\r\n",car_angle);

    /* CPU0 main task owns debug output; CPU2 only publishes the snapshot. */
    printf("%.3f,%.3f,%.3f\r\n",
           encoder_report_speed_mps[0],
           encoder_report_speed_mps[1],
           encoder_report_speed_mps[2]);
}
