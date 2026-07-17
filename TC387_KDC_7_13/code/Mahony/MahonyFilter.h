#ifndef _MAHONY_FILTER_H_
#define _MAHONY_FILTER_H_

/**
 * @file MahonyFilter.h
 * @brief 无磁力计 Mahony 6DOF 姿态滤波器的独立状态接口。
 *
 * 旧版 imu_N.c 把传感器读取、零偏标定、蜂鸣器和姿态算法耦合在一起，
 * 不适合与 Fusion/GPS_INS 共用同一套 IMU。这里仅保留纯算法，传感器读取
 * 和零偏标定统一由 TopSpeed_GPS_INS_Port 完成。
 */

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
    float integral_x;
    float integral_y;
    float integral_z;
    float two_kp;
    float two_ki;
} MahonyFilter_t;

/** 初始化单位四元数和 PI 反馈参数。kp、ki 使用 Mahony 原论文常用定义。 */
void MahonyFilter_Init(MahonyFilter_t *filter, float kp, float ki);

/**
 * 输入一次陀螺仪和加速度计样本。
 * gyro 单位为 deg/s，accel 单位可为 g 或 m/s^2，只要求三个轴单位一致。
 */
void MahonyFilter_UpdateIMU(MahonyFilter_t *filter,
                            float gyro_x_dps,
                            float gyro_y_dps,
                            float gyro_z_dps,
                            float acc_x,
                            float acc_y,
                            float acc_z,
                            float dt_s);

#endif
