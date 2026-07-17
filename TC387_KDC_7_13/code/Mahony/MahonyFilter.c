#include "zf_common_headfile.h"

#define MAHONY_DEG_TO_RAD       (0.01745329251994329577f)
#define MAHONY_MIN_NORM_SQUARED (0.000001f)

void MahonyFilter_Init(MahonyFilter_t *filter, float kp, float ki)
{
    if(filter == NULL)
    {
        return;
    }

    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;
    filter->integral_x = 0.0f;
    filter->integral_y = 0.0f;
    filter->integral_z = 0.0f;
    filter->two_kp = 2.0f * kp;
    filter->two_ki = 2.0f * ki;
}

void MahonyFilter_UpdateIMU(MahonyFilter_t *filter,
                            float gyro_x_dps,
                            float gyro_y_dps,
                            float gyro_z_dps,
                            float acc_x,
                            float acc_y,
                            float acc_z,
                            float dt_s)
{
    float gx;
    float gy;
    float gz;
    float norm_squared;
    float reciprocal_norm;
    float half_vx;
    float half_vy;
    float half_vz;
    float half_ex;
    float half_ey;
    float half_ez;
    float qa;
    float qb;
    float qc;

    if((filter == NULL) || !(dt_s > 0.0f))
    {
        return;
    }

    gx = gyro_x_dps * MAHONY_DEG_TO_RAD;
    gy = gyro_y_dps * MAHONY_DEG_TO_RAD;
    gz = gyro_z_dps * MAHONY_DEG_TO_RAD;

    /* 加速度向量只用于估计重力方向，先归一化以消除量纲。 */
    norm_squared = acc_x * acc_x + acc_y * acc_y + acc_z * acc_z;
    if(norm_squared > MAHONY_MIN_NORM_SQUARED)
    {
        reciprocal_norm = 1.0f / sqrtf(norm_squared);
        acc_x *= reciprocal_norm;
        acc_y *= reciprocal_norm;
        acc_z *= reciprocal_norm;

        /* 由当前四元数计算机体系中的估计重力方向。 */
        half_vx = filter->q1 * filter->q3 - filter->q0 * filter->q2;
        half_vy = filter->q0 * filter->q1 + filter->q2 * filter->q3;
        half_vz = filter->q0 * filter->q0 - 0.5f +
                  filter->q3 * filter->q3;

        /* 实测重力与估计重力的叉积就是姿态反馈误差。 */
        half_ex = acc_y * half_vz - acc_z * half_vy;
        half_ey = acc_z * half_vx - acc_x * half_vz;
        half_ez = acc_x * half_vy - acc_y * half_vx;

        if(filter->two_ki > 0.0f)
        {
            filter->integral_x += filter->two_ki * half_ex * dt_s;
            filter->integral_y += filter->two_ki * half_ey * dt_s;
            filter->integral_z += filter->two_ki * half_ez * dt_s;
            gx += filter->integral_x;
            gy += filter->integral_y;
            gz += filter->integral_z;
        }
        else
        {
            /* Ki 关闭时清零积分项，避免以后重新启用时发生积分突变。 */
            filter->integral_x = 0.0f;
            filter->integral_y = 0.0f;
            filter->integral_z = 0.0f;
        }

        gx += filter->two_kp * half_ex;
        gy += filter->two_kp * half_ey;
        gz += filter->two_kp * half_ez;
    }

    /* q_dot = 0.5 * q * gyro，随后按本次采样周期积分。 */
    gx *= 0.5f * dt_s;
    gy *= 0.5f * dt_s;
    gz *= 0.5f * dt_s;
    qa = filter->q0;
    qb = filter->q1;
    qc = filter->q2;
    filter->q0 += -qb * gx - qc * gy - filter->q3 * gz;
    filter->q1 +=  qa * gx + qc * gz - filter->q3 * gy;
    filter->q2 +=  qa * gy - qb * gz + filter->q3 * gx;
    filter->q3 +=  qa * gz + qb * gy - qc * gx;

    norm_squared = filter->q0 * filter->q0 + filter->q1 * filter->q1 +
                   filter->q2 * filter->q2 + filter->q3 * filter->q3;
    if(norm_squared > MAHONY_MIN_NORM_SQUARED)
    {
        reciprocal_norm = 1.0f / sqrtf(norm_squared);
        filter->q0 *= reciprocal_norm;
        filter->q1 *= reciprocal_norm;
        filter->q2 *= reciprocal_norm;
        filter->q3 *= reciprocal_norm;
    }
}
