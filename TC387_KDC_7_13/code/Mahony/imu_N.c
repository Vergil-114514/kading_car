#include "zf_common_headfile.h"

float twoKi;        // 2 * integral gain (Ki)
float q0, q1, q2, q3;   // quaternion of sensor frame relative to auxiliary frame
float integralFBx, integralFBy, integralFBz;  // integral error terms scaled by Ki
float invSampleFreq;
float roll_mahony, pitch_mahony, yaw_mahony,world_yaw_mahony;
uint8 anglesComputed;

/*
 * 旧接口保留的航向安装偏置。原文件依赖工程外部同名全局变量，导致 Mahony
 * 目录一旦加入构建就无法编译；默认置零后，旧 world/car_getYaw 接口仍可用。
 * 新的三算法选择路径使用 MahonyFilter.c，不依赖这些旧全局状态。
 */
static float imu_start_err = 0.0f;
static float car_head_err = 0.0f;
//*-*-*-**-----------------------------------------------------------------------------
//static float Mahony_invSqrt(float x)  // if use other platform please use float Mahony_Mahony_invSqrt(float x)
//{
//    volatile float tmp = 1.0f;
//    tmp /= __sqrtf(x);
//    return tmp;
//}

#define twoKpDef    (2.0f * 0.5f)   // 2 * proportional gain
#define twoKiDef    (2.0f * 0.0f)   // 2 * integral gain
//#define cheat TRUE  //����ģʽ ȥ����С��gyroֵ
#define correct_Time_define 1000    //�ϵ�ȥ0Ʈ 1000��ȡƽ��
float gyro_old[3];
float gyro[3], accel[3], temp;
float gyro_correct[3]={0};
float RefTemp = 40;   //Destination

float roll,pitch,yaw=0;
uint8 attitude_flag=0;
uint32_t correct_times=0;

float TurnAngle_Integral=0;

void imu_N_init(void)
{
    Mahony_Init(1000);  //mahony��̬�����ʼ��
    imu963ra_init();

}

/***
 * @brief: INS_TASK(void const * argument)
 * @param: argument - �������
 * @retval: void
 * @details: IMU��̬����������

*/
static uint8_t first_mahony=0;
void INS_Task(void)
{
        imu963ra_get_gyro();
        imu963ra_get_acc();
        // ���ӻ���ƽ���˲�
//        static float gyro_filter[3][5] = {0};

//        gyro[0] = moving_average(gyro[0], gyro_filter[0], 5);
//        gyro[1] = moving_average(gyro[1], gyro_filter[1], 5);
//        gyro[2] = moving_average(gyro[2], gyro_filter[2], 5);
        gyro_old[0] =(float)imu963ra_gyro_x*0.0012205f;//gyro/14.3*PI/180,0.001221fʱת360���1����
        gyro_old[1] =(float)imu963ra_gyro_y*0.0012205f;
        gyro_old[2] =(float)imu963ra_gyro_z*0.0012205f;
        accel[0]=(float)imu963ra_acc_x /4098;
        accel[1]=(float)imu963ra_acc_y /4098;
        accel[2]=(float)imu963ra_acc_z /4098;
                if(first_mahony==0)
                {
                    first_mahony++;
                    MahonyAHRSinit(accel[0],accel[1],accel[2],0,0,0);
                }
                if(attitude_flag==1)  //ekf����̬����
                {
                    gyro[0]=gyro_old[0]-gyro_correct[0];   //��ȥ������0Ʈ
                    gyro[1]=gyro_old[1]-gyro_correct[1];
                    gyro[2]=gyro_old[2]-gyro_correct[2];

                    #if cheat              //���� ������yaw���ȶ� ȥ���Ƚ�С��ֵ
                        if(fabsf(gyro[2])<0.003f)
                            gyro[2]=0;
                    #endif

                    //=================================================================================
                    //mahony��̬���㲿��
                    //HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
                    Mahony_update(gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2],0,0,0);
                    Mahony_computeAngles(); //�Ƕȼ���   ��ֲ�����ƽ̨��Ҫ�滻����Ӧ��arm_atan2_f32 �� arm_asin
                    //printf_data();
                }
                else if(attitude_flag==0)   //״̬1 ��ʼ1000�ε�������0Ʈ��ʼ��
                {
                        //gyro correct
                        gyro_correct[0]+=   gyro_old[0];
                        gyro_correct[1]+=   gyro_old[1];
                        gyro_correct[2]+=   gyro_old[2];
                        correct_times++;
                        if(correct_times>=correct_Time_define)
                        {
                            gyro_correct[0]/=correct_Time_define;
                            gyro_correct[1]/=correct_Time_define;
                            gyro_correct[2]/=correct_Time_define;
                            attitude_flag=1; //go to 2 state
                        }
                }
}


void Mahony_Init(float sampleFrequency)
{
    twoKi = twoKiDef;   // 2 * integral gain (Ki)
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
    anglesComputed = 0;
    invSampleFreq = 1.0f / sampleFrequency;
}



float Mahony_invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void MahonyAHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float init_yaw, init_pitch, init_roll;
    float cr2, cp2, cy2, sr2, sp2, sy2;
    float sin_roll, cos_roll, sin_pitch, cos_pitch;
    float magX, magY;

    recipNorm = Mahony_invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f))
    {
        recipNorm = Mahony_invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
    }

    init_pitch = atan2f(-ax, az);
    init_roll = atan2f(ay, az);

    sin_roll  = sinf(init_roll);
    cos_roll  = cosf(init_roll);
    cos_pitch = cosf(init_pitch);
    sin_pitch = sinf(init_pitch);

    if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f))
    {
        magX = mx * cos_pitch + my * sin_pitch * sin_roll + mz * sin_pitch * cos_roll;
        magY = my * cos_roll - mz * sin_roll;
        init_yaw  = atan2f(-magY, magX);
    }
    else
    {
        init_yaw=0.0f;
    }

    cr2 = cosf(init_roll * 0.5f);
    cp2 = cosf(init_pitch * 0.5f);
    cy2 = cosf(init_yaw * 0.5f);
    sr2 = sinf(init_roll * 0.5f);
    sp2 = sinf(init_pitch * 0.5f);
    sy2 = sinf(init_yaw * 0.5f);

    q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
    q1= sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
    q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
    q3= cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

    // Normalise quaternion
    recipNorm = Mahony_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    // Convert gyroscope degrees/sec to radians/sec
//      gx *= 0.0174533f;
//      gy *= 0.0174533f;
//      gz *= 0.0174533f;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // Compute feedback only if accelerometer measurement valid
    // (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = Mahony_invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = Mahony_invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction
        // and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            // integral error scaled by Ki
            integralFBx += twoKi * halfex * invSampleFreq;
            integralFBy += twoKi * halfey * invSampleFreq;
            integralFBz += twoKi * halfez * invSampleFreq;
            gx += integralFBx;  // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKpDef * halfex;
        gy += twoKpDef * halfey;
        gz += twoKpDef * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * invSampleFreq);       // pre-multiply common factors
    gy *= (0.5f * invSampleFreq);
    gz *= (0.5f * invSampleFreq);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = Mahony_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    anglesComputed = 0;
}
//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = Mahony_invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex  * invSampleFreq; // integral error scaled by Ki
            integralFBy += twoKi * halfey  * invSampleFreq;
            integralFBz += twoKi * halfez  * invSampleFreq;
            gx += integralFBx;  // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKpDef * halfex;
        gy += twoKpDef * halfey;
        gz += twoKpDef * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f *   invSampleFreq);     // pre-multiply common factors
    gy *= (0.5f  * invSampleFreq);
    gz *= (0.5f  * invSampleFreq);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = Mahony_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void Mahony_computeAngles()
{
    roll_mahony=atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
    roll_mahony *= 57.29578f;
    pitch_mahony = asinf(-2.0f * (q1*q3 - q0*q2));
    pitch_mahony *= 57.29578f;
    yaw_mahony=atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
    yaw_mahony *=57.29578f;
    anglesComputed = 1;
}
float getRoll() {
    if (!anglesComputed){Mahony_computeAngles();}
    return roll_mahony;
}
float getPitch() {
    if (!anglesComputed){Mahony_computeAngles();}
    return pitch_mahony;
}
float getYaw() {
    if (!anglesComputed){Mahony_computeAngles();}
    return yaw_mahony;
}
float world_getYaw() {
    if (!anglesComputed){
        Mahony_computeAngles();
//        world_yaw_mahony = 360 - (yaw_mahony + imu_start_err);
    }
    world_yaw_mahony =yaw_mahony + imu_start_err;
    if(world_yaw_mahony > 180.0)
        world_yaw_mahony -= 360.0;
    else if(world_yaw_mahony < -180.0)
        world_yaw_mahony += 360.0;
    return world_yaw_mahony;
}

float car_getYaw() {
    if (!anglesComputed){Mahony_computeAngles();}
    float car_yaw_mahony = yaw_mahony+car_head_err;
    if(car_yaw_mahony>180.0)
        car_yaw_mahony-=360;
    else if(car_yaw_mahony < -180.0)
        car_yaw_mahony += 360;
    return car_yaw_mahony;
}

void printf_data(void)
{
    printf("%f,%f,%f\r\n",roll_mahony,pitch_mahony,yaw_mahony);
}
//============================================================================================
// END OF CODE
//============================================================================================
