#ifndef __IMU_N_H
#define __IMU_N_H

#include "zf_common_headfile.h"

extern uint8 attitude_flag;
extern float TurnAngle_Integral;
extern float gyro[3];
void INS_Task(void);
void IMU_task(void);
void imu_N_init(void);
void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void Mahony_Init(float sampleFrequency);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void Mahony_computeAngles(void);
void MahonyAHRSinit(float ax, float ay, float az, float mx, float my, float mz);
float getRoll(void);
float getPitch(void);
float getYaw(void);
float world_getYaw(void);
float car_getYaw(void);
float getRollRadians(void);
float getPitchRadians(void);
float getYawRadians(void);
void printf_data(void);

#endif
