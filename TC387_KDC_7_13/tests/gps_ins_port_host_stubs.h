#ifndef GPS_INS_PORT_HOST_STUBS_H
#define GPS_INS_PORT_HOST_STUBS_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <float.h>
#include <math.h>

#define _zf_common_headfile_h_

typedef uint8_t uint8;
typedef int8_t int8;
typedef int16_t int16;
typedef uint16_t uint16;
typedef int32_t int32;
typedef uint32_t uint32;
typedef uint8_t boolean;

#define TAU1201 (1)

typedef struct
{
    uint8 state;
    double latitude;
    double longitude;
    int8 ns;
    int8 ew;
    float speed;
    float direction;
    uint8 satellite_used;
} gnss_info_struct;

extern gnss_info_struct gnss;
extern volatile uint8 gnss_flag;
extern int16 imu963ra_gyro_x;
extern int16 imu963ra_gyro_y;
extern int16 imu963ra_gyro_z;
extern int16 imu963ra_acc_x;
extern int16 imu963ra_acc_y;
extern int16 imu963ra_acc_z;

boolean IfxCpu_disableInterrupts(void);
void IfxCpu_restoreInterrupts(boolean state);
uint8 imu963ra_init(void);
void imu963ra_get_gyro(void);
void imu963ra_get_acc(void);
float imu963ra_gyro_transition(float raw);
float imu963ra_acc_transition(int16 raw);
void system_delay_ms(uint32 milliseconds);
void gnss_init(int device);
uint8 gnss_data_parse(void);

#include "FusionAhrs.h"
#include "MahonyFilter.h"
#include "TopSpeed_GPS_INS.h"
#include "TopSpeed_GPS_INS_Port.h"

#endif
