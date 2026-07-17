#ifndef _FLASH_H
#define _FLASH_H

#include "zf_driver_flash.h"

typedef union
{
    uint64  u64_type;
    uint32  u32_type;
    double  double_type;
    float   float_type;
} DataTypeUnion;

extern DataTypeUnion IMU_data;
extern DataTypeUnion GPS_data;

void DATA_SAVE(void);
void GPS_POINT_SAVE(void);
void IMU_POINT_SAVE(void);

void DATA_READ(void);
void GPS_POINT_READ(void);
void IMU_POINT_READ(void);

void DATA_buffer_clean(void);
void GPS_buffer_clean(void);
void IMU_buffer_clean(void);

void Flash_Test_Routine(void);

#endif
