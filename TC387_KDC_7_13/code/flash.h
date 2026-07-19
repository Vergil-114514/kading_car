#ifndef _FLASH_H
#define _FLASH_H

#include "zf_driver_flash.h"

#define FLASH_IMU_POINT_CAPACITY    (1024U)

typedef union
{
    uint64  u64_type;
    uint32  u32_type;
    double  double_type;
    float   float_type;
} DataTypeUnion;

extern DataTypeUnion IMU_data;
extern DataTypeUnion GPS_data;

/* RAM point buffer persisted by DATA_SAVE() + IMU_POINT_SAVE(). */
extern uint16 IMU_savenum;
extern float IMU_X[FLASH_IMU_POINT_CAPACITY];
extern float IMU_Y[FLASH_IMU_POINT_CAPACITY];

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
