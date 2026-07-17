
#ifndef _IMU_H
#define _IMU_H

#include "zf_common_headfile.h"
#include "Fusion.h"

#define IMU_SAMPLE_RATE_HZ          (100U)
#define IMU_OUTPUT_RATE_HZ          (10U)
#define IMU_SAMPLE_PERIOD_MS        (1000U / IMU_SAMPLE_RATE_HZ)
#define IMU_OUTPUT_DECIMATION       (IMU_SAMPLE_RATE_HZ / IMU_OUTPUT_RATE_HZ)
#define IMU_STANDARD_GRAVITY_MPS2   (9.80665f)

typedef enum
{
    IMU_STATE_UNINITIALISED = 0,
    IMU_STATE_CALIBRATING,
    IMU_STATE_READY,
    IMU_STATE_ERROR
} imu_state_t;

// Units are part of the member names to prevent accidental mixing.
// Body axes must be X-forward, Y-left, Z-up to match FusionConventionNwu.
typedef struct
{
    FusionVector accelerometer_mps2;
    FusionVector gyroscope_dps;
    FusionVector magnetometer_uT;
    FusionEuler attitude_deg;
    FusionVector linear_acceleration_nwu_mps2;
} imu_solution_t;

void imu_init(void);
void imu_calibrate(void);
void imu_data_get(void);
void imu_calculate(void);
// Returns 1 after the IMU963RA hardware has initialised. Calibration may
// still be running; use imu_get_state() when calibration status is required.
uint8 imu_is_ready(void);
imu_state_t imu_get_state(void);
uint8 imu_get_calibration_progress(void);
const imu_solution_t *imu_get_solution(void);
void imu_print_header(void);
void imu_print_solution(void);
void angle_printf(void);

#endif
