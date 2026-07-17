// author: Doro   date: 2026.6.5   version: 1.0
// Notes:
// Test 9-axis IMU code, currently using IMU963RA.

#include "zf_common_headfile.h"

#define DELTA_TIME_SECONDS          (1.0f / (float)IMU_SAMPLE_RATE_HZ)
#define IMU_CALIBRATE_SAMPLE_COUNT  (200U)
#define GAUSS_TO_MICROTESLA         (100.0f)

static FusionAhrs ahrs;
static FusionBias bias;

static const FusionMatrix gyroscopeMisalignment = {
    .array = {1.0f, 0.0f, 0.0f,
              0.0f, 1.0f, 0.0f,
              0.0f, 0.0f, 1.0f}
};
static const FusionVector gyroscopeSensitivity = {.array = {1.0f, 1.0f, 1.0f}};
static FusionVector gyroscopeOffset = {.array = {0.0f, 0.0f, 0.0f}};

static const FusionMatrix accelerometerMisalignment = {
    .array = {1.0f, 0.0f, 0.0f,
              0.0f, 1.0f, 0.0f,
              0.0f, 0.0f, 1.0f}
};
static const FusionVector accelerometerSensitivity = {.array = {1.0f, 1.0f, 1.0f}};
static FusionVector accelerometerOffset = {.array = {0.0f, 0.0f, 0.0f}};

// IMU963RA magnetometer calibration calculated from imu2.csv with MATLAB magcal.
// softIronMatrix is dimensionless. hardIronOffset is in uT because the input to
// FusionModelMagnetic is converted to uT before calibration.
static const FusionMatrix softIronMatrix = {
    .array = { 0.978173589f, -0.011825151f, -0.005114699f,
              -0.011825151f,  0.988800313f,  0.013310992f,
              -0.005114699f,  0.013310992f,  1.034246501f}
};
static const FusionVector hardIronOffset = {
    .array = {-3.792296275f, 0.005240282f, 0.073910123f}
};

static FusionVector gyroscope = {.array = {0.0f, 0.0f, 0.0f}};
static FusionVector accelerometer = {.array = {0.0f, 0.0f, 1.0f}};
static FusionVector magnetometer = {.array = {1.0f, 0.0f, 0.0f}};
static imu_solution_t solution;
static FusionVector calibrationGyroSum;
static FusionVector calibrationAccelSum;
static volatile uint16 calibrationSampleCount = 0U;
static volatile imu_state_t imuState = IMU_STATE_UNINITIALISED;
static volatile uint8 imuInitialised = 0U;

static void imu_calibrate_step(void);

void imu_init(void)
{
    imuInitialised = 0U;
    imuState = IMU_STATE_UNINITIALISED;
    FusionAhrsInitialise(&ahrs);

    const FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,
        .gain = 0.5f,
        .gyroscopeRange = 2000.0f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 10.0f,
        .recoveryTriggerPeriod = 5U * IMU_SAMPLE_RATE_HZ,
    };

    FusionAhrsSetSettings(&ahrs, &settings);

    FusionBiasInitialise(&bias);

    FusionBiasSettings biasSettings = fusionBiasDefaultSettings;
    biasSettings.sampleRate = (float)IMU_SAMPLE_RATE_HZ;
    FusionBiasSetSettings(&bias, &biasSettings);

    if (0U != imu963ra_init())
    {
        imuInitialised = 0U;
        imuState = IMU_STATE_ERROR;
        zf_log(0, "imu963ra_init failed.");
        return;
    }

    imuInitialised = 1U;

    // Start calibration without blocking here. imu_calculate() collects one
    // sample per call while still producing data for the serial port.
    imu_calibrate();
}

void imu_calibrate(void)
{
    if (0U == imuInitialised)
    {
        return;
    }

    calibrationGyroSum = FUSION_VECTOR_ZERO;
    calibrationAccelSum = FUSION_VECTOR_ZERO;
    calibrationSampleCount = 0U;
    imuState = IMU_STATE_CALIBRATING;
}

static void imu_calibrate_step(void)
{
    // imu_data_get() has already converted the newest sample to physical units.
    calibrationGyroSum.axis.x += gyroscope.axis.x;
    calibrationGyroSum.axis.y += gyroscope.axis.y;
    calibrationGyroSum.axis.z += gyroscope.axis.z;

    calibrationAccelSum.axis.x += accelerometer.axis.x;
    calibrationAccelSum.axis.y += accelerometer.axis.y;
    calibrationAccelSum.axis.z += accelerometer.axis.z;

    calibrationSampleCount++;
    if (calibrationSampleCount < IMU_CALIBRATE_SAMPLE_COUNT)
    {
        return;
    }

    gyroscopeOffset.axis.x = calibrationGyroSum.axis.x / (float)IMU_CALIBRATE_SAMPLE_COUNT;
    gyroscopeOffset.axis.y = calibrationGyroSum.axis.y / (float)IMU_CALIBRATE_SAMPLE_COUNT;
    gyroscopeOffset.axis.z = calibrationGyroSum.axis.z / (float)IMU_CALIBRATE_SAMPLE_COUNT;

    accelerometerOffset.axis.x = calibrationAccelSum.axis.x / (float)IMU_CALIBRATE_SAMPLE_COUNT;
    accelerometerOffset.axis.y = calibrationAccelSum.axis.y / (float)IMU_CALIBRATE_SAMPLE_COUNT;
    accelerometerOffset.axis.z = (calibrationAccelSum.axis.z / (float)IMU_CALIBRATE_SAMPLE_COUNT) - 1.0f;

    FusionBiasSetOffset(&bias, (FusionVector){.array = {0.0f, 0.0f, 0.0f}});
    imuState = IMU_STATE_READY;
}

void imu_data_get(void)
{
    if (0U == imuInitialised)
    {
        return;
    }

    imu963ra_get_gyro();
    imu963ra_get_acc();
    imu963ra_get_mag();

    gyroscope.axis.x = imu963ra_gyro_transition(imu963ra_gyro_x);
    gyroscope.axis.y = imu963ra_gyro_transition(imu963ra_gyro_y);
    gyroscope.axis.z = imu963ra_gyro_transition(imu963ra_gyro_z);

    accelerometer.axis.x = imu963ra_acc_transition(imu963ra_acc_x);
    accelerometer.axis.y = imu963ra_acc_transition(imu963ra_acc_y);
    accelerometer.axis.z = imu963ra_acc_transition(imu963ra_acc_z);

    // The driver returns gauss. 1 gauss = 100 microtesla.
    magnetometer.axis.x = imu963ra_mag_transition(imu963ra_mag_x) * GAUSS_TO_MICROTESLA;
    magnetometer.axis.y = imu963ra_mag_transition(imu963ra_mag_y) * GAUSS_TO_MICROTESLA;
    magnetometer.axis.z = imu963ra_mag_transition(imu963ra_mag_z) * GAUSS_TO_MICROTESLA;
}

void imu_calculate(void)
{
    FusionVector calibratedGyroscope;
    FusionVector calibratedAccelerometer;
    FusionVector calibratedMagnetometer;
    FusionVector earth;

    if (0U == imuInitialised)
    {
        return;
    }

    imu_data_get();

    if (IMU_STATE_CALIBRATING == imuState)
    {
        imu_calibrate_step();
    }

    calibratedGyroscope = FusionModelInertial(gyroscope,
                                              gyroscopeMisalignment,
                                              gyroscopeSensitivity,
                                              gyroscopeOffset);
    calibratedAccelerometer = FusionModelInertial(accelerometer,
                                                  accelerometerMisalignment,
                                                  accelerometerSensitivity,
                                                  accelerometerOffset);
    calibratedMagnetometer = FusionModelMagnetic(magnetometer,
                                                 softIronMatrix,
                                                 hardIronOffset);

    calibratedGyroscope = FusionBiasUpdate(&bias, calibratedGyroscope);

    FusionAhrsUpdate(&ahrs,
                     calibratedGyroscope,
                     calibratedAccelerometer,
                     calibratedMagnetometer,
                     DELTA_TIME_SECONDS);

    earth = FusionAhrsGetEarthAcceleration(&ahrs);

    solution.accelerometer_mps2 = FusionVectorScale(calibratedAccelerometer,
                                                    IMU_STANDARD_GRAVITY_MPS2);
    solution.gyroscope_dps = calibratedGyroscope;
    solution.magnetometer_uT = calibratedMagnetometer;
    solution.attitude_deg = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    solution.linear_acceleration_nwu_mps2 = FusionVectorScale(earth,
                                                              IMU_STANDARD_GRAVITY_MPS2);
}

uint8 imu_is_ready(void)
{
    return imuInitialised;
}

imu_state_t imu_get_state(void)
{
    return imuState;
}

uint8 imu_get_calibration_progress(void)
{
    if (IMU_STATE_READY == imuState)
    {
        return 100U;
    }

    if (IMU_STATE_CALIBRATING != imuState)
    {
        return 0U;
    }

    return (uint8)(((uint32)calibrationSampleCount * 100U) /
                   IMU_CALIBRATE_SAMPLE_COUNT);
}

const imu_solution_t *imu_get_solution(void)
{
    return &solution;
}

void imu_print_header(void)
{
    if (0U == imuInitialised)
    {
        return;
    }

    printf("ax_mps2,ay_mps2,az_mps2,gx_dps,gy_dps,gz_dps,"
           "mx_uT,my_uT,mz_uT,roll_deg,pitch_deg,yaw_deg,"
           "lin_north_mps2,lin_west_mps2,lin_up_mps2\r\n");
}

void imu_print_solution(void)
{
    if (0U == imuInitialised)
    {
        return;
    }

    printf(//"%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,"
           "%.3f,%.3f,%.3f,%.5f,%.5f,%.5f\r\n",
//           solution.accelerometer_mps2.axis.x,
//           solution.accelerometer_mps2.axis.y,
//           solution.accelerometer_mps2.axis.z,
//           solution.gyroscope_dps.axis.x,
//           solution.gyroscope_dps.axis.y,
//           solution.gyroscope_dps.axis.z,
//           solution.magnetometer_uT.axis.x,
//           solution.magnetometer_uT.axis.y,
//           solution.magnetometer_uT.axis.z,
           solution.attitude_deg.angle.roll,
           solution.attitude_deg.angle.pitch,
           solution.attitude_deg.angle.yaw,
           solution.linear_acceleration_nwu_mps2.axis.x,
           solution.linear_acceleration_nwu_mps2.axis.y,
           solution.linear_acceleration_nwu_mps2.axis.z);
}

void angle_printf(void)
{
    imu_print_solution();
}
