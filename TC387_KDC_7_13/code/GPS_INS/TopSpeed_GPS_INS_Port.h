#ifndef _TOPSPEED_GPS_INS_PORT_H_
#define _TOPSPEED_GPS_INS_PORT_H_

#include "TopSpeed_GPS_INS.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TC387 hardware adapter defaults, matching the active TopSpeed sensor setup. */
#define TOPSPEED_GPS_INS_PERIOD_MS                 (5U)
#define TOPSPEED_GPS_INS_PERIOD_S                  (0.005f)
#define TOPSPEED_GPS_INS_GPS_DELAY_S               (0.30f)
#define TOPSPEED_GPS_INS_GYRO_CALIBRATION_SAMPLES  (100U)
#define TOPSPEED_GPS_INS_AUTO_ORIGIN_ENABLE        (1U)

/*
 * CPU1_NAVIGATION_SOLUTION 可选择的三个编译期常量。
 * 三种后端只负责姿态/航向滤波；车辆坐标均使用同一套后轮编码器航位推算，
 * CPU1_USE_GPS=1 时再由 GPS 对通用导航坐标和速度进行校正。
 * 存点模式的局部坐标始终直接使用 Mahony 航向和编码器速度积分，不使用
 * GPS 位置修正。
 */
#define NAVIGATION_SOLUTION_MAHONY                  (1U)
#define NAVIGATION_SOLUTION_FUSION                  (2U)
#define NAVIGATION_SOLUTION_GPS_INS                 (3U)

/*
 * TopSpeed's Madgwick yaw is left/CCW positive.  The navigation API uses the
 * conventional GNSS heading (right/CW positive), so the default sign is -1.
 * Change the sign/offset if the IMU mounting direction differs.
 */
#define TOPSPEED_GPS_INS_IMU_HEADING_SIGN          (-1.0f)
#define TOPSPEED_GPS_INS_IMU_HEADING_OFFSET_DEG    (0.0f)

typedef struct
{
    float acc_x_mps2;
    float acc_y_mps2;
    float acc_z_mps2;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float roll_deg;
    float pitch_deg;
    float raw_yaw_deg;
    float heading_deg;
    uint8_t valid;
} TopSpeed_GPS_INS_IMU_Output;

/** Menu/debug snapshot of the latest parsed GNSS values. */
typedef struct
{
    double latitude_deg;
    double longitude_deg;
    float speed_mps;
    float direction_deg;
    uint8_t satellite_used;
    uint8_t enabled;
    uint8_t valid;
} TopSpeed_GPS_INS_GPS_Output;

/** CPU1-only local odometry used by SAVE POINT mode. */
typedef struct
{
    TopSpeed_GPS_INS_Point position_m;
    float relative_yaw_deg;
    float encoder_speed_mps;
    float distance_m;
    uint32_t reset_sequence;
    uint8_t initialized;
    uint8_t heading_valid;
    uint8_t encoder_valid;
    uint8_t valid;
} TopSpeed_GPS_INS_LocalOdometryOutput;

/*
 * Initializes the selected attitude solution, IMU/INS and optional GPS.
 * Returns 0 when IMU963RA initialization succeeds, 1 on IMU failure.
 * With use_gps=0 the estimator still runs from IMU + wheel encoder speed.
 */
uint8_t TopSpeed_GPS_INS_PortInit(uint8_t use_gps,
                                  uint8_t navigation_solution);

/** 返回当前选择值和适合菜单显示的名称。 */
uint8_t TopSpeed_GPS_INS_PortGetNavigationSolution(void);
const char *TopSpeed_GPS_INS_PortGetNavigationSolutionName(void);

/* Call from a precise 5 ms PIT ISR. GPS/encoder queues are consumed here. */
void TopSpeed_GPS_INS_Port5msCallback(void);

/* Call continuously from the CPU1 main loop; it only queues fresh RMC data. */
void TopSpeed_GPS_INS_PortGnssPoll(void);

/* Runtime alignment for a different IMU installation/launch direction. */
void TopSpeed_GPS_INS_PortSetHeadingAlignment(float sign, float offset_deg);

/* Blocking calibration; keep the vehicle stationary. Call before PIT starts. */
uint8_t TopSpeed_GPS_INS_PortCalibrateGyro(uint16_t sample_count);
void TopSpeed_GPS_INS_PortSetGyroBiasRaw(float x, float y, float z);

/* Origin is geodetic; the next GPS update is expressed relative to it. */
void TopSpeed_GPS_INS_PortSetOrigin(double latitude_deg, double longitude_deg);
void TopSpeed_GPS_INS_PortResetOrigin(void);
uint8_t TopSpeed_GPS_INS_PortHasOrigin(void);

/* Optional direct GPS entry points for a different parser or test source. */
void TopSpeed_GPS_INS_PortGPSGeodeticUpdate(double latitude_deg,
                                            double longitude_deg,
                                            float speed_mps,
                                            uint8_t valid);
void TopSpeed_GPS_INS_PortGPSLocalUpdate(float east_m,
                                        float north_m,
                                        float speed_mps,
                                        uint8_t valid);

/* Encoder adapter: signed forward speed or signed delta count.
 * CPU2 control ISR publishes the newest sample through a single-slot mailbox;
 * CPU1 consumes it on the next 5 ms navigation tick. */
void TopSpeed_GPS_INS_PortEncoderSpeedUpdate(float speed_mps);
void TopSpeed_GPS_INS_PortEncoderInvalidate(void);
void TopSpeed_GPS_INS_PortEncoderDeltaUpdate(int32_t delta_count,
                                             float metre_per_count,
                                             float sample_period_s);

/* CPU2 requests a new (0,0,0 deg) launch frame. CPU1 applies it on its next
 * 5 ms navigation tick and returns the request sequence in the snapshot. */
uint32_t TopSpeed_GPS_INS_PortRequestLocalOdometryReset(void);
void TopSpeed_GPS_INS_PortGetLocalOdometryOutput(
    TopSpeed_GPS_INS_LocalOdometryOutput *output);

/* Sequence-checked snapshots safe for CPU0/CPU2 cross-core readers. */
void TopSpeed_GPS_INS_PortGetOutput(TopSpeed_GPS_INS_Output *output);
void TopSpeed_GPS_INS_PortGetIMUOutput(TopSpeed_GPS_INS_IMU_Output *output);
void TopSpeed_GPS_INS_PortGetGPSOutput(TopSpeed_GPS_INS_GPS_Output *output);

#ifdef __cplusplus
}
#endif

#endif
