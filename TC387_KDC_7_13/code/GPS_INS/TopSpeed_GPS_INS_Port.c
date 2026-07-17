#include "zf_common_headfile.h"

#define TOPSPEED_GPS_INS_PORT_PI               (3.14159265358979323846f)
#define TOPSPEED_GPS_INS_GRAVITY_MPS2          (9.80665f)
#define TOPSPEED_GPS_INS_METRE_PER_DEGREE      (111000.0)
#define TOPSPEED_GPS_INS_MADGWICK_BETA         (0.01f)

typedef struct
{
    volatile uint8_t ready;
    float speed_mps;
    uint8_t valid;
} TopSpeed_GPS_INS_EncoderPending;

typedef struct
{
    volatile uint8_t ready;
    TopSpeed_GPS_INS_Point position_m;
    float speed_mps;
    uint8_t valid;
} TopSpeed_GPS_INS_GPSPending;

typedef struct
{
    uint8_t initialized;
    uint8_t imu_ready;
    uint8_t gps_enabled;
    uint8_t navigation_solution;
    uint8_t origin_valid;
    volatile uint8_t reset_pending;

    double origin_latitude_deg;
    double origin_longitude_deg;
    float origin_cos_latitude;

    float heading_sign;
    float heading_offset_deg;
    float gyro_bias_x_raw;
    float gyro_bias_y_raw;
    float gyro_bias_z_raw;

    float q0;
    float q1;
    float q2;
    float q3;
    MahonyFilter_t mahony;
    FusionAhrs fusion;

    TopSpeed_GPS_INS_EncoderPending encoder_pending;
    TopSpeed_GPS_INS_GPSPending gps_pending;

    TopSpeed_GPS_INS_IMU_Output imu_output;
    TopSpeed_GPS_INS_GPS_Output gps_output;
    TopSpeed_GPS_INS_Output navigation_snapshot;
    TopSpeed_GPS_INS_IMU_Output imu_snapshot;
    TopSpeed_GPS_INS_GPS_Output gps_snapshot;
    volatile uint32_t snapshot_sequence;
} TopSpeed_GPS_INS_PortContext;

static TopSpeed_GPS_INS_PortContext g_topSpeedGpsInsPort;

static uint8_t TopSpeed_GPS_INS_PortIsFiniteFloat(float value)
{
    return (uint8_t)((value == value) &&
                     (value <= FLT_MAX) &&
                     (value >= -FLT_MAX));
}

static uint8_t TopSpeed_GPS_INS_PortIsFiniteDouble(double value)
{
    /* Latitude/longitude callers also apply numeric ranges below. The self
     * comparison rejects NaN; infinities fail those range checks. */
    return (uint8_t)(value == value);
}

static float TopSpeed_GPS_INS_PortNormalizeHeading(float heading_deg)
{
    while (heading_deg >= 180.0f)
    {
        heading_deg -= 360.0f;
    }
    while (heading_deg < -180.0f)
    {
        heading_deg += 360.0f;
    }
    return heading_deg;
}

static void TopSpeed_GPS_INS_PortResetQuaternion(void)
{
    FusionAhrsSettings fusion_settings;

    g_topSpeedGpsInsPort.q0 = 1.0f;
    g_topSpeedGpsInsPort.q1 = 0.0f;
    g_topSpeedGpsInsPort.q2 = 0.0f;
    g_topSpeedGpsInsPort.q3 = 0.0f;

    /* Mahony 和 Fusion 使用与 GPS_INS 相同的 5 ms 采样周期和初始姿态。 */
    MahonyFilter_Init(&g_topSpeedGpsInsPort.mahony, 0.5f, 0.0f);
    FusionAhrsInitialise(&g_topSpeedGpsInsPort.fusion);
    fusion_settings = fusionAhrsDefaultSettings;
    fusion_settings.convention = FusionConventionNwu;
    fusion_settings.gain = 0.5f;
    fusion_settings.gyroscopeRange = 2000.0f;
    fusion_settings.accelerationRejection = 10.0f;
    fusion_settings.magneticRejection = 0.0f;
    fusion_settings.recoveryTriggerPeriod = 1000U;
    FusionAhrsSetSettings(&g_topSpeedGpsInsPort.fusion, &fusion_settings);
}

static void TopSpeed_GPS_INS_PortMadgwickUpdate(float gx_rad_s,
                                                float gy_rad_s,
                                                float gz_rad_s,
                                                float ax_g,
                                                float ay_g,
                                                float az_g,
                                                float dt_s)
{
    float q0 = g_topSpeedGpsInsPort.q0;
    float q1 = g_topSpeedGpsInsPort.q1;
    float q2 = g_topSpeedGpsInsPort.q2;
    float q3 = g_topSpeedGpsInsPort.q3;
    float q_dot0;
    float q_dot1;
    float q_dot2;
    float q_dot3;
    float norm;

    q_dot0 = 0.5f * (-q1 * gx_rad_s - q2 * gy_rad_s - q3 * gz_rad_s);
    q_dot1 = 0.5f * ( q0 * gx_rad_s + q2 * gz_rad_s - q3 * gy_rad_s);
    q_dot2 = 0.5f * ( q0 * gy_rad_s - q1 * gz_rad_s + q3 * gx_rad_s);
    q_dot3 = 0.5f * ( q0 * gz_rad_s + q1 * gy_rad_s - q2 * gx_rad_s);

    norm = ax_g * ax_g + ay_g * ay_g + az_g * az_g;
    if (norm > 0.000001f)
    {
        float reciprocal_norm = 1.0f / sqrtf(norm);
        float s0;
        float s1;
        float s2;
        float s3;
        float gradient_norm;
        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;

        ax_g *= reciprocal_norm;
        ay_g *= reciprocal_norm;
        az_g *= reciprocal_norm;

        s0 = 4.0f * q0 * q2q2 + 2.0f * q2 * ax_g +
             4.0f * q0 * q1q1 - 2.0f * q1 * ay_g;
        s1 = 4.0f * q1 * q3q3 - 2.0f * q3 * ax_g +
             4.0f * q0q0 * q1 - 2.0f * q0 * ay_g - 4.0f * q1 +
             8.0f * q1 * q1q1 + 8.0f * q1 * q2q2 + 4.0f * q1 * az_g;
        s2 = 4.0f * q0q0 * q2 + 2.0f * q0 * ax_g +
             4.0f * q2 * q3q3 - 2.0f * q3 * ay_g - 4.0f * q2 +
             8.0f * q2 * q1q1 + 8.0f * q2 * q2q2 + 4.0f * q2 * az_g;
        s3 = 4.0f * q1q1 * q3 - 2.0f * q1 * ax_g +
             4.0f * q2q2 * q3 - 2.0f * q2 * ay_g;

        gradient_norm = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (gradient_norm > 0.000001f)
        {
            reciprocal_norm = 1.0f / sqrtf(gradient_norm);
            q_dot0 -= TOPSPEED_GPS_INS_MADGWICK_BETA * s0 * reciprocal_norm;
            q_dot1 -= TOPSPEED_GPS_INS_MADGWICK_BETA * s1 * reciprocal_norm;
            q_dot2 -= TOPSPEED_GPS_INS_MADGWICK_BETA * s2 * reciprocal_norm;
            q_dot3 -= TOPSPEED_GPS_INS_MADGWICK_BETA * s3 * reciprocal_norm;
        }
    }

    q0 += q_dot0 * dt_s;
    q1 += q_dot1 * dt_s;
    q2 += q_dot2 * dt_s;
    q3 += q_dot3 * dt_s;

    norm = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if (norm > 0.000001f)
    {
        float reciprocal_norm = 1.0f / sqrtf(norm);
        g_topSpeedGpsInsPort.q0 = q0 * reciprocal_norm;
        g_topSpeedGpsInsPort.q1 = q1 * reciprocal_norm;
        g_topSpeedGpsInsPort.q2 = q2 * reciprocal_norm;
        g_topSpeedGpsInsPort.q3 = q3 * reciprocal_norm;
    }
}

static void TopSpeed_GPS_INS_PortUpdateEuler(void)
{
    TopSpeed_GPS_INS_IMU_Output *imu = &g_topSpeedGpsInsPort.imu_output;
    float q0 = g_topSpeedGpsInsPort.q0;
    float q1 = g_topSpeedGpsInsPort.q1;
    float q2 = g_topSpeedGpsInsPort.q2;
    float q3 = g_topSpeedGpsInsPort.q3;
    float sin_pitch;
    float raw_yaw;

    imu->roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3),
                           1.0f - 2.0f * (q1 * q1 + q2 * q2)) *
                    (180.0f / TOPSPEED_GPS_INS_PORT_PI);

    sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    if (sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }
    else if (sin_pitch < -1.0f)
    {
        sin_pitch = -1.0f;
    }
    imu->pitch_deg = asinf(sin_pitch) *
                     (180.0f / TOPSPEED_GPS_INS_PORT_PI);

    raw_yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                     1.0f - 2.0f * (q2 * q2 + q3 * q3)) *
              (180.0f / TOPSPEED_GPS_INS_PORT_PI);
    imu->raw_yaw_deg = TopSpeed_GPS_INS_PortNormalizeHeading(raw_yaw);
    imu->heading_deg = TopSpeed_GPS_INS_PortNormalizeHeading(
        raw_yaw * g_topSpeedGpsInsPort.heading_sign +
        g_topSpeedGpsInsPort.heading_offset_deg);
}

/** 根据 CPU0 选择的后端更新四元数，再交给统一的欧拉角/航向转换。 */
static void TopSpeed_GPS_INS_PortUpdateAttitude(float gyro_x_dps,
                                                float gyro_y_dps,
                                                float gyro_z_dps,
                                                float acc_x_g,
                                                float acc_y_g,
                                                float acc_z_g)
{
    if(g_topSpeedGpsInsPort.navigation_solution == NAVIGATION_SOLUTION_MAHONY)
    {
        MahonyFilter_UpdateIMU(&g_topSpeedGpsInsPort.mahony,
                               gyro_x_dps, gyro_y_dps, gyro_z_dps,
                               acc_x_g, acc_y_g, acc_z_g,
                               TOPSPEED_GPS_INS_PERIOD_S);
        g_topSpeedGpsInsPort.q0 = g_topSpeedGpsInsPort.mahony.q0;
        g_topSpeedGpsInsPort.q1 = g_topSpeedGpsInsPort.mahony.q1;
        g_topSpeedGpsInsPort.q2 = g_topSpeedGpsInsPort.mahony.q2;
        g_topSpeedGpsInsPort.q3 = g_topSpeedGpsInsPort.mahony.q3;
    }
    else if(g_topSpeedGpsInsPort.navigation_solution == NAVIGATION_SOLUTION_FUSION)
    {
        FusionVector gyroscope = {
            .array = {gyro_x_dps, gyro_y_dps, gyro_z_dps}
        };
        FusionVector accelerometer = {
            .array = {acc_x_g, acc_y_g, acc_z_g}
        };
        FusionQuaternion quaternion;

        FusionAhrsUpdateNoMagnetometer(&g_topSpeedGpsInsPort.fusion,
                                       gyroscope,
                                       accelerometer,
                                       TOPSPEED_GPS_INS_PERIOD_S);
        quaternion = FusionAhrsGetQuaternion(&g_topSpeedGpsInsPort.fusion);
        g_topSpeedGpsInsPort.q0 = quaternion.element.w;
        g_topSpeedGpsInsPort.q1 = quaternion.element.x;
        g_topSpeedGpsInsPort.q2 = quaternion.element.y;
        g_topSpeedGpsInsPort.q3 = quaternion.element.z;
    }
    else
    {
        /* GPS_INS 后端沿用当前 TopSpeed 移植中的 Madgwick 6DOF 姿态解算。 */
        TopSpeed_GPS_INS_PortMadgwickUpdate(
            gyro_x_dps * (TOPSPEED_GPS_INS_PORT_PI / 180.0f),
            gyro_y_dps * (TOPSPEED_GPS_INS_PORT_PI / 180.0f),
            gyro_z_dps * (TOPSPEED_GPS_INS_PORT_PI / 180.0f),
            acc_x_g,
            acc_y_g,
            acc_z_g,
            TOPSPEED_GPS_INS_PERIOD_S);
    }

    TopSpeed_GPS_INS_PortUpdateEuler();
}

static void TopSpeed_GPS_INS_PortReadIMU(void)
{
    TopSpeed_GPS_INS_IMU_Output *imu = &g_topSpeedGpsInsPort.imu_output;
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;

    imu963ra_get_gyro();
    imu->gyro_x_dps = imu963ra_gyro_transition(
        (float)imu963ra_gyro_x - g_topSpeedGpsInsPort.gyro_bias_x_raw);
    imu->gyro_y_dps = imu963ra_gyro_transition(
        (float)imu963ra_gyro_y - g_topSpeedGpsInsPort.gyro_bias_y_raw);
    imu->gyro_z_dps = imu963ra_gyro_transition(
        (float)imu963ra_gyro_z - g_topSpeedGpsInsPort.gyro_bias_z_raw);

    imu963ra_get_acc();
    acc_x_g = imu963ra_acc_transition(imu963ra_acc_x);
    acc_y_g = imu963ra_acc_transition(imu963ra_acc_y);
    acc_z_g = imu963ra_acc_transition(imu963ra_acc_z);

    imu->acc_x_mps2 = acc_x_g * TOPSPEED_GPS_INS_GRAVITY_MPS2;
    imu->acc_y_mps2 = acc_y_g * TOPSPEED_GPS_INS_GRAVITY_MPS2;
    imu->acc_z_mps2 = acc_z_g * TOPSPEED_GPS_INS_GRAVITY_MPS2;

    TopSpeed_GPS_INS_PortUpdateAttitude(
        imu->gyro_x_dps,
        imu->gyro_y_dps,
        imu->gyro_z_dps,
        acc_x_g,
        acc_y_g,
        acc_z_g);
    imu->valid = 1U;
}

static void TopSpeed_GPS_INS_PortPublishSnapshot(void)
{
    g_topSpeedGpsInsPort.snapshot_sequence++;
    g_topSpeedGpsInsPort.navigation_snapshot = TopSpeed_GPS_INS_GetOutput();
    g_topSpeedGpsInsPort.imu_snapshot = g_topSpeedGpsInsPort.imu_output;
    g_topSpeedGpsInsPort.gps_snapshot = g_topSpeedGpsInsPort.gps_output;
    g_topSpeedGpsInsPort.snapshot_sequence++;
}

uint8_t TopSpeed_GPS_INS_PortCalibrateGyro(uint16_t sample_count)
{
    uint16_t index;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    if (!g_topSpeedGpsInsPort.imu_ready || (sample_count == 0U))
    {
        return 1U;
    }

    for (index = 0U; index < sample_count; index++)
    {
        imu963ra_get_gyro();
        sum_x += (float)imu963ra_gyro_x;
        sum_y += (float)imu963ra_gyro_y;
        sum_z += (float)imu963ra_gyro_z;
        system_delay_ms(TOPSPEED_GPS_INS_PERIOD_MS);
    }

    g_topSpeedGpsInsPort.gyro_bias_x_raw = sum_x / (float)sample_count;
    g_topSpeedGpsInsPort.gyro_bias_y_raw = sum_y / (float)sample_count;
    g_topSpeedGpsInsPort.gyro_bias_z_raw = sum_z / (float)sample_count;
    TopSpeed_GPS_INS_PortResetQuaternion();
    return 0U;
}

void TopSpeed_GPS_INS_PortSetGyroBiasRaw(float x, float y, float z)
{
    g_topSpeedGpsInsPort.gyro_bias_x_raw = x;
    g_topSpeedGpsInsPort.gyro_bias_y_raw = y;
    g_topSpeedGpsInsPort.gyro_bias_z_raw = z;
}

uint8_t TopSpeed_GPS_INS_PortInit(uint8_t use_gps,
                                  uint8_t navigation_solution)
{
    TopSpeed_GPS_INS_Config config;
    TopSpeed_GPS_INS_Point local_origin = {0.0f, 0.0f};
    uint8_t imu_state;

    memset(&g_topSpeedGpsInsPort, 0, sizeof(g_topSpeedGpsInsPort));
    if((navigation_solution < NAVIGATION_SOLUTION_MAHONY) ||
       (navigation_solution > NAVIGATION_SOLUTION_GPS_INS))
    {
        /* 非法选择不允许静默运行，返回独立错误码供启动日志定位。 */
        return 2U;
    }
    g_topSpeedGpsInsPort.navigation_solution = navigation_solution;
    g_topSpeedGpsInsPort.gps_enabled = (use_gps != 0U) ? 1U : 0U;
    g_topSpeedGpsInsPort.gps_output.enabled =
        g_topSpeedGpsInsPort.gps_enabled;
    g_topSpeedGpsInsPort.heading_sign = TOPSPEED_GPS_INS_IMU_HEADING_SIGN;
    g_topSpeedGpsInsPort.heading_offset_deg =
        TOPSPEED_GPS_INS_IMU_HEADING_OFFSET_DEG;
    TopSpeed_GPS_INS_PortResetQuaternion();

    TopSpeed_GPS_INS_GetDefaultConfig(&config);
    config.gps_delay_s = TOPSPEED_GPS_INS_GPS_DELAY_S;
    TopSpeed_GPS_INS_Init(&config);

    /*
     * With GPS disabled there will never be a first GNSS fix to establish the
     * local position.  Start at (0, 0) explicitly so IMU heading plus signed
     * rear-wheel encoder speed can immediately perform dead reckoning.
     */
    if (!g_topSpeedGpsInsPort.gps_enabled)
    {
        TopSpeed_GPS_INS_StartDeadReckoning(local_origin);
    }

    imu_state = imu963ra_init();
    g_topSpeedGpsInsPort.imu_ready = (uint8_t)(imu_state == 0U);
    if (g_topSpeedGpsInsPort.imu_ready)
    {
        (void)TopSpeed_GPS_INS_PortCalibrateGyro(
            TOPSPEED_GPS_INS_GYRO_CALIBRATION_SAMPLES);
    }

    if (g_topSpeedGpsInsPort.gps_enabled)
    {
        gnss_init(TAU1201);
        gnss_flag = 0U;
    }
    g_topSpeedGpsInsPort.initialized = 1U;
    TopSpeed_GPS_INS_PortPublishSnapshot();
    return imu_state;
}

uint8_t TopSpeed_GPS_INS_PortGetNavigationSolution(void)
{
    return g_topSpeedGpsInsPort.navigation_solution;
}

const char *TopSpeed_GPS_INS_PortGetNavigationSolutionName(void)
{
    if(g_topSpeedGpsInsPort.navigation_solution == NAVIGATION_SOLUTION_MAHONY)
    {
        return "MAHONY";
    }
    if(g_topSpeedGpsInsPort.navigation_solution == NAVIGATION_SOLUTION_FUSION)
    {
        return "FUSION";
    }
    if(g_topSpeedGpsInsPort.navigation_solution == NAVIGATION_SOLUTION_GPS_INS)
    {
        return "GPS_INS";
    }
    return "INVALID";
}

void TopSpeed_GPS_INS_PortSetHeadingAlignment(float sign, float offset_deg)
{
    if (TopSpeed_GPS_INS_PortIsFiniteFloat(sign) && (fabsf(sign) > 0.0001f))
    {
        g_topSpeedGpsInsPort.heading_sign = (sign > 0.0f) ? 1.0f : -1.0f;
    }
    if (TopSpeed_GPS_INS_PortIsFiniteFloat(offset_deg))
    {
        g_topSpeedGpsInsPort.heading_offset_deg = offset_deg;
    }
}

void TopSpeed_GPS_INS_PortSetOrigin(double latitude_deg, double longitude_deg)
{
    if (!TopSpeed_GPS_INS_PortIsFiniteDouble(latitude_deg) ||
        !TopSpeed_GPS_INS_PortIsFiniteDouble(longitude_deg) ||
        (latitude_deg < -90.0) || (latitude_deg > 90.0) ||
        (longitude_deg < -180.0) || (longitude_deg > 180.0))
    {
        return;
    }

    g_topSpeedGpsInsPort.origin_latitude_deg = latitude_deg;
    g_topSpeedGpsInsPort.origin_longitude_deg = longitude_deg;
    g_topSpeedGpsInsPort.origin_cos_latitude = cosf(
        (float)latitude_deg * (TOPSPEED_GPS_INS_PORT_PI / 180.0f));
    g_topSpeedGpsInsPort.origin_valid = 1U;
    g_topSpeedGpsInsPort.gps_pending.ready = 0U;
    g_topSpeedGpsInsPort.reset_pending = 1U;
}

void TopSpeed_GPS_INS_PortResetOrigin(void)
{
    g_topSpeedGpsInsPort.origin_valid = 0U;
    g_topSpeedGpsInsPort.gps_pending.ready = 0U;
    g_topSpeedGpsInsPort.reset_pending = 1U;
}

uint8_t TopSpeed_GPS_INS_PortHasOrigin(void)
{
    return g_topSpeedGpsInsPort.origin_valid;
}

void TopSpeed_GPS_INS_PortGPSLocalUpdate(float east_m,
                                        float north_m,
                                        float speed_mps,
                                        uint8_t valid)
{
    boolean interrupt_state = IfxCpu_disableInterrupts();

    g_topSpeedGpsInsPort.gps_pending.ready = 0U;
    g_topSpeedGpsInsPort.gps_pending.position_m.x = east_m;
    g_topSpeedGpsInsPort.gps_pending.position_m.y = north_m;
    g_topSpeedGpsInsPort.gps_pending.speed_mps = speed_mps;
    g_topSpeedGpsInsPort.gps_pending.valid = valid;
    g_topSpeedGpsInsPort.gps_pending.ready = 1U;
    IfxCpu_restoreInterrupts(interrupt_state);
}

void TopSpeed_GPS_INS_PortGPSGeodeticUpdate(double latitude_deg,
                                            double longitude_deg,
                                            float speed_mps,
                                            uint8_t valid)
{
    double delta_latitude;
    double delta_longitude;
    float east_m;
    float north_m;

    if (!valid ||
        !TopSpeed_GPS_INS_PortIsFiniteDouble(latitude_deg) ||
        !TopSpeed_GPS_INS_PortIsFiniteDouble(longitude_deg) ||
        !TopSpeed_GPS_INS_PortIsFiniteFloat(speed_mps) ||
        (latitude_deg < -90.0) || (latitude_deg > 90.0) ||
        (longitude_deg < -180.0) || (longitude_deg > 180.0))
    {
        TopSpeed_GPS_INS_PortGPSLocalUpdate(0.0f, 0.0f, 0.0f, 0U);
        return;
    }

    if (!g_topSpeedGpsInsPort.origin_valid)
    {
#if TOPSPEED_GPS_INS_AUTO_ORIGIN_ENABLE
        TopSpeed_GPS_INS_PortSetOrigin(latitude_deg, longitude_deg);
#else
        return;
#endif
    }

    delta_latitude = latitude_deg - g_topSpeedGpsInsPort.origin_latitude_deg;
    delta_longitude = longitude_deg - g_topSpeedGpsInsPort.origin_longitude_deg;
    east_m = (float)(delta_longitude * TOPSPEED_GPS_INS_METRE_PER_DEGREE) *
             g_topSpeedGpsInsPort.origin_cos_latitude;
    north_m = (float)(delta_latitude * TOPSPEED_GPS_INS_METRE_PER_DEGREE);
    TopSpeed_GPS_INS_PortGPSLocalUpdate(east_m, north_m, speed_mps, 1U);
}

void TopSpeed_GPS_INS_PortGnssPoll(void)
{
    double latitude;
    double longitude;
    boolean interrupt_state;

    if (!g_topSpeedGpsInsPort.gps_enabled || !gnss_flag)
    {
        return;
    }

    /* Clear before parsing so a UART event during parsing is not lost. */
    gnss_flag = 0U;
    if (gnss_data_parse())
    {
        return;
    }

    latitude = gnss.latitude;
    longitude = gnss.longitude;
    if (gnss.ns == 'S')
    {
        latitude = -latitude;
    }
    if (gnss.ew == 'W')
    {
        longitude = -longitude;
    }

    /* Publish signed geodetic values for the menu/debug snapshot. */
    interrupt_state = IfxCpu_disableInterrupts();
    g_topSpeedGpsInsPort.gps_output.latitude_deg = latitude;
    g_topSpeedGpsInsPort.gps_output.longitude_deg = longitude;
    g_topSpeedGpsInsPort.gps_output.speed_mps = gnss.speed / 3.6f;
    g_topSpeedGpsInsPort.gps_output.direction_deg = gnss.direction;
    g_topSpeedGpsInsPort.gps_output.satellite_used = gnss.satellite_used;
    g_topSpeedGpsInsPort.gps_output.valid = gnss.state;
    IfxCpu_restoreInterrupts(interrupt_state);

    TopSpeed_GPS_INS_PortGPSGeodeticUpdate(
        latitude,
        longitude,
        gnss.speed / 3.6f,
        gnss.state);
}

void TopSpeed_GPS_INS_PortEncoderSpeedUpdate(float speed_mps)
{
    boolean interrupt_state = IfxCpu_disableInterrupts();

    g_topSpeedGpsInsPort.encoder_pending.ready = 0U;
    g_topSpeedGpsInsPort.encoder_pending.speed_mps = speed_mps;
    g_topSpeedGpsInsPort.encoder_pending.valid = 1U;
    g_topSpeedGpsInsPort.encoder_pending.ready = 1U;
    IfxCpu_restoreInterrupts(interrupt_state);
}

void TopSpeed_GPS_INS_PortEncoderInvalidate(void)
{
    boolean interrupt_state = IfxCpu_disableInterrupts();

    g_topSpeedGpsInsPort.encoder_pending.ready = 0U;
    g_topSpeedGpsInsPort.encoder_pending.speed_mps = 0.0f;
    g_topSpeedGpsInsPort.encoder_pending.valid = 0U;
    g_topSpeedGpsInsPort.encoder_pending.ready = 1U;
    IfxCpu_restoreInterrupts(interrupt_state);
}

void TopSpeed_GPS_INS_PortEncoderDeltaUpdate(int32_t delta_count,
                                             float metre_per_count,
                                             float sample_period_s)
{
    float speed_mps;

    if (!TopSpeed_GPS_INS_PortIsFiniteFloat(metre_per_count) ||
        !TopSpeed_GPS_INS_PortIsFiniteFloat(sample_period_s) ||
        (metre_per_count <= 0.0f) || (sample_period_s <= 0.0f))
    {
        TopSpeed_GPS_INS_PortEncoderInvalidate();
        return;
    }

    speed_mps = (float)delta_count * metre_per_count / sample_period_s;
    TopSpeed_GPS_INS_PortEncoderSpeedUpdate(speed_mps);
}

void TopSpeed_GPS_INS_Port5msCallback(void)
{
    TopSpeed_GPS_INS_EncoderPending encoder;
    TopSpeed_GPS_INS_GPSPending gps;
    boolean interrupt_state;
    uint8_t encoder_ready = 0U;
    uint8_t gps_ready = 0U;

    if (!g_topSpeedGpsInsPort.initialized)
    {
        return;
    }

    if (g_topSpeedGpsInsPort.reset_pending)
    {
        TopSpeed_GPS_INS_Reset();
        g_topSpeedGpsInsPort.reset_pending = 0U;
    }

    interrupt_state = IfxCpu_disableInterrupts();
    if (g_topSpeedGpsInsPort.encoder_pending.ready)
    {
        encoder.speed_mps = g_topSpeedGpsInsPort.encoder_pending.speed_mps;
        encoder.valid = g_topSpeedGpsInsPort.encoder_pending.valid;
        g_topSpeedGpsInsPort.encoder_pending.ready = 0U;
        encoder_ready = 1U;
    }
    IfxCpu_restoreInterrupts(interrupt_state);
    if (encoder_ready)
    {
        TopSpeed_GPS_INS_UpdateEncoderSpeed(encoder.speed_mps, encoder.valid);
    }

    if (g_topSpeedGpsInsPort.imu_ready)
    {
        TopSpeed_GPS_INS_PortReadIMU();
        /* TopSpeed vehicle X-forward acceleration is mounted on -IMU Y. */
        TopSpeed_GPS_INS_UpdateIMU(
            g_topSpeedGpsInsPort.imu_output.heading_deg,
            -g_topSpeedGpsInsPort.imu_output.acc_y_mps2,
            TOPSPEED_GPS_INS_PERIOD_S);
    }
    else
    {
        TopSpeed_GPS_INS_Tick(TOPSPEED_GPS_INS_PERIOD_S);
    }

    interrupt_state = IfxCpu_disableInterrupts();
    if (g_topSpeedGpsInsPort.gps_pending.ready)
    {
        gps.position_m = g_topSpeedGpsInsPort.gps_pending.position_m;
        gps.speed_mps = g_topSpeedGpsInsPort.gps_pending.speed_mps;
        gps.valid = g_topSpeedGpsInsPort.gps_pending.valid;
        g_topSpeedGpsInsPort.gps_pending.ready = 0U;
        gps_ready = 1U;
    }
    IfxCpu_restoreInterrupts(interrupt_state);
    if (gps_ready)
    {
        TopSpeed_GPS_INS_UpdateGPS(gps.position_m, gps.speed_mps, gps.valid);
    }

    TopSpeed_GPS_INS_PortPublishSnapshot();
}

void TopSpeed_GPS_INS_PortGetOutput(TopSpeed_GPS_INS_Output *output)
{
    uint32_t sequence_start;
    uint32_t sequence_end;

    if (output == 0)
    {
        return;
    }

    for (;;)
    {
        sequence_start = g_topSpeedGpsInsPort.snapshot_sequence;
        if (sequence_start & 1U)
        {
            continue;
        }
        *output = g_topSpeedGpsInsPort.navigation_snapshot;
        sequence_end = g_topSpeedGpsInsPort.snapshot_sequence;
        if ((sequence_start == sequence_end) && !(sequence_end & 1U))
        {
            break;
        }
    }
}

void TopSpeed_GPS_INS_PortGetIMUOutput(TopSpeed_GPS_INS_IMU_Output *output)
{
    uint32_t sequence_start;
    uint32_t sequence_end;

    if (output == 0)
    {
        return;
    }

    for (;;)
    {
        sequence_start = g_topSpeedGpsInsPort.snapshot_sequence;
        if (sequence_start & 1U)
        {
            continue;
        }
        *output = g_topSpeedGpsInsPort.imu_snapshot;
        sequence_end = g_topSpeedGpsInsPort.snapshot_sequence;
        if ((sequence_start == sequence_end) && !(sequence_end & 1U))
        {
            break;
        }
    }
}

void TopSpeed_GPS_INS_PortGetGPSOutput(TopSpeed_GPS_INS_GPS_Output *output)
{
    uint32_t sequence_start;
    uint32_t sequence_end;

    if (output == 0)
    {
        return;
    }

    for (;;)
    {
        sequence_start = g_topSpeedGpsInsPort.snapshot_sequence;
        if (sequence_start & 1U)
        {
            continue;
        }
        *output = g_topSpeedGpsInsPort.gps_snapshot;
        sequence_end = g_topSpeedGpsInsPort.snapshot_sequence;
        if ((sequence_start == sequence_end) && !(sequence_end & 1U))
        {
            break;
        }
    }
}
