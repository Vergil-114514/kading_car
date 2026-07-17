#ifndef _TOPSPEED_GPS_INS_H_
#define _TOPSPEED_GPS_INS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Coordinate convention used by this port:
 *   x: east (m), y: north (m)
 *   heading: 0 deg = north, +90 deg = east (clockwise positive)
 */
typedef struct
{
    float x;
    float y;
} TopSpeed_GPS_INS_Point;

typedef enum
{
    TOPSPEED_GPS_INS_SPEED_NONE = 0,
    TOPSPEED_GPS_INS_SPEED_GPS,
    TOPSPEED_GPS_INS_SPEED_ENCODER
} TopSpeed_GPS_INS_SpeedSource;

typedef struct
{
    float gps_delay_s;              /* TAU1201 measurement/transport delay. */
    float gps_timeout_s;            /* GPS speed is stale after this time. */
    float encoder_timeout_s;        /* Encoder speed is stale after this time. */
    float gps_jump_gate_m;          /* 0 disables position jump rejection. */
    float max_abs_speed_mps;        /* Reject impossible GPS/encoder speed. */
} TopSpeed_GPS_INS_Config;

typedef struct
{
    TopSpeed_GPS_INS_Point position_m;
    TopSpeed_GPS_INS_Point gps_position_m;
    TopSpeed_GPS_INS_Point compensated_gps_position_m;

    float speed_mps;
    float gps_speed_mps;
    float encoder_speed_mps;
    float heading_deg;
    float forward_acc_mps2;
    float gps_age_s;
    float encoder_age_s;

    uint32_t imu_update_count;
    uint32_t gps_update_count;
    uint32_t encoder_update_count;
    uint32_t gps_reject_count;

    TopSpeed_GPS_INS_SpeedSource speed_source;
    uint8_t initialized;
    uint8_t valid;
    uint8_t heading_valid;
    uint8_t gps_valid;
    uint8_t encoder_valid;
} TopSpeed_GPS_INS_Output;

void TopSpeed_GPS_INS_GetDefaultConfig(TopSpeed_GPS_INS_Config *config);
void TopSpeed_GPS_INS_Init(const TopSpeed_GPS_INS_Config *config);
void TopSpeed_GPS_INS_Reset(void);

/*
 * Start GPS-free dead reckoning from a known local east/north coordinate.
 * This only establishes the local position; valid IMU heading and encoder
 * speed are still required before the estimator can advance the vehicle.
 */
void TopSpeed_GPS_INS_StartDeadReckoning(
    TopSpeed_GPS_INS_Point initial_position_m);

/* Advance the estimator when no new IMU heading is available. */
void TopSpeed_GPS_INS_Tick(float dt_s);

/* Feed the latest heading and advance the estimator by dt_s. */
void TopSpeed_GPS_INS_UpdateIMU(float heading_deg,
                               float forward_acc_mps2,
                               float dt_s);

/* GPS position must already be converted to local east/north metres. */
void TopSpeed_GPS_INS_UpdateGPS(TopSpeed_GPS_INS_Point position_m,
                               float speed_mps,
                               uint8_t valid);

/* Encoder speed is signed: forward positive, reverse negative. */
void TopSpeed_GPS_INS_UpdateEncoderSpeed(float speed_mps, uint8_t valid);

TopSpeed_GPS_INS_Output TopSpeed_GPS_INS_GetOutput(void);

#ifdef __cplusplus
}
#endif

#endif
