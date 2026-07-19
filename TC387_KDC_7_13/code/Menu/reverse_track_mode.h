#ifndef _REVERSE_TRACK_MODE_H_
#define _REVERSE_TRACK_MODE_H_

#include <stdint.h>

/*
 * Compile-time route direction selector:
 * - FORWARD: start at point 0 and drive to the final stored point;
 * - REVERSE: start at the final stored point and reverse to point 0.
 * Keep the vehicle at the selected endpoint with its recorded heading before
 * resetting into TRACK mode.
 */
#define TRACK_DIRECTION_FORWARD                   (1U)
#define TRACK_DIRECTION_REVERSE                   (2U)
#ifndef TRACK_MODE_DIRECTION
#define TRACK_MODE_DIRECTION                      (TRACK_DIRECTION_FORWARD)
#endif

#if (TRACK_MODE_DIRECTION != TRACK_DIRECTION_FORWARD) && \
    (TRACK_MODE_DIRECTION != TRACK_DIRECTION_REVERSE)
    #error "TRACK_MODE_DIRECTION must be FORWARD or REVERSE"
#endif

/*
 * Forward-route automatic parking configuration.  Coordinates and yaw use
 * the saved route frame: point 0 is (0,0), +Y is the launch direction, +X is
 * vehicle-right, and positive yaw turns from +Y toward +X.
 *
 * Set TRACK_PARK_TARGET_CONFIGURED to 1U only after replacing X/Y/YAW with
 * the real garage pose.  A forward build deliberately fails otherwise.
 */
#ifndef TRACK_FORWARD_PARK_ENABLE
#define TRACK_FORWARD_PARK_ENABLE                 (1U)
#endif
#ifndef TRACK_PARK_TARGET_CONFIGURED
#define TRACK_PARK_TARGET_CONFIGURED              (1U)
#endif
#ifndef TRACK_PARK_TARGET_X_M
#define TRACK_PARK_TARGET_X_M                     (-7.0f)
#endif
#ifndef TRACK_PARK_TARGET_Y_M
#define TRACK_PARK_TARGET_Y_M                     (0.5f)
#endif
#ifndef TRACK_PARK_TARGET_YAW_DEG
#define TRACK_PARK_TARGET_YAW_DEG                 (0.0f)
#endif

#define TRACK_PARK_MAX_REVERSE_SPEED_MPS          (0.5f)
#define TRACK_PARK_POSITION_TOLERANCE_M           (0.15f)
#define TRACK_PARK_HEADING_TOLERANCE_DEG          (5.0f)
#define TRACK_PARK_ENTRY_STOP_SPEED_MPS           (0.05f)
#define TRACK_PARK_LOOKAHEAD_BASE_M               (0.25f)
#define TRACK_PARK_LOOKAHEAD_TIME_S               (0.50f)
#define TRACK_PARK_LOOKAHEAD_MIN_M                (0.25f)
#define TRACK_PARK_LOOKAHEAD_MAX_M                (0.60f)
#define TRACK_PARK_SLOWDOWN_DISTANCE_M             (1.00f)
#define TRACK_PARK_BEZIER_HANDLE_RATIO            (0.50f)
#define TRACK_PARK_BEZIER_HANDLE_MIN_M            (0.25f)
#define TRACK_PARK_BEZIER_HANDLE_MAX_M            (2.00f)
#define TRACK_PARK_MIN_PATH_DISTANCE_M            (0.30f)
#define TRACK_PARK_PATH_END_EPSILON_M             (0.02f)
#define TRACK_PARK_PATH_POINT_COUNT               (17U)

#if (TRACK_FORWARD_PARK_ENABLE != 0U) && \
    (TRACK_FORWARD_PARK_ENABLE != 1U)
    #error "TRACK_FORWARD_PARK_ENABLE must be 0U or 1U"
#endif
#if (TRACK_PARK_PATH_POINT_COUNT < 2U) || \
    (TRACK_PARK_PATH_POINT_COUNT > 1024U)
    #error "TRACK_PARK_PATH_POINT_COUNT is out of range"
#endif
#if (TRACK_MODE_DIRECTION == TRACK_DIRECTION_FORWARD) && \
    (TRACK_FORWARD_PARK_ENABLE != 0U) && \
    (TRACK_PARK_TARGET_CONFIGURED == 0U)
    #error "Set TRACK parking X/Y/YAW and TRACK_PARK_TARGET_CONFIGURED=1U"
#endif

/* Route pure-pursuit tuning defaults shared by both directions. */
#define REVERSE_TRACK_MAX_SPEED_MPS              (4.0f)
#define REVERSE_TRACK_ACCELERATION_MPS2          (1.5f)
#define REVERSE_TRACK_MAX_LATERAL_ACCEL_MPS2     (1.5f)
#define REVERSE_TRACK_STOP_DISTANCE_M            (0.50f)
#define REVERSE_TRACK_LOOKAHEAD_BASE_M           (0.75f)
#define REVERSE_TRACK_LOOKAHEAD_TIME_S           (0.75f)
#define REVERSE_TRACK_LOOKAHEAD_MIN_M            (0.75f)
#define REVERSE_TRACK_LOOKAHEAD_MAX_M            (3.0f)
#define REVERSE_TRACK_MAX_STEERING_DEG           (20.0f)
#define REVERSE_TRACK_MAX_STEERING_RATE_DPS      (120.0f)
#define REVERSE_TRACK_PWM_LIMIT                  (8000.0f)
#define REVERSE_TRACK_SEGMENT_SEARCH_COUNT       (6U)
#define REVERSE_TRACK_SEGMENT_EPSILON_M          (0.01f)

typedef enum
{
    REVERSE_TRACK_STATE_IDLE = 0,
    REVERSE_TRACK_STATE_WAIT_NAV,
    REVERSE_TRACK_STATE_RUNNING,
    REVERSE_TRACK_STATE_WAIT_PARK_STOP,
    REVERSE_TRACK_STATE_PARKING,
    REVERSE_TRACK_STATE_WAIT_PARK_COMPLETE,
    REVERSE_TRACK_STATE_COMPLETED,
    REVERSE_TRACK_STATE_ROUTE_FAULT,
    REVERSE_TRACK_STATE_SENSOR_FAULT
} ReverseTrackModeState_e;

typedef enum
{
    REVERSE_TRACK_FAULT_NONE = 0,
    REVERSE_TRACK_FAULT_POINT_COUNT,
    REVERSE_TRACK_FAULT_POINT_VALUE,
    REVERSE_TRACK_FAULT_ZERO_LENGTH_ROUTE,
    REVERSE_TRACK_FAULT_NAVIGATION,
    REVERSE_TRACK_FAULT_ENCODER,
    REVERSE_TRACK_FAULT_PATH_PROJECTION,
    REVERSE_TRACK_FAULT_PARK_CONFIG,
    REVERSE_TRACK_FAULT_PARK_POSE
} ReverseTrackFault_e;

typedef struct
{
    ReverseTrackModeState_e state;
    ReverseTrackFault_e fault;
    uint16_t point_count;
    uint16_t segment_index;
    uint8_t route_valid;
    uint8_t aligned;
    uint8_t outputs_enabled;
    uint8_t parking_active;

    float local_x_m;
    float local_y_m;
    float remaining_distance_m;
    float lookahead_x_m;
    float lookahead_y_m;
    float lookahead_distance_m;
    float curvature_inv_m;
    float target_speed_mps;
    float ramped_speed_mps;
    float steering_target_rad;
    float steering_encoder_target_deg;

    float left_target_mps;
    float left_measured_mps;
    float left_pwm;
    float right_target_mps;
    float right_measured_mps;
    float right_pwm;
    float parking_position_error_m;
    float parking_heading_error_deg;
} ReverseTrackModeStatus_t;

/** Initialize RAM state. This is the only call that clears a latched stop. */
void ReverseTrackMode_Init(void);
/** Load the saved route and enter the automatic TRACK-mode startup sequence. */
void ReverseTrackMode_Enter(void);
/** Reserved non-real-time task; flash access remains outside the 5 ms ISR. */
void ReverseTrackMode_Task(void);
/** Execute alignment, pure pursuit, electronic differential and motor PID. */
void ReverseTrackMode_5msCallback(void);
/** Disable all outputs. A terminal latch is deliberately not cleared. */
void ReverseTrackMode_Exit(void);
/** Copy the latest state for diagnostics and host tests. */
void ReverseTrackMode_GetStatus(ReverseTrackModeStatus_t *status);

#endif
