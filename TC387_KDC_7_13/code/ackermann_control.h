#ifndef _ACKERMANN_CONTROL_H_
#define _ACKERMANN_CONTROL_H_

#include <stdint.h>

/* Measured distance between axles and between the rear-wheel contact centres. */
#define ACKERMANN_WHEELBASE_M                  (0.58f)
#define ACKERMANN_TRACK_WIDTH_M                (0.60f)
#define ACKERMANN_MAX_WHEEL_SPEED_MPS          (3.0f)
#define ACKERMANN_CONTROL_PERIOD_S             (0.005f)
#define ACKERMANN_CONTROL_PERIOD_MS            (5U)

/* Steering calibration verified on the vehicle: raw 876 is straight ahead,
 * and measured encoder angle increases when the front wheels turn left. */
#define ACKERMANN_STEERING_CENTER_ENCODER_DEG  (76.9921875f)
#define ACKERMANN_STEERING_ENCODER_RATIO       (1.0f)
#define ACKERMANN_STEERING_SIGN                (1.0f)

#define ACKERMANN_PI                           (3.14159265358979323846f)
#define ACKERMANN_DEG_TO_RAD                   (ACKERMANN_PI / 180.0f)
#define ACKERMANN_RAD_TO_DEG                   (180.0f / ACKERMANN_PI)

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    float deadband;
} ACKERMANN_PID_GAIN;

typedef struct
{
    float wheelbase_m;
    float track_width_m;
    float control_period_s;

    /* Encoder sign corrects the signed linear speed produced by encoder.c. */
    float left_encoder_sign;
    float right_encoder_sign;
    float left_motor_sign;
    float right_motor_sign;

    float stanley_gain;
    float stanley_softening_speed_mps;
    float max_cross_track_error_m;
    float max_steering_angle_rad;
    float max_steering_rate_rad_s;

    float max_vehicle_speed_mps;
    float max_acceleration_mps2;
    float max_wheel_speed_mps;
    float stop_speed_threshold_mps;
    float command_timeout_s;
    uint8_t allow_active_braking;

    /* Steering encoder angle at the straight-ahead position. A negative
     * value captures the current incremental-encoder position during init. */
    float steering_center_encoder_deg;
    /* Encoder degrees per road-wheel steering degree. */
    float steering_encoder_deg_per_road_deg;
    float steering_sign;

    ACKERMANN_PID_GAIN steering_position_pid;
    ACKERMANN_PID_GAIN left_speed_pid;
    ACKERMANN_PID_GAIN right_speed_pid;
    float left_feedforward_pwm_per_mps;
    float right_feedforward_pwm_per_mps;
} ACKERMANN_CONTROL_CONFIG;

typedef struct
{
    /* Positive cross-track error means the path is to the vehicle's left.
     * Positive heading error means path heading is counter-clockwise from
     * vehicle heading. Angles are radians. Forward speed only. */
    float cross_track_error_m;
    float heading_error_rad;
    float target_speed_mps;
    uint8_t valid;
} ACKERMANN_PATH_COMMAND;

typedef struct
{
    float cross_track_error_m;
    float heading_error_rad;
    float commanded_speed_mps;
    float ramped_speed_mps;
    float measured_vehicle_speed_mps;

    float steering_target_rad;
    float steering_encoder_target_deg;
    float steering_encoder_measured_deg;

    float left_target_mps;
    float right_target_mps;
    float left_measured_mps;
    float right_measured_mps;
    float left_pwm;
    float right_pwm;

    float command_age_s;
    uint8_t enabled;
    uint8_t path_valid;
    uint8_t encoder_valid;
    uint8_t command_timed_out;
} ACKERMANN_CONTROL_TELEMETRY;

void Ackermann_get_default_config(ACKERMANN_CONTROL_CONFIG *config);

/* Returns 0 on success. Hardware remains disabled until
 * Ackermann_control_enable(ZF_TRUE) is called. */
uint8_t Ackermann_control_init(const ACKERMANN_CONTROL_CONFIG *config);

void Ackermann_control_enable(uint8_t enable);
void Ackermann_control_reset(void);

/* Call this whenever a new path-tracking result is available. The default
 * 0.2 s watchdog requires the command to be refreshed periodically. */
void Ackermann_set_path_command(const ACKERMANN_PATH_COMMAND *command);
void Ackermann_set_path_error(float cross_track_error_m,
                              float heading_error_rad,
                              float target_speed_mps);
void Ackermann_invalidate_path(void);

/* left/right_measured_mps come directly from encoder.c. This controller no
 * longer reads counts, converts CPR, or requires a wheel radius. */
void Ackermann_control_step(float left_measured_mps,
                            float right_measured_mps,
                            uint8_t encoder_feedback_valid);

void Ackermann_set_steering_center(float encoder_angle_deg);
float Ackermann_get_steering_center(void);
void Ackermann_get_telemetry(ACKERMANN_CONTROL_TELEMETRY *telemetry);

/* Runtime gain access used by the LoRa PID tuning mode. Gain changes reset
 * the corresponding controller state and remain in RAM until power-off. */
void Ackermann_set_rear_speed_pid_gains(const ACKERMANN_PID_GAIN *left_gain,
                                        const ACKERMANN_PID_GAIN *right_gain);
void Ackermann_get_rear_speed_pid_gains(ACKERMANN_PID_GAIN *left_gain,
                                        ACKERMANN_PID_GAIN *right_gain);

/* Pure model helpers, also useful for offline tests. */
float Ackermann_stanley_steering(float cross_track_error_m,
                                 float heading_error_rad,
                                 float vehicle_speed_mps,
                                 const ACKERMANN_CONTROL_CONFIG *config);
void Ackermann_electronic_differential(float center_speed_mps,
                                       float steering_angle_rad,
                                       const ACKERMANN_CONTROL_CONFIG *config,
                                       float *left_speed_mps,
                                       float *right_speed_mps);
float Ackermann_road_steering_to_encoder_angle_deg(
    float road_steering_rad,
    const ACKERMANN_CONTROL_CONFIG *config);
float Ackermann_encoder_angle_to_road_steering_rad(
    float encoder_angle_deg,
    const ACKERMANN_CONTROL_CONFIG *config);

#endif
