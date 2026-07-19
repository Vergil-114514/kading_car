#ifndef REVERSE_TRACK_MODE_HOST_STUBS_H
#define REVERSE_TRACK_MODE_HOST_STUBS_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define _zf_common_headfile_h_

#define FLASH_IMU_POINT_CAPACITY              (1024U)
#define ACKERMANN_WHEELBASE_M                  (0.58f)
#define ACKERMANN_TRACK_WIDTH_M                (0.60f)
#define ACKERMANN_MAX_WHEEL_SPEED_MPS          (3.0f)
#define ACKERMANN_CONTROL_PERIOD_S             (0.005f)
#define ACKERMANN_PI                           (3.14159265358979323846f)
#define ACKERMANN_DEG_TO_RAD                   (ACKERMANN_PI / 180.0f)
#define ACKERMANN_RAD_TO_DEG                   (180.0f / ACKERMANN_PI)

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;

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
    uint8 allow_active_braking;
    float steering_center_encoder_deg;
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
    uint8 enabled;
    uint8 path_valid;
    uint8 encoder_valid;
    uint8 command_timed_out;
} ACKERMANN_CONTROL_TELEMETRY;

typedef struct
{
    float motor_speed;
    float feedforward_pwm;
} MOTOR_DUTY;

typedef struct
{
    float target_mps;
    float measured_mps;
    float feedforward_pwm;
    float output_sign;
    float pwm_limit;
    uint8 enabled;
} MOTOR_REAR_SPEED_INPUT;

typedef struct
{
    float target_mps;
    float measured_mps;
    float error_mps;
    float pwm;
} MOTOR_REAR_SPEED_STATUS;

typedef struct
{
    int16_t joystick[4];
    uint8 key[4];
    uint8 switch_key[4];
    uint32 sequence;
    uint16 age_ms;
    uint8 link_ok;
} LoraRemoteState_t;

typedef struct
{
    float x;
    float y;
} TopSpeed_GPS_INS_Point;

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
    uint32 imu_update_count;
    uint32 gps_update_count;
    uint32 encoder_update_count;
    uint32 gps_reject_count;
    int speed_source;
    uint8 initialized;
    uint8 valid;
    uint8 heading_valid;
    uint8 gps_valid;
    uint8 encoder_valid;
} TopSpeed_GPS_INS_Output;

extern uint16 IMU_savenum;
extern float IMU_X[FLASH_IMU_POINT_CAPACITY];
extern float IMU_Y[FLASH_IMU_POINT_CAPACITY];
extern MOTOR_DUTY servo_duty;

void DATA_READ(void);
void IMU_POINT_READ(void);
int reverse_track_test_printf(const char *format, ...);
void LoraRemote_GetState(LoraRemoteState_t *state);
void TopSpeed_GPS_INS_PortGetOutput(TopSpeed_GPS_INS_Output *output);
void Ackermann_get_default_config(ACKERMANN_CONTROL_CONFIG *config);
void Ackermann_control_enable(uint8 enable);
void Ackermann_get_telemetry(ACKERMANN_CONTROL_TELEMETRY *telemetry);
float Ackermann_get_steering_center(void);
void Ackermann_electronic_differential(
    float center_speed_mps,
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
void Motor_rear_speed_pid_reset(void);
void Motor_rear_speed_control(const MOTOR_REAR_SPEED_INPUT *left,
                              const MOTOR_REAR_SPEED_INPUT *right,
                              uint8 allow_active_braking);
void Motor_get_rear_speed_status(MOTOR_REAR_SPEED_STATUS *left_status,
                                 MOTOR_REAR_SPEED_STATUS *right_status);
void Motor_enable_channels(uint8 left_enable,
                           uint8 right_enable,
                           uint8 steering_enable);
void Motor_stop_all(void);
void Servo_pid_reset(void);
void Servo_set_angle(float angle_deg);
void Servo_position_control(MOTOR_DUTY *duty);
float Servo_get_angle(void);

#define printf reverse_track_test_printf

#endif
