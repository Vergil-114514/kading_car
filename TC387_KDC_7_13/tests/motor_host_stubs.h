#ifndef MOTOR_HOST_STUBS_H
#define MOTOR_HOST_STUBS_H

#include <stdint.h>

/* 阻止 motor.c 拉入 TC387 专用头文件，仅为主机侧 PID 单元测试提供最小类型。 */
#define _zf_common_headfile_h_
#define _MOTOR_H

typedef uint8_t uint8;
typedef uint32_t uint32;
typedef int pwm_channel_enum;
typedef int gpio_pin_enum;

#define ZF_FALSE             (0U)
#define ZF_TRUE              (1U)
#define PWM_DUTY_MAX         (10000U)
#define GPO                  (0)
#define GPO_PUSH_PULL        (0)

#define MOTOR_PWM_FREQ_HZ    (10000U)
#define MOTOR_CONTROL_HZ     (200.0f)
#define MOTOR_CONTROL_DT     (1.0f / MOTOR_CONTROL_HZ)
#define MOTOR_FULL_ANGLE_DEG (360.0f)
#define ENABLE               (1)
#define DISABLE              (0)

#define motor_L_ENABLE       (1)
#define motor_LA1            (2)
#define motor_LA2            (3)
#define motor_LB1            (4)
#define motor_LB2            (5)
#define motor_R_ENABLE       (6)
#define motor_RA1            (7)
#define motor_RA2            (8)
#define motor_RB1            (9)
#define motor_RB2            (10)
#define motor_T_ENABLE       (11)
#define motor_TA1            (12)
#define motor_TA2            (13)
#define motor_TB1            (14)
#define motor_TB2            (15)

#define motor_L_DIS          motor_L_ENABLE
#define motor_L_PWM1         motor_LA1
#define motor_L_PWM1N        motor_LA2
#define motor_L_PWM2         motor_LB1
#define motor_L_PWM2N        motor_LB2
#define motor_R_DIS          motor_R_ENABLE
#define motor_R_PWM1         motor_RA1
#define motor_R_PWM1N        motor_RA2
#define motor_R_PWM2         motor_RB1
#define motor_R_PWM2N        motor_RB2
#define motor_T_DIS          motor_T_ENABLE
#define motor_T_PWM1         motor_TA1
#define motor_T_PWM1N        motor_TA2
#define motor_T_PWM2         motor_TB1
#define motor_T_PWM2N        motor_TB2

typedef struct
{
    pwm_channel_enum top_pin;
    pwm_channel_enum bottom_pin;
} MOTOR_PWM_HALF_BRIDGE;

typedef struct
{
    gpio_pin_enum enable_pin;
    MOTOR_PWM_HALF_BRIDGE phase_a;
    MOTOR_PWM_HALF_BRIDGE phase_b;
    uint32 pwm_freq_hz;
    float pwm_limit;
} MOTOR_DRIVER;

typedef struct
{
    float motor_speed;
    float feedforward_pwm;
} MOTOR_DUTY;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    float deadband;
} MOTOR_PID_GAIN;

typedef struct
{
    float error;
    float last_error;
    float last_measurement;
    float integral;
    float derivative;
    float output;
    uint8 initialized;
} MOTOR_PID_STATE;

typedef struct
{
    MOTOR_PID_GAIN gain;
    MOTOR_PID_STATE state;
} MOTOR_PID;

typedef struct
{
    MOTOR_PID position;
    MOTOR_PID speed;
    float target_angle_deg;
    float target_speed_dps;
    float angle_deg;
    float speed_dps;
} MOTOR_CASCADE_PID;

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

extern MOTOR_DUTY motor_L_duty;
extern MOTOR_DUTY motor_R_duty;
extern MOTOR_DUTY servo_duty;
extern MOTOR_CASCADE_PID servo_pid;
extern MOTOR_PID servo_position_pid;
extern float car_speed[3];
extern float car_angle;

void Motor_get_default_pid_gains(MOTOR_PID_GAIN *steering_position_gain,
                                 MOTOR_PID_GAIN *left_rear_speed_gain,
                                 MOTOR_PID_GAIN *right_rear_speed_gain);

void pwm_hl_init(pwm_channel_enum top, pwm_channel_enum bottom,
                 uint32 frequency, uint32 duty);
void pwm_hl_set_duty(pwm_channel_enum top, uint32 duty);
void gpio_init(gpio_pin_enum pin, int direction, uint8 level, int mode);
void gpio_set_level(gpio_pin_enum pin, uint8 level);

void motor_init(void);
void Motor_set_rear_speed_pid_gains(const MOTOR_PID_GAIN *left_gain,
                                    const MOTOR_PID_GAIN *right_gain);
void Motor_get_rear_speed_pid_gains(MOTOR_PID_GAIN *left_gain,
                                    MOTOR_PID_GAIN *right_gain);
void Motor_rear_speed_pid_reset(void);
void Motor_rear_speed_control(const MOTOR_REAR_SPEED_INPUT *left,
                              const MOTOR_REAR_SPEED_INPUT *right,
                              uint8 allow_active_braking);
void Motor_get_rear_speed_status(MOTOR_REAR_SPEED_STATUS *left_status,
                                 MOTOR_REAR_SPEED_STATUS *right_status);
void Motor_enable_channels(uint8 left_enable,
                           uint8 right_enable,
                           uint8 steering_enable);
void Servo_init(void);
void Servo_set_angle(float angle_deg);
void Servo_pid_reset(void);

#endif
