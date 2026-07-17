#ifndef MENU_HOST_STUBS_H
#define MENU_HOST_STUBS_H

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define _zf_common_headfile_h_
#define _ACKERMANN_CONTROL_H_
#define _MOTOR_H
#define _zf_device_ips200_h_
#define _zf_driver_gpio_h_
#define CODE_ZF_DEVICE_LORA3A22_H_

#define P33_4                 (0x334)
#define P33_5                 (0x335)
#define GPI                   (0)
#define GPI_PULL_UP           (1)
#define IPS200_TYPE_SPI       (0)
#define IPS200_PORTAIT_180    (1)
#define IPS200_8X16_FONT      (1)
#define RGB565_WHITE          (0xFFFFU)
#define RGB565_BLACK          (0x0000U)
#define ACKERMANN_PI          (3.14159265358979323846f)
#define ACKERMANN_DEG_TO_RAD  (ACKERMANN_PI / 180.0f)

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
    float wheel_radius_m;
    float left_motor_sign;
    float right_motor_sign;
    ACKERMANN_PID_GAIN steering_position_pid;
    ACKERMANN_PID_GAIN left_speed_pid;
    ACKERMANN_PID_GAIN right_speed_pid;
} ACKERMANN_CONTROL_CONFIG;

typedef struct
{
    float left_measured_rad_s;
    float right_measured_rad_s;
} ACKERMANN_CONTROL_TELEMETRY;

typedef struct
{
    float motor_speed;
    float feedforward_pwm;
} MOTOR_DUTY;

typedef ACKERMANN_PID_GAIN MOTOR_PID_GAIN;

typedef struct
{
    float error;
    float last_error;
    float last_measurement;
    float integral;
    float derivative;
    float output;
    uint8_t initialized;
} MOTOR_PID_STATE;

typedef struct
{
    MOTOR_PID_GAIN gain;
    MOTOR_PID_STATE state;
} MOTOR_PID;

typedef struct
{
    float target_rad_s;
    float measured_rad_s;
    float feedforward_pwm;
    float output_sign;
    float pwm_limit;
    uint8_t enabled;
} MOTOR_REAR_SPEED_INPUT;

typedef struct
{
    float target_rad_s;
    float measured_rad_s;
    float error_rad_s;
    float pwm;
} MOTOR_REAR_SPEED_STATUS;

typedef struct
{
    uint8_t head;
    uint8_t sum_check;
    int16_t joystick[4];
    uint8_t key[4];
    uint8_t switch_key[4];
} lora3a22_uart_transfer_dat_struct;

extern MOTOR_DUTY servo_duty;
extern MOTOR_PID servo_position_pid;

void Ackermann_control_enable(uint8_t enable);
void Ackermann_get_default_config(ACKERMANN_CONTROL_CONFIG *config);
void Ackermann_get_telemetry(ACKERMANN_CONTROL_TELEMETRY *telemetry);
float Ackermann_get_steering_center(void);
void Ackermann_electronic_differential(float center_speed_mps,
                                       float steering_angle_rad,
                                       const ACKERMANN_CONTROL_CONFIG *config,
                                       float *left_speed_mps,
                                       float *right_speed_mps);
void Ackermann_set_rear_speed_pid_gains(const ACKERMANN_PID_GAIN *left_gain,
                                        const ACKERMANN_PID_GAIN *right_gain);
void Ackermann_get_rear_speed_pid_gains(ACKERMANN_PID_GAIN *left_gain,
                                        ACKERMANN_PID_GAIN *right_gain);

void Motor_set_pwm(float left_pwm, float right_pwm);
void Motor_set_rear_speed_pid_gains(const MOTOR_PID_GAIN *left_gain,
                                    const MOTOR_PID_GAIN *right_gain);
void Motor_get_rear_speed_pid_gains(MOTOR_PID_GAIN *left_gain,
                                    MOTOR_PID_GAIN *right_gain);
void Motor_rear_speed_pid_reset(void);
void Motor_rear_speed_control(const MOTOR_REAR_SPEED_INPUT *left,
                              const MOTOR_REAR_SPEED_INPUT *right,
                              uint8_t allow_active_braking);
void Motor_get_rear_speed_status(MOTOR_REAR_SPEED_STATUS *left_status,
                                 MOTOR_REAR_SPEED_STATUS *right_status);
void Motor_enable_channels(uint8_t left, uint8_t right, uint8_t steering);
void Motor_stop_all(void);
void Servo_pid_reset(void);
void Servo_set_position_pid_gain(const MOTOR_PID_GAIN *gain);
void Servo_set_angle(float angle_deg);
void Servo_position_control(MOTOR_DUTY *duty);
float Servo_get_angle(void);

uint8_t lora3a22_get_snapshot(lora3a22_uart_transfer_dat_struct *snapshot,
                              uint32 *sequence);
void lora3a22_init(void);
void lora3a22_5ms_callback(void);

void gpio_init(int pin, int dir, uint8_t level, int mode);
uint8_t gpio_get_level(int pin);

void ips200_init(int type);
void ips200_set_dir(int dir);
void ips200_set_font(int font);
void ips200_set_color(uint16_t pen, uint16_t background);
void ips200_clear(void);
void ips200_show_string(uint16_t x, uint16_t y, const char text[]);
void ips200_show_int(uint16_t x, uint16_t y, int32_t value, uint8_t digits);
void ips200_show_uint(uint16_t x, uint16_t y, uint32_t value, uint8_t digits);
void ips200_show_float(uint16_t x,
                       uint16_t y,
                       double value,
                       uint8_t digits,
                       uint8_t decimals);

#include "lora_remote.h"
#include "TopSpeed_GPS_INS_Port.h"
#include "board_mode_switch.h"
#include "remote_mode.h"
#include "test_mode.h"
#include "Menu.h"

#endif
