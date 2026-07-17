#ifndef ACKERMANN_HOST_TEST
#include "zf_common_headfile.h"
#else
#include <math.h>
#include "ackermann_control.h"
#define ZF_FALSE (0U)
#define ZF_TRUE  (1U)
typedef ACKERMANN_PID_GAIN MOTOR_PID_GAIN;
typedef struct
{
    float motor_speed;
    float feedforward_pwm;
} MOTOR_DUTY;
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
extern MOTOR_DUTY servo_duty;
extern void motor_init(void);
extern void Servo_init(void);
extern void Servo_set_position_pid_gain(const MOTOR_PID_GAIN *gain);
extern void Servo_set_angle(float angle_deg);
extern void Servo_position_control(MOTOR_DUTY *duty);
extern void Servo_pid_reset(void);
extern float Servo_get_angle(void);
extern void Motor_get_default_pid_gains(MOTOR_PID_GAIN *steering_position_gain,
                                        MOTOR_PID_GAIN *left_rear_speed_gain,
                                        MOTOR_PID_GAIN *right_rear_speed_gain);
extern void Motor_set_rear_speed_pid_gains(const MOTOR_PID_GAIN *left_gain,
                                           const MOTOR_PID_GAIN *right_gain);
extern void Motor_get_rear_speed_pid_gains(MOTOR_PID_GAIN *left_gain,
                                           MOTOR_PID_GAIN *right_gain);
extern void Motor_rear_speed_pid_reset(void);
extern void Motor_rear_speed_control(const MOTOR_REAR_SPEED_INPUT *left,
                                     const MOTOR_REAR_SPEED_INPUT *right,
                                     uint8_t allow_active_braking);
extern void Motor_get_rear_speed_status(MOTOR_REAR_SPEED_STATUS *left_status,
                                        MOTOR_REAR_SPEED_STATUS *right_status);
extern void Motor_set_pwm(float left_pwm, float right_pwm);
extern void Motor_stop_all(void);
extern void start(void);
#endif

typedef struct
{
    ACKERMANN_CONTROL_CONFIG config;
    volatile ACKERMANN_PATH_COMMAND command[2];
    volatile uint8_t active_command_index;
    volatile uint32_t command_sequence;
    uint32_t last_command_sequence;

    ACKERMANN_CONTROL_TELEMETRY telemetry;

    float command_age_s;
    float speed_reference_mps;
    float steering_reference_rad;
    float left_measured_rad_s;
    float right_measured_rad_s;
    uint8_t initialized;
    uint8_t enabled;
} ACKERMANN_CONTROL_STATE;

static ACKERMANN_CONTROL_STATE g_ackermann;

static float ack_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float ack_clampf(float value, float min_value, float max_value)
{
    if(value > max_value)
    {
        return max_value;
    }
    if(value < min_value)
    {
        return min_value;
    }
    return value;
}

static void ack_copy_motor_pid_gain(ACKERMANN_PID_GAIN *destination,
                                    const MOTOR_PID_GAIN *source)
{
    destination->kp = source->kp;
    destination->ki = source->ki;
    destination->kd = source->kd;
    destination->integral_limit = source->integral_limit;
    destination->output_limit = source->output_limit;
    destination->deadband = source->deadband;
}

static void ack_copy_to_motor_pid_gain(MOTOR_PID_GAIN *destination,
                                       const ACKERMANN_PID_GAIN *source)
{
    destination->kp = source->kp;
    destination->ki = source->ki;
    destination->kd = source->kd;
    destination->integral_limit = source->integral_limit;
    destination->output_limit = source->output_limit;
    destination->deadband = source->deadband;
}

static float ack_sign(float value)
{
    return (value < 0.0f) ? -1.0f : 1.0f;
}

static float ack_wrap_pi(float angle_rad)
{
    while(angle_rad > ACKERMANN_PI)
    {
        angle_rad -= 2.0f * ACKERMANN_PI;
    }
    while(angle_rad < -ACKERMANN_PI)
    {
        angle_rad += 2.0f * ACKERMANN_PI;
    }
    return angle_rad;
}

static float ack_approach(float current, float target, float max_delta)
{
    float delta = target - current;

    if(delta > max_delta)
    {
        return current + max_delta;
    }
    if(delta < -max_delta)
    {
        return current - max_delta;
    }
    return target;
}

static uint8_t ack_config_is_valid(const ACKERMANN_CONTROL_CONFIG *config)
{
    if(config == 0)
    {
        return ZF_FALSE;
    }

    if((config->wheelbase_m <= 0.0f)
    || (config->track_width_m <= 0.0f)
    || (config->wheel_radius_m <= 0.0f)
    || (config->control_period_s <= 0.0f)
    || (config->stanley_softening_speed_mps <= 0.0f)
    || (config->max_steering_angle_rad <= 0.0f)
    || (config->max_steering_rate_rad_s <= 0.0f)
    || (config->max_vehicle_speed_mps <= 0.0f)
    || (config->max_acceleration_mps2 <= 0.0f)
    || (config->max_wheel_speed_rad_s <= 0.0f)
    || (config->steering_encoder_deg_per_road_deg == 0.0f))
    {
        return ZF_FALSE;
    }
    return ZF_TRUE;
}

static void ack_read_command(ACKERMANN_PATH_COMMAND *command, uint32_t *sequence)
{
    uint8_t index;

    if((command == 0) || (sequence == 0))
    {
        return;
    }

    /* The writer fills the inactive slot and publishes its index last. The
     * ISR never spins waiting for a writer that it may have interrupted. */
    index = g_ackermann.active_command_index;
    command->cross_track_error_m = g_ackermann.command[index].cross_track_error_m;
    command->heading_error_rad = g_ackermann.command[index].heading_error_rad;
    command->target_speed_mps = g_ackermann.command[index].target_speed_mps;
    command->valid = g_ackermann.command[index].valid;
    *sequence = g_ackermann.command_sequence;
}

static void ack_clear_runtime(void)
{
    g_ackermann.speed_reference_mps = 0.0f;
    g_ackermann.steering_reference_rad = 0.0f;
    g_ackermann.left_measured_rad_s = 0.0f;
    g_ackermann.right_measured_rad_s = 0.0f;
    Motor_rear_speed_pid_reset();

    g_ackermann.telemetry.ramped_speed_mps = 0.0f;
    g_ackermann.telemetry.left_target_rad_s = 0.0f;
    g_ackermann.telemetry.right_target_rad_s = 0.0f;
    g_ackermann.telemetry.left_pwm = 0.0f;
    g_ackermann.telemetry.right_pwm = 0.0f;
}

void Ackermann_get_default_config(ACKERMANN_CONTROL_CONFIG *config)
{
    MOTOR_PID_GAIN steering_position_gain;
    MOTOR_PID_GAIN left_rear_speed_gain;
    MOTOR_PID_GAIN right_rear_speed_gain;

    if(config == 0)
    {
        return;
    }

    config->wheelbase_m = ACKERMANN_WHEELBASE_M;
    config->track_width_m = ACKERMANN_TRACK_WIDTH_M;
    config->wheel_radius_m = ACKERMANN_WHEEL_RADIUS_M;
    config->control_period_s = ACKERMANN_CONTROL_PERIOD_S;

    config->left_encoder_sign = 1.0f;
    config->right_encoder_sign = 1.0f;
    config->left_motor_sign = 1.0f;
    config->right_motor_sign = 1.0f;

    config->stanley_gain = 1.5f;
    config->stanley_softening_speed_mps = 0.5f;
    config->max_cross_track_error_m = 3.0f;
    config->max_steering_angle_rad = 30.0f * ACKERMANN_DEG_TO_RAD;
    config->max_steering_rate_rad_s = 120.0f * ACKERMANN_DEG_TO_RAD;

    config->max_vehicle_speed_mps = 3.0f;
    config->max_acceleration_mps2 = 1.5f;
    config->max_wheel_speed_rad_s = 25.0f;
    config->stop_speed_threshold_mps = 0.02f;
    config->command_timeout_s = 0.2f;
    /* Reverse torque can create a large current spike without current sensing. */
    config->allow_active_braking = ZF_FALSE;

    config->steering_center_encoder_deg = -1.0f;
    config->steering_encoder_deg_per_road_deg = 1.0f;
    config->steering_sign = 1.0f;

    /* 三路电机 PID 的唯一默认参数源位于 motor.c。 */
    Motor_get_default_pid_gains(&steering_position_gain,
                                &left_rear_speed_gain,
                                &right_rear_speed_gain);
    ack_copy_motor_pid_gain(&config->steering_position_pid,
                            &steering_position_gain);
    ack_copy_motor_pid_gain(&config->left_speed_pid,
                            &left_rear_speed_gain);
    ack_copy_motor_pid_gain(&config->right_speed_pid,
                            &right_rear_speed_gain);
    config->left_feedforward_pwm_per_rad_s = 0.0f;
    config->right_feedforward_pwm_per_rad_s = 0.0f;
}

uint8_t Ackermann_control_init(const ACKERMANN_CONTROL_CONFIG *config)
{
    MOTOR_PID_GAIN steering_gain;
    MOTOR_PID_GAIN left_speed_gain;
    MOTOR_PID_GAIN right_speed_gain;

    if(ack_config_is_valid(config) == ZF_FALSE)
    {
        return 1U;
    }

    g_ackermann.config = *config;
    g_ackermann.config.left_encoder_sign = ack_sign(config->left_encoder_sign);
    g_ackermann.config.right_encoder_sign = ack_sign(config->right_encoder_sign);
    g_ackermann.config.left_motor_sign = ack_sign(config->left_motor_sign);
    g_ackermann.config.right_motor_sign = ack_sign(config->right_motor_sign);
    g_ackermann.config.steering_sign = ack_sign(config->steering_sign);

    g_ackermann.command_sequence = 0U;
    g_ackermann.last_command_sequence = 0U;
    g_ackermann.active_command_index = 0U;
    g_ackermann.command[0].cross_track_error_m = 0.0f;
    g_ackermann.command[0].heading_error_rad = 0.0f;
    g_ackermann.command[0].target_speed_mps = 0.0f;
    g_ackermann.command[0].valid = ZF_FALSE;
    g_ackermann.command[1] = g_ackermann.command[0];
    g_ackermann.command_age_s = 0.0f;
    g_ackermann.enabled = ZF_FALSE;

    motor_init();
    Servo_init();

    /* Ackermann 只保存参数配置；实际后轮 PID 参数和状态均由 motor.c 持有。 */
    ack_copy_to_motor_pid_gain(&left_speed_gain, &config->left_speed_pid);
    ack_copy_to_motor_pid_gain(&right_speed_gain, &config->right_speed_pid);
    Motor_set_rear_speed_pid_gains(&left_speed_gain, &right_speed_gain);
    if(g_ackermann.config.steering_center_encoder_deg < 0.0f)
    {
        g_ackermann.config.steering_center_encoder_deg = Servo_get_angle();
    }
    steering_gain.kp = g_ackermann.config.steering_position_pid.kp;
    steering_gain.ki = g_ackermann.config.steering_position_pid.ki;
    steering_gain.kd = g_ackermann.config.steering_position_pid.kd;
    steering_gain.integral_limit = g_ackermann.config.steering_position_pid.integral_limit;
    steering_gain.output_limit = g_ackermann.config.steering_position_pid.output_limit;
    steering_gain.deadband = g_ackermann.config.steering_position_pid.deadband;
    Servo_set_position_pid_gain(&steering_gain);
    Servo_set_angle(g_ackermann.config.steering_center_encoder_deg);

    ack_clear_runtime();
    g_ackermann.telemetry.commanded_speed_mps = 0.0f;
    g_ackermann.telemetry.measured_vehicle_speed_mps = 0.0f;
    g_ackermann.telemetry.steering_target_rad = 0.0f;
    g_ackermann.telemetry.steering_encoder_target_deg =
        g_ackermann.config.steering_center_encoder_deg;
    g_ackermann.telemetry.steering_encoder_measured_deg = Servo_get_angle();
    g_ackermann.telemetry.enabled = ZF_FALSE;
    g_ackermann.telemetry.path_valid = ZF_FALSE;
    g_ackermann.telemetry.encoder_valid = ZF_FALSE;
    g_ackermann.telemetry.command_timed_out = ZF_FALSE;

    g_ackermann.initialized = ZF_TRUE;
    Motor_stop_all();
    return 0U;
}

void Ackermann_control_enable(uint8_t enable)
{
    if(g_ackermann.initialized == ZF_FALSE)
    {
        return;
    }

    if(enable != ZF_FALSE)
    {
        if(g_ackermann.enabled == ZF_FALSE)
        {
            ack_clear_runtime();
            Servo_pid_reset();
            start();
            g_ackermann.enabled = ZF_TRUE;
        }
    }
    else
    {
        if(g_ackermann.enabled != ZF_FALSE)
        {
            g_ackermann.enabled = ZF_FALSE;
            ack_clear_runtime();
            Servo_pid_reset();
            Motor_stop_all();
        }
    }
    g_ackermann.telemetry.enabled = g_ackermann.enabled;
}

void Ackermann_control_reset(void)
{
    if(g_ackermann.initialized == ZF_FALSE)
    {
        return;
    }

    ack_clear_runtime();
    Servo_pid_reset();
    Motor_set_pwm(0.0f, 0.0f);
}

void Ackermann_set_path_command(const ACKERMANN_PATH_COMMAND *command)
{
    uint8_t write_index;

    if(command == 0)
    {
        return;
    }

    write_index = (uint8_t)(g_ackermann.active_command_index ^ 1U);
    g_ackermann.command[write_index].cross_track_error_m = command->cross_track_error_m;
    g_ackermann.command[write_index].heading_error_rad = command->heading_error_rad;
    g_ackermann.command[write_index].target_speed_mps = command->target_speed_mps;
    g_ackermann.command[write_index].valid = command->valid;
    ++g_ackermann.command_sequence;
    g_ackermann.active_command_index = write_index;
}

void Ackermann_set_path_error(float cross_track_error_m,
                              float heading_error_rad,
                              float target_speed_mps)
{
    ACKERMANN_PATH_COMMAND command;

    command.cross_track_error_m = cross_track_error_m;
    command.heading_error_rad = heading_error_rad;
    command.target_speed_mps = target_speed_mps;
    command.valid = ZF_TRUE;
    Ackermann_set_path_command(&command);
}

void Ackermann_invalidate_path(void)
{
    ACKERMANN_PATH_COMMAND command;

    command.cross_track_error_m = 0.0f;
    command.heading_error_rad = 0.0f;
    command.target_speed_mps = 0.0f;
    command.valid = ZF_FALSE;
    Ackermann_set_path_command(&command);
}

float Ackermann_stanley_steering(float cross_track_error_m,
                                 float heading_error_rad,
                                 float vehicle_speed_mps,
                                 const ACKERMANN_CONTROL_CONFIG *config)
{
    float limited_error;
    float denominator;
    float steering;

    if(config == 0)
    {
        return 0.0f;
    }

    limited_error = ack_clampf(cross_track_error_m,
                               -ack_absf(config->max_cross_track_error_m),
                               ack_absf(config->max_cross_track_error_m));
    denominator = ack_absf(vehicle_speed_mps) + config->stanley_softening_speed_mps;
    steering = ack_wrap_pi(heading_error_rad)
             + atan2f(config->stanley_gain * limited_error, denominator);
    return ack_clampf(steering,
                      -config->max_steering_angle_rad,
                      config->max_steering_angle_rad);
}

void Ackermann_electronic_differential(float center_speed_mps,
                                       float steering_angle_rad,
                                       const ACKERMANN_CONTROL_CONFIG *config,
                                       float *left_speed_mps,
                                       float *right_speed_mps)
{
    float yaw_rate_rad_s;

    if((config == 0) || (left_speed_mps == 0) || (right_speed_mps == 0)
    || (config->wheelbase_m <= 0.0f))
    {
        return;
    }

    yaw_rate_rad_s = center_speed_mps * tanf(steering_angle_rad) / config->wheelbase_m;
    *left_speed_mps = center_speed_mps
                    - yaw_rate_rad_s * config->track_width_m * 0.5f;
    *right_speed_mps = center_speed_mps
                     + yaw_rate_rad_s * config->track_width_m * 0.5f;
}

void Ackermann_control_step(float left_measured_rad_s,
                            float right_measured_rad_s,
                            uint8_t encoder_feedback_valid)
{
    ACKERMANN_PATH_COMMAND command;
    MOTOR_REAR_SPEED_INPUT left_motor_input;
    MOTOR_REAR_SPEED_INPUT right_motor_input;
    MOTOR_REAR_SPEED_STATUS left_motor_status;
    MOTOR_REAR_SPEED_STATUS right_motor_status;
    uint32_t command_sequence;
    float dt;
    float target_speed_mps;
    float desired_steering_rad;
    float max_steering_step;
    float left_target_mps;
    float right_target_mps;
    float left_target_rad_s;
    float right_target_rad_s;
    float largest_wheel_target;
    float wheel_scale;
    float steering_encoder_target_deg;
    uint8_t command_timed_out;
    uint8_t command_is_valid;

    if(g_ackermann.initialized == ZF_FALSE)
    {
        return;
    }

    dt = g_ackermann.config.control_period_s;
    ack_read_command(&command, &command_sequence);
    if(command_sequence != g_ackermann.last_command_sequence)
    {
        g_ackermann.command_age_s = 0.0f;
        g_ackermann.last_command_sequence = command_sequence;
    }
    else
    {
        g_ackermann.command_age_s += dt;
    }

    command_timed_out = (g_ackermann.config.command_timeout_s > 0.0f)
                     && (g_ackermann.command_age_s > g_ackermann.config.command_timeout_s);
    command_is_valid = (command.valid != ZF_FALSE) && (command_timed_out == ZF_FALSE);

    g_ackermann.telemetry.cross_track_error_m = command.cross_track_error_m;
    g_ackermann.telemetry.heading_error_rad = command.heading_error_rad;
    g_ackermann.telemetry.command_age_s = g_ackermann.command_age_s;
    g_ackermann.telemetry.enabled = g_ackermann.enabled;
    g_ackermann.telemetry.path_valid = command_is_valid;
    g_ackermann.telemetry.encoder_valid = encoder_feedback_valid;
    g_ackermann.telemetry.command_timed_out = command_timed_out;

    if(encoder_feedback_valid == ZF_FALSE)
    {
        if(g_ackermann.enabled != ZF_FALSE)
        {
            ack_clear_runtime();
            Motor_set_pwm(0.0f, 0.0f);
            Servo_set_angle(g_ackermann.config.steering_center_encoder_deg);
            Servo_position_control(&servo_duty);
        }
        return;
    }

    /* 速度换算和低通滤波都由 encoder.c 完成，PID 直接使用其输出。 */
    g_ackermann.left_measured_rad_s =
        left_measured_rad_s * g_ackermann.config.left_encoder_sign;
    g_ackermann.right_measured_rad_s =
        right_measured_rad_s * g_ackermann.config.right_encoder_sign;

    g_ackermann.telemetry.left_measured_rad_s = g_ackermann.left_measured_rad_s;
    g_ackermann.telemetry.right_measured_rad_s = g_ackermann.right_measured_rad_s;
    g_ackermann.telemetry.measured_vehicle_speed_mps =
        0.5f * g_ackermann.config.wheel_radius_m
        * (g_ackermann.left_measured_rad_s + g_ackermann.right_measured_rad_s);
    g_ackermann.telemetry.steering_encoder_measured_deg = Servo_get_angle();

    /* Keep encoder telemetry live while path control is disabled so the
     * time-limited motor test page can display measured speed and angle. */
    if(g_ackermann.enabled == ZF_FALSE)
    {
        return;
    }

    if(command_is_valid == ZF_FALSE)
    {
        ack_clear_runtime();
        Motor_set_pwm(0.0f, 0.0f);
        Servo_set_angle(g_ackermann.config.steering_center_encoder_deg);
        Servo_position_control(&servo_duty);
        return;
    }

    /* This Stanley implementation intentionally accepts forward commands only. */
    target_speed_mps = ack_clampf(command.target_speed_mps,
                                  0.0f,
                                  g_ackermann.config.max_vehicle_speed_mps);
    g_ackermann.speed_reference_mps = ack_approach(
        g_ackermann.speed_reference_mps,
        target_speed_mps,
        g_ackermann.config.max_acceleration_mps2 * dt);

    desired_steering_rad = Ackermann_stanley_steering(
        command.cross_track_error_m,
        command.heading_error_rad,
        g_ackermann.telemetry.measured_vehicle_speed_mps,
        &g_ackermann.config);
    max_steering_step = g_ackermann.config.max_steering_rate_rad_s * dt;
    g_ackermann.steering_reference_rad = ack_approach(
        g_ackermann.steering_reference_rad,
        desired_steering_rad,
        max_steering_step);

    Ackermann_electronic_differential(g_ackermann.speed_reference_mps,
                                      g_ackermann.steering_reference_rad,
                                      &g_ackermann.config,
                                      &left_target_mps,
                                      &right_target_mps);
    left_target_rad_s = left_target_mps / g_ackermann.config.wheel_radius_m;
    right_target_rad_s = right_target_mps / g_ackermann.config.wheel_radius_m;

    largest_wheel_target = ack_absf(left_target_rad_s);
    if(ack_absf(right_target_rad_s) > largest_wheel_target)
    {
        largest_wheel_target = ack_absf(right_target_rad_s);
    }
    if(largest_wheel_target > g_ackermann.config.max_wheel_speed_rad_s)
    {
        wheel_scale = g_ackermann.config.max_wheel_speed_rad_s / largest_wheel_target;
        left_target_rad_s *= wheel_scale;
        right_target_rad_s *= wheel_scale;
    }

    if((target_speed_mps <= g_ackermann.config.stop_speed_threshold_mps)
    && (g_ackermann.speed_reference_mps <= g_ackermann.config.stop_speed_threshold_mps))
    {
        left_target_rad_s = 0.0f;
        right_target_rad_s = 0.0f;
        Motor_rear_speed_pid_reset();
        Motor_set_pwm(0.0f, 0.0f);
    }
    else
    {
        /* Ackermann 到这里已经完成 Stanley、电子差速和目标限速。
         * 后轮 PID、抗积分饱和和 PWM 输出从此处统一交给 motor.c。 */
        left_motor_input.target_rad_s = left_target_rad_s;
        left_motor_input.measured_rad_s = g_ackermann.left_measured_rad_s;
        left_motor_input.feedforward_pwm =
            g_ackermann.config.left_feedforward_pwm_per_rad_s * left_target_rad_s;
        left_motor_input.output_sign = g_ackermann.config.left_motor_sign;
        left_motor_input.pwm_limit = 0.0f;
        left_motor_input.enabled = ZF_TRUE;

        right_motor_input.target_rad_s = right_target_rad_s;
        right_motor_input.measured_rad_s = g_ackermann.right_measured_rad_s;
        right_motor_input.feedforward_pwm =
            g_ackermann.config.right_feedforward_pwm_per_rad_s * right_target_rad_s;
        right_motor_input.output_sign = g_ackermann.config.right_motor_sign;
        right_motor_input.pwm_limit = 0.0f;
        right_motor_input.enabled = ZF_TRUE;

        Motor_rear_speed_control(&left_motor_input,
                                 &right_motor_input,
                                 g_ackermann.config.allow_active_braking);
    }
    Motor_get_rear_speed_status(&left_motor_status, &right_motor_status);

    steering_encoder_target_deg = g_ackermann.config.steering_center_encoder_deg
        + g_ackermann.config.steering_sign
        * g_ackermann.steering_reference_rad * ACKERMANN_RAD_TO_DEG
        * g_ackermann.config.steering_encoder_deg_per_road_deg;
    Servo_set_angle(steering_encoder_target_deg);
    Servo_position_control(&servo_duty);

    g_ackermann.telemetry.commanded_speed_mps = target_speed_mps;
    g_ackermann.telemetry.ramped_speed_mps = g_ackermann.speed_reference_mps;
    g_ackermann.telemetry.steering_target_rad = g_ackermann.steering_reference_rad;
    g_ackermann.telemetry.steering_encoder_target_deg = steering_encoder_target_deg;
    g_ackermann.telemetry.steering_encoder_measured_deg = Servo_get_angle();
    g_ackermann.telemetry.left_target_rad_s = left_target_rad_s;
    g_ackermann.telemetry.right_target_rad_s = right_target_rad_s;
    g_ackermann.telemetry.left_pwm = left_motor_status.pwm;
    g_ackermann.telemetry.right_pwm = right_motor_status.pwm;
}

void Ackermann_set_steering_center(float encoder_angle_deg)
{
    if(g_ackermann.initialized == ZF_FALSE)
    {
        return;
    }

    g_ackermann.config.steering_center_encoder_deg = encoder_angle_deg;
    Servo_set_angle(encoder_angle_deg);
}

float Ackermann_get_steering_center(void)
{
    if(g_ackermann.initialized == ZF_FALSE)
    {
        return 0.0f;
    }
    return g_ackermann.config.steering_center_encoder_deg;
}

void Ackermann_get_telemetry(ACKERMANN_CONTROL_TELEMETRY *telemetry)
{
    if(telemetry != 0)
    {
        *telemetry = g_ackermann.telemetry;
    }
}

void Ackermann_set_rear_speed_pid_gains(const ACKERMANN_PID_GAIN *left_gain,
                                        const ACKERMANN_PID_GAIN *right_gain)
{
    MOTOR_PID_GAIN motor_left_gain;
    MOTOR_PID_GAIN motor_right_gain;
    const MOTOR_PID_GAIN *motor_left_gain_ptr = 0;
    const MOTOR_PID_GAIN *motor_right_gain_ptr = 0;

    if(g_ackermann.initialized == ZF_FALSE)
    {
        return;
    }

    if(left_gain != 0)
    {
        g_ackermann.config.left_speed_pid = *left_gain;
        ack_copy_to_motor_pid_gain(&motor_left_gain, left_gain);
        motor_left_gain_ptr = &motor_left_gain;
    }
    if(right_gain != 0)
    {
        g_ackermann.config.right_speed_pid = *right_gain;
        ack_copy_to_motor_pid_gain(&motor_right_gain, right_gain);
        motor_right_gain_ptr = &motor_right_gain;
    }
    Motor_set_rear_speed_pid_gains(motor_left_gain_ptr, motor_right_gain_ptr);
}

void Ackermann_get_rear_speed_pid_gains(ACKERMANN_PID_GAIN *left_gain,
                                        ACKERMANN_PID_GAIN *right_gain)
{
    MOTOR_PID_GAIN motor_left_gain;
    MOTOR_PID_GAIN motor_right_gain;

    if(g_ackermann.initialized == ZF_FALSE)
    {
        return;
    }

    Motor_get_rear_speed_pid_gains((left_gain != 0) ? &motor_left_gain : 0,
                                   (right_gain != 0) ? &motor_right_gain : 0);
    if(left_gain != 0) { ack_copy_motor_pid_gain(left_gain, &motor_left_gain); }
    if(right_gain != 0) { ack_copy_motor_pid_gain(right_gain, &motor_right_gain); }
}
