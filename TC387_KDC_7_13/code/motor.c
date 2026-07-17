#include "zf_common_headfile.h"

#define SERVO_POS_DEADBAND_DEG      (0.4f)
#define SERVO_SPD_DEADBAND_DPS      (2.0f)
#define SERVO_MAX_SPEED_DPS         (720.0f)
#define SERVO_PWM_LIMIT             (2000.0f)
#define SERVO_PWM_INTEGRAL_LIMIT    (1200.0f)

/*
 * =========================== PID 参数统一修改区 ===========================
 * 后续调车时只修改本区域。字段顺序依次为：
 * Kp、Ki、Kd、积分输出限幅、PID 输出限幅、误差死区。
 */
static const MOTOR_PID_GAIN motor_default_steering_position_pid =
{
    60.0f,      /* Kp */
    1.0f,       /* Ki */
    0.5f,       /* Kd */
    400.0f,     /* 积分输出限幅 */
    2000.0f,    /* PWM 输出限幅 */
    0.3f        /* 角度死区，单位 deg */
};

static const MOTOR_PID_GAIN motor_default_left_rear_speed_pid =
{
    550.0f,     /* Kp */
    120.0f,     /* Ki */
    0.0f,       /* Kd */
    3000.0f,    /* 积分输出限幅 */
    8000.0f,    /* PWM 输出限幅 */
    0.05f       /* 速度死区，单位 rad/s */
};

static const MOTOR_PID_GAIN motor_default_right_rear_speed_pid =
{
    550.0f,     /* Kp */
    120.0f,     /* Ki */
    0.0f,       /* Kd */
    3000.0f,    /* 积分输出限幅 */
    8000.0f,    /* PWM 输出限幅 */
    0.05f       /* 速度死区，单位 rad/s */
};

/* 串级转向控制的外环和内环参数；当前阿克曼控制使用上面的直接位置环。 */
static const MOTOR_PID_GAIN motor_default_steering_cascade_position_pid =
{
    10.0f, 0.0f, 0.0f, 0.0f, 360.0f, SERVO_POS_DEADBAND_DEG
};

static const MOTOR_PID_GAIN motor_default_steering_cascade_speed_pid =
{
    2.0f, 0.04f, 0.0f,
    SERVO_PWM_INTEGRAL_LIMIT, 1200.0f, SERVO_SPD_DEADBAND_DPS
};
/* ========================= PID 参数统一修改区结束 ========================= */

MOTOR_DUTY motor_L_duty;
MOTOR_DUTY motor_R_duty;
MOTOR_DUTY servo_duty;
MOTOR_CASCADE_PID servo_pid;
MOTOR_PID servo_position_pid;
/* 左右后轮速度 PID 的参数和运行状态只保存在 motor.c 中。 */
static MOTOR_PID motor_left_rear_speed_pid;
static MOTOR_PID motor_right_rear_speed_pid;
static MOTOR_REAR_SPEED_STATUS motor_left_rear_speed_status;
static MOTOR_REAR_SPEED_STATUS motor_right_rear_speed_status;

static const MOTOR_DRIVER motor_l_driver =
{
    motor_L_DIS,
    { motor_L_PWM1, motor_L_PWM1N }, /* PWM1: upper MOSFET, PWM1N: lower MOSFET */
    { motor_L_PWM2, motor_L_PWM2N }, /* PWM2: upper MOSFET, PWM2N: lower MOSFET */
    MOTOR_PWM_FREQ_HZ,
    (float)PWM_DUTY_MAX
};

static const MOTOR_DRIVER motor_r_driver =
{
    motor_R_DIS,
    { motor_R_PWM1, motor_R_PWM1N }, /* PWM1: upper MOSFET, PWM1N: lower MOSFET */
    { motor_R_PWM2, motor_R_PWM2N }, /* PWM2: upper MOSFET, PWM2N: lower MOSFET */
    MOTOR_PWM_FREQ_HZ,
    (float)PWM_DUTY_MAX
};

static const MOTOR_DRIVER servo_driver =
{
    motor_T_DIS,
    { motor_T_PWM1, motor_T_PWM1N }, /* PWM1: upper MOSFET, PWM1N: lower MOSFET */
    { motor_T_PWM2, motor_T_PWM2N }, /* PWM2: upper MOSFET, PWM2N: lower MOSFET */
    MOTOR_PWM_FREQ_HZ,
    SERVO_PWM_LIMIT
};

/**
 * @brief  计算浮点数绝对值。
 * @param  value 输入值。
 * @return value 的绝对值。
 */
static float motor_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief  将浮点数限制在指定范围内。
 * @param  value     待限幅值。
 * @param  min_value 最小允许值。
 * @param  max_value 最大允许值。
 * @return 限幅后的值。
 */
static float motor_clampf(float value, float min_value, float max_value)
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

/**
 * @brief  将角度归一化到 0~360 度。
 * @param  angle_deg 原始角度，单位 deg。
 * @return 归一化后的角度，单位 deg。
 */
static float motor_normalize_angle(float angle_deg)
{
    while(angle_deg >= MOTOR_FULL_ANGLE_DEG)
    {
        angle_deg -= MOTOR_FULL_ANGLE_DEG;
    }
    while(angle_deg < 0.0f)
    {
        angle_deg += MOTOR_FULL_ANGLE_DEG;
    }
    return angle_deg;
}

/**
 * @brief  计算带 0/360 度过零处理的最短角度误差。
 * @param  target_deg  目标角度，单位 deg。
 * @param  current_deg 当前角度，单位 deg。
 * @return 目标角度到当前角度的最短误差，范围约为 -180~180 deg。
 */
static float motor_angle_error(float target_deg, float current_deg)
{
    float error = motor_normalize_angle(target_deg) - motor_normalize_angle(current_deg);

    if(error > (MOTOR_FULL_ANGLE_DEG * 0.5f))
    {
        error -= MOTOR_FULL_ANGLE_DEG;
    }
    else if(error < -(MOTOR_FULL_ANGLE_DEG * 0.5f))
    {
        error += MOTOR_FULL_ANGLE_DEG;
    }

    return error;
}

/**
 * @brief  清空单个 PID 控制器的运行状态。
 * @param  pid 需要复位的 PID 控制器。
 * @note   只清运行状态，不改变 PID 参数。
 */
static void motor_pid_reset(MOTOR_PID *pid)
{
    if(pid == 0)
    {
        return;
    }

    pid->state.error = 0.0f;
    pid->state.last_error = 0.0f;
    pid->state.last_measurement = 0.0f;
    pid->state.integral = 0.0f;
    pid->state.derivative = 0.0f;
    pid->state.output = 0.0f;
    pid->state.initialized = ZF_FALSE;
}

/**
 * @brief  设置 PID 参数并清空该 PID 的运行状态。
 * @param  pid            目标 PID 控制器。
 * @param  kp             比例系数。
 * @param  ki             积分系数，内部按 ki * error * dt 积分。
 * @param  kd             微分系数，内部按误差变化率或测量值变化率计算。
 * @param  integral_limit 积分项限幅，传 0 表示不单独限幅。
 * @param  output_limit   输出限幅，传 0 表示 PID 内部不限制输出。
 * @param  deadband       误差死区，绝对误差小于该值时按 0 处理。
 */
static void motor_pid_set_gain(MOTOR_PID *pid,
                               float kp,
                               float ki,
                               float kd,
                               float integral_limit,
                               float output_limit,
                               float deadband)
{
    if(pid == 0)
    {
        return;
    }

    pid->gain.kp = kp;
    pid->gain.ki = ki;
    pid->gain.kd = kd;
    pid->gain.integral_limit = motor_absf(integral_limit);
    pid->gain.output_limit = motor_absf(output_limit);
    pid->gain.deadband = motor_absf(deadband);
    motor_pid_reset(pid);
}

/**
 * @brief  拷贝 PID 参数，并统一整理限幅和死区为非负值。
 * @param  pid  目标 PID 控制器。
 * @param  gain 外部传入的 PID 参数。
 * @note   该函数不清空 PID 状态，调用方可按需要再 reset。
 */
static void motor_pid_copy_gain(MOTOR_PID *pid, const MOTOR_PID_GAIN *gain)
{
    if((pid == 0) || (gain == 0))
    {
        return;
    }

    pid->gain.kp = gain->kp;
    pid->gain.ki = gain->ki;
    pid->gain.kd = gain->kd;
    pid->gain.integral_limit = motor_absf(gain->integral_limit);
    pid->gain.output_limit = motor_absf(gain->output_limit);
    pid->gain.deadband = motor_absf(gain->deadband);
}

/**
 * @brief  使用已计算好的误差更新 PID。
 * @param  pid                       目标 PID 控制器。
 * @param  error                     当前误差。
 * @param  measurement               当前测量值，用于微分对测量值模式。
 * @param  dt                        控制周期，单位 s。
 * @param  derivative_on_measurement 非 0 时微分项对测量值求导，适合速度环抑制目标阶跃冲击。
 * @return PID 输出值。
 * @note   内置死区、积分限幅、输出限幅和输出饱和时的抗积分继续累积。
 */
static float motor_pid_update_by_error(MOTOR_PID *pid,
                                       float error,
                                       float measurement,
                                       float dt,
                                       uint8 derivative_on_measurement)
{
    float p_out;
    float d_out;
    float integral_candidate;
    float output_candidate;
    uint8 saturating_high;
    uint8 saturating_low;

    if(pid == 0)
    {
        return 0.0f;
    }

    if(motor_absf(error) <= pid->gain.deadband)
    {
        error = 0.0f;
    }

    p_out = pid->gain.kp * error;

    if((pid->state.initialized != ZF_FALSE) && (dt > 0.0f))
    {
        if(derivative_on_measurement != ZF_FALSE)
        {
            pid->state.derivative = -(measurement - pid->state.last_measurement) / dt;
        }
        else
        {
            pid->state.derivative = (error - pid->state.last_error) / dt;
        }
    }
    else
    {
        pid->state.derivative = 0.0f;
        pid->state.initialized = ZF_TRUE;
    }
    d_out = pid->gain.kd * pid->state.derivative;

    integral_candidate = pid->state.integral;
    if(pid->gain.ki != 0.0f)
    {
        integral_candidate += pid->gain.ki * error * dt;
        if(pid->gain.integral_limit > 0.0f)
        {
            integral_candidate = motor_clampf(integral_candidate,
                                             -pid->gain.integral_limit,
                                             pid->gain.integral_limit);
        }
    }
    else
    {
        integral_candidate = 0.0f;
    }

    output_candidate = p_out + integral_candidate + d_out;
    saturating_high = (pid->gain.output_limit > 0.0f)
                    && (output_candidate > pid->gain.output_limit)
                    && (error > 0.0f);
    saturating_low = (pid->gain.output_limit > 0.0f)
                   && (output_candidate < -pid->gain.output_limit)
                   && (error < 0.0f);

    if((saturating_high == ZF_FALSE) && (saturating_low == ZF_FALSE))
    {
        pid->state.integral = integral_candidate;
    }
    else
    {
        output_candidate = p_out + pid->state.integral + d_out;
    }

    if(pid->gain.output_limit > 0.0f)
    {
        output_candidate = motor_clampf(output_candidate,
                                       -pid->gain.output_limit,
                                       pid->gain.output_limit);
    }

    pid->state.error = error;
    pid->state.last_error = error;
    pid->state.last_measurement = measurement;
    pid->state.output = output_candidate;

    return output_candidate;
}

/**
 * @brief  使用目标值和测量值更新 PID。
 * @param  pid                       目标 PID 控制器。
 * @param  target                    目标值。
 * @param  measurement               当前测量值。
 * @param  dt                        控制周期，单位 s。
 * @param  derivative_on_measurement 非 0 时微分项对测量值求导。
 * @return PID 输出值。
 */
static float motor_pid_update(MOTOR_PID *pid,
                              float target,
                              float measurement,
                              float dt,
                              uint8 derivative_on_measurement)
{
    return motor_pid_update_by_error(pid,
                                     target - measurement,
                                     measurement,
                                     dt,
                                     derivative_on_measurement);
}

/**
 * @brief 计算一个后轮速度环，并把 PID、前馈、运行限幅和防反向制动统一处理。
 * @note  measurement 必须是 encoder.c 已经换算好的 rad/s，不在本函数读取硬件计数器。
 */
static float motor_rear_speed_pid_update(MOTOR_PID *pid,
                                         const MOTOR_REAR_SPEED_INPUT *input,
                                         MOTOR_REAR_SPEED_STATUS *status,
                                         uint8 allow_active_braking)
{
    float error;
    float derivative;
    float integral_candidate;
    float output_candidate;
    float output_limit;
    float output_sign;
    uint8 pushes_high;
    uint8 pushes_low;

    if((pid == 0) || (status == 0))
    {
        return 0.0f;
    }

    if(input == 0)
    {
        motor_pid_reset(pid);
        status->target_rad_s = 0.0f;
        status->measured_rad_s = 0.0f;
        status->error_rad_s = 0.0f;
        status->pwm = 0.0f;
        return 0.0f;
    }

    status->target_rad_s = input->target_rad_s;
    status->measured_rad_s = input->measured_rad_s;
    status->error_rad_s = input->target_rad_s - input->measured_rad_s;

    if(input->enabled == 0U)
    {
        motor_pid_reset(pid);
        status->pwm = 0.0f;
        return 0.0f;
    }

    error = status->error_rad_s;
    if(motor_absf(error) <= pid->gain.deadband)
    {
        error = 0.0f;
    }

    if(pid->state.initialized != ZF_FALSE)
    {
        /* 速度环对测量值微分，目标速度阶跃时不会产生微分冲击。 */
        derivative = -(input->measured_rad_s - pid->state.last_measurement)
                   / MOTOR_CONTROL_DT;
    }
    else
    {
        derivative = 0.0f;
        pid->state.initialized = ZF_TRUE;
    }

    output_limit = pid->gain.output_limit;
    if((input->pwm_limit > 0.0f)
    && ((output_limit <= 0.0f) || (input->pwm_limit < output_limit)))
    {
        output_limit = input->pwm_limit;
    }

    integral_candidate = pid->state.integral + pid->gain.ki * error * MOTOR_CONTROL_DT;
    if(pid->gain.integral_limit > 0.0f)
    {
        integral_candidate = motor_clampf(integral_candidate,
                                         -pid->gain.integral_limit,
                                         pid->gain.integral_limit);
    }

    output_candidate = input->feedforward_pwm
                     + pid->gain.kp * error
                     + integral_candidate
                     + pid->gain.kd * derivative;
    pushes_high = (output_limit > 0.0f)
                && (output_candidate > output_limit)
                && (error > 0.0f);
    pushes_low = (output_limit > 0.0f)
               && (output_candidate < -output_limit)
               && (error < 0.0f);

    if((pushes_high == ZF_FALSE) && (pushes_low == ZF_FALSE))
    {
        pid->state.integral = integral_candidate;
    }
    else
    {
        /* 输出继续进入饱和区时不接受本周期的新积分。 */
        output_candidate = input->feedforward_pwm
                         + pid->gain.kp * error
                         + pid->state.integral
                         + pid->gain.kd * derivative;
    }

    if(output_limit > 0.0f)
    {
        output_candidate = motor_clampf(output_candidate,
                                        -output_limit,
                                        output_limit);
    }

    if((allow_active_braking == 0U)
    && (((input->target_rad_s > 0.0f) && (output_candidate < 0.0f))
     || ((input->target_rad_s < 0.0f) && (output_candidate > 0.0f))))
    {
        output_candidate = 0.0f;
        /* 防止反向积分在禁止主动制动时长期残留。 */
        if(((input->target_rad_s > 0.0f) && (pid->state.integral < 0.0f))
        || ((input->target_rad_s < 0.0f) && (pid->state.integral > 0.0f)))
        {
            pid->state.integral = 0.0f;
        }
    }

    pid->state.error = error;
    pid->state.last_error = error;
    pid->state.last_measurement = input->measured_rad_s;
    pid->state.derivative = derivative;
    pid->state.output = output_candidate;

    output_sign = (input->output_sign < 0.0f) ? -1.0f : 1.0f;
    status->pwm = output_candidate * output_sign;
    return status->pwm;
}

/**
 * @brief  初始化一个双半桥电机驱动器。
 * @param  driver 电机驱动硬件配置。
 * @note   phase_a 和 phase_b 分别初始化为互补 PWM，enable 引脚默认拉高。
 */
static void motor_driver_init(const MOTOR_DRIVER *driver)
{
    if(driver == 0)
    {
        return;
    }

    pwm_hl_init(driver->phase_a.top_pin, driver->phase_a.bottom_pin, driver->pwm_freq_hz, 0);
    pwm_hl_init(driver->phase_b.top_pin, driver->phase_b.bottom_pin, driver->pwm_freq_hz, 0);
    /* Keep the bridge disabled until start() is called explicitly. */
    gpio_init(driver->enable_pin, GPO, DISABLE, GPO_PUSH_PULL);
}

/**
 * @brief  设置电机驱动器有符号 PWM 输出。
 * @param  driver 电机驱动硬件配置。
 * @param  pwm    有符号 PWM，占空比单位与 PWM_DUTY_MAX 一致。
 * @note   pwm > 0 时 phase_a 输出，pwm < 0 时 phase_b 输出，pwm = 0 时两相关闭。
 */
static void motor_driver_set_pwm(const MOTOR_DRIVER *driver, float pwm)
{
    float limit;
    float abs_pwm;
    uint32 duty;

    if(driver == 0)
    {
        return;
    }

    limit = driver->pwm_limit;
    if((limit <= 0.0f) || (limit > (float)PWM_DUTY_MAX))
    {
        limit = (float)PWM_DUTY_MAX;
    }

    pwm = motor_clampf(pwm, -limit, limit);
    abs_pwm = motor_absf(pwm);
    duty = (uint32)(abs_pwm + 0.5f);

    if(duty > PWM_DUTY_MAX)
    {
        duty = PWM_DUTY_MAX;
    }

    if(duty == 0u)
    {
        pwm_hl_set_duty(driver->phase_a.top_pin, 0);
        pwm_hl_set_duty(driver->phase_b.top_pin, 0);
    }
    else if(pwm > 0.0f)
    {
        /*
         * Positive direction:
         *   phase A: main/complementary PWM pair
         *   phase B: main output is constantly 1, complementary output is constantly 0
         */
        pwm_hl_set_duty(driver->phase_a.top_pin, duty);
        pwm_hl_set_duty(driver->phase_b.top_pin, PWM_DUTY_MAX);
    }
    else
    {
        /* Reverse direction uses the symmetric bridge state. */
        pwm_hl_set_duty(driver->phase_a.top_pin, PWM_DUTY_MAX);
        pwm_hl_set_duty(driver->phase_b.top_pin, duty);
    }
}

/**
 * @brief  加载转向电机串级 PID 的默认参数。
 * @note   默认 kp/ki/kd 为 0，避免未知硬件上电后直接动作；限幅和死区会先配置好。
 */
static void servo_pid_load_default(void)
{
    motor_pid_set_gain(&servo_pid.position,
        motor_default_steering_cascade_position_pid.kp,
        motor_default_steering_cascade_position_pid.ki,
        motor_default_steering_cascade_position_pid.kd,
        motor_default_steering_cascade_position_pid.integral_limit,
        motor_default_steering_cascade_position_pid.output_limit,
        motor_default_steering_cascade_position_pid.deadband);

    motor_pid_set_gain(&servo_pid.speed,
        motor_default_steering_cascade_speed_pid.kp,
        motor_default_steering_cascade_speed_pid.ki,
        motor_default_steering_cascade_speed_pid.kd,
        motor_default_steering_cascade_speed_pid.integral_limit,
        motor_default_steering_cascade_speed_pid.output_limit,
        motor_default_steering_cascade_speed_pid.deadband);

    motor_pid_set_gain(&servo_position_pid,
        motor_default_steering_position_pid.kp,
        motor_default_steering_position_pid.ki,
        motor_default_steering_position_pid.kd,
        motor_default_steering_position_pid.integral_limit,
        motor_default_steering_position_pid.output_limit,
        motor_default_steering_position_pid.deadband);
}

void Motor_get_default_pid_gains(MOTOR_PID_GAIN *steering_position_gain,
                                 MOTOR_PID_GAIN *left_rear_speed_gain,
                                 MOTOR_PID_GAIN *right_rear_speed_gain)
{
    if(steering_position_gain != 0)
    {
        *steering_position_gain = motor_default_steering_position_pid;
    }
    if(left_rear_speed_gain != 0)
    {
        *left_rear_speed_gain = motor_default_left_rear_speed_pid;
    }
    if(right_rear_speed_gain != 0)
    {
        *right_rear_speed_gain = motor_default_right_rear_speed_pid;
    }
}

void Motor_set_rear_speed_pid_gains(const MOTOR_PID_GAIN *left_gain,
                                    const MOTOR_PID_GAIN *right_gain)
{
    if(left_gain != 0)
    {
        motor_pid_copy_gain(&motor_left_rear_speed_pid, left_gain);
        motor_pid_reset(&motor_left_rear_speed_pid);
    }
    if(right_gain != 0)
    {
        motor_pid_copy_gain(&motor_right_rear_speed_pid, right_gain);
        motor_pid_reset(&motor_right_rear_speed_pid);
    }
}

void Motor_get_rear_speed_pid_gains(MOTOR_PID_GAIN *left_gain,
                                    MOTOR_PID_GAIN *right_gain)
{
    if(left_gain != 0)
    {
        *left_gain = motor_left_rear_speed_pid.gain;
    }
    if(right_gain != 0)
    {
        *right_gain = motor_right_rear_speed_pid.gain;
    }
}

void Motor_rear_speed_pid_reset(void)
{
    motor_pid_reset(&motor_left_rear_speed_pid);
    motor_pid_reset(&motor_right_rear_speed_pid);

    motor_left_rear_speed_status.target_rad_s = 0.0f;
    motor_left_rear_speed_status.measured_rad_s = 0.0f;
    motor_left_rear_speed_status.error_rad_s = 0.0f;
    motor_left_rear_speed_status.pwm = 0.0f;
    motor_right_rear_speed_status.target_rad_s = 0.0f;
    motor_right_rear_speed_status.measured_rad_s = 0.0f;
    motor_right_rear_speed_status.error_rad_s = 0.0f;
    motor_right_rear_speed_status.pwm = 0.0f;
}

/**
 * @brief  初始化左右后轮电机驱动。
 * @note   当前只完成 PWM/GPIO 初始化和目标值清零，后轮闭环控制逻辑可继续在此结构上扩展。
 */
void motor_init(void)
{
    motor_L_duty.motor_speed = 0.0f;
    motor_L_duty.feedforward_pwm = 0.0f;
    motor_R_duty.motor_speed = 0.0f;
    motor_R_duty.feedforward_pwm = 0.0f;

    motor_driver_init(&motor_l_driver);
    motor_driver_init(&motor_r_driver);
    motor_driver_set_pwm(&motor_l_driver, 0.0f);
    motor_driver_set_pwm(&motor_r_driver, 0.0f);

    /* 后轮驱动初始化时一并装载 motor.c 顶部的默认 PID。 */
    motor_pid_copy_gain(&motor_left_rear_speed_pid,
                        &motor_default_left_rear_speed_pid);
    motor_pid_copy_gain(&motor_right_rear_speed_pid,
                        &motor_default_right_rear_speed_pid);
    Motor_rear_speed_pid_reset();
}

/**
 * @brief  初始化转向电机驱动和串级 PID，并读取 encoder.c 的初始位置。
 * @note   本函数不初始化、不读取、不清零编码器。编码器必须先由
 *         QdInit() 初始化；实车启动前应先把前轮摆到直行位置。
 */
void Servo_init(void)
{
    servo_duty.motor_speed = 0.0f;
    servo_duty.feedforward_pwm = 0.0f;

    motor_driver_init(&servo_driver);
    motor_driver_set_pwm(&servo_driver, 0.0f);
    servo_pid_load_default();

    Servo_pid_reset();
    /* 编码器由 QdInit/GetSpeed 管理，motor.c 只使用计算结果。 */
    servo_pid.angle_deg = car_angle;
    servo_pid.speed_dps = car_speed[0];
    servo_duty.motor_speed = servo_pid.angle_deg;
    servo_pid.target_angle_deg = servo_pid.angle_deg;
}

/**
 * @brief  使能左右后轮电机和转向电机驱动。
 * @note   使用明确置高，不再使用 toggle，避免多次调用导致驱动反复开关。
 */
void start(void)
{
    gpio_set_level(motor_L_DIS, ENABLE);
    gpio_set_level(motor_R_DIS, ENABLE);
    gpio_set_level(motor_T_DIS, ENABLE);
}

void Motor_set_pwm(float left_pwm, float right_pwm)
{
    motor_driver_set_pwm(&motor_l_driver, left_pwm);
    motor_driver_set_pwm(&motor_r_driver, right_pwm);
}

void Motor_rear_speed_control(const MOTOR_REAR_SPEED_INPUT *left,
                              const MOTOR_REAR_SPEED_INPUT *right,
                              uint8 allow_active_braking)
{
    float left_pwm;
    float right_pwm;

    left_pwm = motor_rear_speed_pid_update(&motor_left_rear_speed_pid,
                                           left,
                                           &motor_left_rear_speed_status,
                                           allow_active_braking);
    right_pwm = motor_rear_speed_pid_update(&motor_right_rear_speed_pid,
                                            right,
                                            &motor_right_rear_speed_status,
                                            allow_active_braking);
    Motor_set_pwm(left_pwm, right_pwm);
}

void Motor_get_rear_speed_status(MOTOR_REAR_SPEED_STATUS *left_status,
                                 MOTOR_REAR_SPEED_STATUS *right_status)
{
    if(left_status != 0)
    {
        *left_status = motor_left_rear_speed_status;
    }
    if(right_status != 0)
    {
        *right_status = motor_right_rear_speed_status;
    }
}

void Motor_enable_channels(uint8 left_enable,
                           uint8 right_enable,
                           uint8 steering_enable)
{
    gpio_set_level(motor_L_DIS, (left_enable != 0U) ? ENABLE : DISABLE);
    gpio_set_level(motor_R_DIS, (right_enable != 0U) ? ENABLE : DISABLE);
    gpio_set_level(motor_T_DIS, (steering_enable != 0U) ? ENABLE : DISABLE);
}

void Motor_stop_all(void)
{
    motor_driver_set_pwm(&motor_l_driver, 0.0f);
    motor_driver_set_pwm(&motor_r_driver, 0.0f);
    motor_driver_set_pwm(&servo_driver, 0.0f);

    Motor_rear_speed_pid_reset();

    gpio_set_level(motor_L_DIS, DISABLE);
    gpio_set_level(motor_R_DIS, DISABLE);
    gpio_set_level(motor_T_DIS, DISABLE);
}

/**
 * @brief  转向电机串级 PID 控制周期函数。
 * @param  duty 目标输入，duty->motor_speed 表示目标角度，单位 deg。
 * @note   建议按 MOTOR_CONTROL_HZ 固定周期调用。外环位置 PID 输出目标速度，内环速度 PID 输出 PWM。
 */
void Servo_control(MOTOR_DUTY *duty)
{
    float position_error;
    float pwm_output;
    float output_limit;

    if(duty == 0)
    {
        motor_driver_set_pwm(&servo_driver, 0.0f);
        return;
    }

    /* 不读取硬件编码器，PID 直接使用 encoder.c 的角度和速度结果。 */
    servo_pid.angle_deg = car_angle;
    servo_pid.speed_dps = car_speed[0];

    servo_pid.target_angle_deg = motor_normalize_angle(duty->motor_speed);
    position_error = motor_angle_error(servo_pid.target_angle_deg, servo_pid.angle_deg);

    servo_pid.target_speed_dps = motor_pid_update_by_error(&servo_pid.position,
                                                           position_error,
                                                           servo_pid.angle_deg,
                                                           MOTOR_CONTROL_DT,
                                                           ZF_FALSE);

    pwm_output = motor_pid_update(&servo_pid.speed,
                                  servo_pid.target_speed_dps,
                                  servo_pid.speed_dps,
                                  MOTOR_CONTROL_DT,
                                  ZF_TRUE);

    pwm_output += duty->feedforward_pwm;
    output_limit = servo_pid.speed.gain.output_limit;
    if(output_limit <= 0.0f)
    {
        output_limit = servo_driver.pwm_limit;
    }
    pwm_output = motor_clampf(pwm_output, -output_limit, output_limit);
    servo_pid.speed.state.output = pwm_output;

    motor_driver_set_pwm(&servo_driver, pwm_output);
}

void Servo_position_control(MOTOR_DUTY *duty)
{
    float position_error;
    float pwm_output;
    float output_limit;

    if(duty == 0)
    {
        motor_driver_set_pwm(&servo_driver, 0.0f);
        return;
    }

    /* 不读取硬件编码器，直接使用 encoder.c 已经计算好的角度和速度。 */
    servo_pid.angle_deg = car_angle;
    servo_pid.speed_dps = car_speed[0];
    servo_pid.target_angle_deg = motor_normalize_angle(duty->motor_speed);
    position_error = motor_angle_error(servo_pid.target_angle_deg, servo_pid.angle_deg);

    /* The steering target is rate limited by the Ackermann layer, therefore
     * derivative-on-error is acceptable and does not produce a large setpoint
     * kick during normal operation. */
    pwm_output = motor_pid_update_by_error(&servo_position_pid,
                                           position_error,
                                           servo_pid.angle_deg,
                                           MOTOR_CONTROL_DT,
                                           ZF_FALSE);
    pwm_output += duty->feedforward_pwm;

    output_limit = servo_position_pid.gain.output_limit;
    if(output_limit <= 0.0f)
    {
        output_limit = servo_driver.pwm_limit;
    }
    pwm_output = motor_clampf(pwm_output, -output_limit, output_limit);
    servo_position_pid.state.output = pwm_output;
    motor_driver_set_pwm(&servo_driver, pwm_output);
}

/**
 * @brief  设置转向电机目标角度。
 * @param  angle_deg 目标角度，单位 deg，内部会归一化到 0~360。
 */
void Servo_set_angle(float angle_deg)
{
    servo_duty.motor_speed = motor_normalize_angle(angle_deg);
}

/**
 * @brief  设置转向电机串级 PID 参数。
 * @param  position_gain 位置环 PID 参数，传 0 表示保持原参数。
 * @param  speed_gain    速度环 PID 参数，传 0 表示保持原参数。
 * @note   设置后会清空 PID 运行状态，避免沿用旧积分和旧误差。
 */
void Servo_set_pid_gain(const MOTOR_PID_GAIN *position_gain, const MOTOR_PID_GAIN *speed_gain)
{
    if(position_gain != 0)
    {
        motor_pid_copy_gain(&servo_pid.position, position_gain);
    }

    if(speed_gain != 0)
    {
        motor_pid_copy_gain(&servo_pid.speed, speed_gain);
    }

    Servo_pid_reset();
}

void Servo_set_position_pid_gain(const MOTOR_PID_GAIN *position_gain)
{
    if(position_gain == 0)
    {
        return;
    }

    motor_pid_copy_gain(&servo_position_pid, position_gain);
    motor_pid_reset(&servo_position_pid);
}

float Servo_get_angle(void)
{
    return car_angle;
}

/**
 * @brief  清空转向电机串级 PID 的运行状态。
 * @note   保留 PID 参数和当前目标角，只清积分、误差等 PID 运行量；编码器
 *         速度滤波状态由 encoder.c 独立维护。
 */
void Servo_pid_reset(void)
{
    motor_pid_reset(&servo_pid.position);
    motor_pid_reset(&servo_pid.speed);
    motor_pid_reset(&servo_position_pid);
    servo_pid.target_angle_deg = motor_normalize_angle(servo_duty.motor_speed);
    servo_pid.target_speed_dps = 0.0f;
    servo_pid.speed_dps = 0.0f;
}
