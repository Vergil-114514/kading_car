#include "zf_common_headfile.h"

/*
 * 测试模式分成两个时间域：
 * - 主循环 TestMode_Task：处理按键上升沿和 PID 参数修改；
 * - 5 ms TestMode_5msCallback：执行通道切换、失联保护和电机闭环。
 *
 * 遥控器四个测试拨码只在本模式生效，主板 P33.4/P33.5 仍只负责选择模式。
 * 为避免台架测试误动作，只有“LoRa 有效且恰好一个拨码开启”时才会使能输出。
 */

/* 固定控制周期和后轮目标角速度斜坡。 */
#define TEST_CONTROL_DT                 (0.005f)
#define TEST_WHEEL_ACCEL_RAD_S2         (20.0f)
/* 小于该角速度视为停止，速度 PID 输出及积分直接清零。 */
#define TEST_STOP_RAD_S                 (0.05f)
/* 后轮速度环每按一次 +/- 键的 Kp、Ki、Kd 步长。 */
#define TEST_REAR_KP_STEP               (10.0f)
#define TEST_REAR_KI_STEP               (5.0f)
#define TEST_REAR_KD_STEP               (0.1f)
/* 转向位置环量纲不同，因此使用更小的独立调节步长。 */
#define TEST_STEER_KP_STEP              (1.0f)
#define TEST_STEER_KI_STEP              (0.1f)
#define TEST_STEER_KD_STEP              (0.05f)
/* 在线调参允许的绝对上限，防止长按/误触导致参数失控。 */
#define TEST_PID_KP_MAX                 (5000.0f)
#define TEST_PID_KI_MAX                 (2000.0f)
#define TEST_PID_KD_MAX                 (500.0f)

/** 单个后轮速度 PID 的运行状态。 */
#if 0 /* 旧的测试模式私有 PID 已停用，后轮 PID 统一由 motor.c 实现。 */
typedef struct
{
    float integral;          /**< 已包含 Ki 的积分输出项。 */
    float last_measurement;  /**< 上周期编码器速度，用于微分。 */
    float output;            /**< 最近一次限幅后的 PWM 输出。 */
    uint8_t initialized;     /**< 0 表示下一周期跳过微分。 */
} TestSpeedPidState_t;
#endif

/* 屏幕显示文本的顺序必须与 TestModeChannel_e 枚举一致。 */
static const char *const g_channel_names[] =
{
    "NO SWITCH",
    "STEERING PID",
    "LEFT REAR PID",
    "RIGHT REAR PID",
    "DRIVE TEST",
    "SWITCH CONFLICT"
};

/* 车辆几何配置，以及三个可独立修改的 PID 参数副本。 */
static ACKERMANN_CONTROL_CONFIG g_vehicle_config;
static ACKERMANN_PID_GAIN g_steering_gain;
static ACKERMANN_PID_GAIN g_left_gain;
static ACKERMANN_PID_GAIN g_right_gain;
/* 后轮目标斜坡状态和转向机构机械中心。 */
static float g_left_ramped_target;
static float g_right_ramped_target;
static float g_steering_center_deg;
/* 用于检测普通按键 0->1 上升沿，按住不会连续修改参数。 */
static uint8_t g_previous_key[4];
/* active 由菜单生命周期控制，outputs_enabled 记录当前硬件使能状态。 */
static volatile uint8_t g_active;
static volatile uint8_t g_outputs_enabled;
/* 参数修改和控制器复位均通过挂起标志在安全时机执行。 */
static volatile uint8_t g_apply_gain_pending;
static volatile uint8_t g_reset_pending;
static volatile TestModeChannel_e g_channel;
static volatile TestModePidTerm_e g_selected_term;
/* 当前周期的调试/显示镜像，不参与下一周期控制计算。 */
static volatile TestModeStatus_t g_status;

static float test_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float test_clampf(float value, float min_value, float max_value)
{
    if(value > max_value) { return max_value; }
    if(value < min_value) { return min_value; }
    return value;
}

static float test_approach(float current, float target, float max_delta)
{
    /* 每周期最多改变 max_delta，形成确定的角加速度限制。 */
    float delta = target - current;
    if(delta > max_delta) { return current + max_delta; }
    if(delta < -max_delta) { return current - max_delta; }
    return target;
}

static float test_angle_error(float target_deg, float measured_deg)
{
    /* 将角度误差折算到 [-180,180]，兼容跨越编码器零点的情况。 */
    float error = target_deg - measured_deg;
    while(error > 180.0f) { error -= 360.0f; }
    while(error < -180.0f) { error += 360.0f; }
    return error;
}

/** 将菜单使用的参数类型转换为 motor.c 的 PID 参数类型。 */
static void test_copy_gain_to_motor(MOTOR_PID_GAIN *destination,
                                    const ACKERMANN_PID_GAIN *source)
{
    destination->kp = source->kp;
    destination->ki = source->ki;
    destination->kd = source->kd;
    destination->integral_limit = source->integral_limit;
    destination->output_limit = source->output_limit;
    destination->deadband = source->deadband;
}

/** 将 motor.c 当前生效的参数复制到菜单调参副本。 */
static void test_copy_gain_from_motor(ACKERMANN_PID_GAIN *destination,
                                      const MOTOR_PID_GAIN *source)
{
    destination->kp = source->kp;
    destination->ki = source->ki;
    destination->kd = source->kd;
    destination->integral_limit = source->integral_limit;
    destination->output_limit = source->output_limit;
    destination->deadband = source->deadband;
}

#if 0 /* 保留迁移前算法仅作历史对照，不参与编译和控制。 */
static void test_pid_reset(TestSpeedPidState_t *state)
{
    state->integral = 0.0f;
    state->last_measurement = 0.0f;
    state->output = 0.0f;
    state->initialized = 0U;
}

static float test_pid_update(TestSpeedPidState_t *state,
                             const ACKERMANN_PID_GAIN *gain,
                             float target,
                             float measurement)
{
    float error;
    float derivative;
    float integral_candidate;
    float output;
    uint8_t saturating;

    /* 零速时清空历史状态，防止积分残留让车轮继续转动。 */
    if(test_absf(target) <= TEST_STOP_RAD_S)
    {
        test_pid_reset(state);
        return 0.0f;
    }

    error = target - measurement;
    /* 小误差按零处理，以抑制编码器量化噪声引起的 PWM 抖动。 */
    if(test_absf(error) <= gain->deadband) { error = 0.0f; }
    if(state->initialized != 0U)
    {
        /* 对测量值微分，摇杆目标突变时不会出现微分冲击。 */
        derivative = -(measurement - state->last_measurement) / TEST_CONTROL_DT;
    }
    else
    {
        derivative = 0.0f;
        state->initialized = 1U;
    }

    /* integral 直接保存 Ki*积分(error)，随后与 P/D 项直接相加。 */
    integral_candidate = state->integral + gain->ki * error * TEST_CONTROL_DT;
    integral_candidate = test_clampf(integral_candidate,
                                     -test_absf(gain->integral_limit),
                                     test_absf(gain->integral_limit));
    output = gain->kp * error + integral_candidate + gain->kd * derivative;
    /* 输出饱和且误差仍推动同方向饱和时冻结积分，防止 windup。 */
    saturating = ((output > gain->output_limit) && (error > 0.0f))
              || ((output < -gain->output_limit) && (error < 0.0f));
    if(saturating == 0U)
    {
        state->integral = integral_candidate;
    }
    else
    {
        output = gain->kp * error + state->integral + gain->kd * derivative;
    }
    output = test_clampf(output, -gain->output_limit, gain->output_limit);

    /* 台架测试不允许 PWM 与目标方向相反，降低编码器/相序接反风险。 */
    if(((target > 0.0f) && (output < 0.0f))
    || ((target < 0.0f) && (output > 0.0f)))
    {
        output = 0.0f;
    }
    state->last_measurement = measurement;
    state->output = output;
    return output;
}

#endif

static TestModeChannel_e test_decode_switches(const uint8_t switches[4])
{
    uint8_t count = 0U;
    uint8_t index = 0U;
    uint8_t i;

    /* 先统计开启数量：0 个=停机，多个=冲突，仅 1 个才允许测试。 */
    for(i = 0U; i < 4U; ++i)
    {
        if(switches[i] != 0U)
        {
            ++count;
            index = i;
        }
    }
    if(count == 0U) { return TEST_MODE_CHANNEL_NONE; }
    if(count > 1U) { return TEST_MODE_CHANNEL_SWITCH_CONFLICT; }
    /* 索引映射集中定义在 lora_remote.h，避免协议变化时散落修改。 */
    if(index == LORA_REMOTE_SWITCH_STEERING_INDEX)
    {
        return TEST_MODE_CHANNEL_STEERING;
    }
    if(index == LORA_REMOTE_SWITCH_LEFT_REAR_INDEX)
    {
        return TEST_MODE_CHANNEL_LEFT_REAR;
    }
    if(index == LORA_REMOTE_SWITCH_RIGHT_REAR_INDEX)
    {
        return TEST_MODE_CHANNEL_RIGHT_REAR;
    }
    return TEST_MODE_CHANNEL_DRIVE;
}

static ACKERMANN_PID_GAIN *test_gain(TestModeChannel_e channel)
{
    /* DRIVE/NONE/CONFLICT 没有独立 PID 参数页面，返回空指针。 */
    if(channel == TEST_MODE_CHANNEL_STEERING) { return &g_steering_gain; }
    if(channel == TEST_MODE_CHANNEL_LEFT_REAR) { return &g_left_gain; }
    if(channel == TEST_MODE_CHANNEL_RIGHT_REAR) { return &g_right_gain; }
    return 0;
}

static void test_reset_controllers(void)
{
    /* 每次换通道或改参数都从零积分、零斜坡开始。 */
    Motor_rear_speed_pid_reset();
    Servo_pid_reset();
    g_left_ramped_target = 0.0f;
    g_right_ramped_target = 0.0f;
    g_reset_pending = 0U;
}

static void test_apply_gains(void)
{
    MOTOR_PID_GAIN steering_gain;
    MOTOR_PID_GAIN left_gain;
    MOTOR_PID_GAIN right_gain;

    /* motor 模块使用等价但独立的类型，因此逐字段转换转向 PID。 */
    steering_gain.kp = g_steering_gain.kp;
    steering_gain.ki = g_steering_gain.ki;
    steering_gain.kd = g_steering_gain.kd;
    steering_gain.integral_limit = g_steering_gain.integral_limit;
    steering_gain.output_limit = g_steering_gain.output_limit;
    steering_gain.deadband = g_steering_gain.deadband;
    Servo_set_position_pid_gain(&steering_gain);
    /* 后轮参数写入共享区，遥控模式下次进入时会加载这些调参结果。 */
    test_copy_gain_to_motor(&left_gain, &g_left_gain);
    test_copy_gain_to_motor(&right_gain, &g_right_gain);
    Motor_set_rear_speed_pid_gains(&left_gain, &right_gain);
    test_reset_controllers();
    g_apply_gain_pending = 0U;
}

/**
 * @brief 把测试模式的目标轮速和 encoder.c 反馈提交给 motor.c。
 * @note 未使能的通道输出 0 并清空其 PID，适用于左右后轮单独台架测试。
 */
static void test_run_rear_speed_control(float left_target,
                                        float right_target,
                                        float left_measured,
                                        float right_measured,
                                        uint8_t left_enabled,
                                        uint8_t right_enabled,
                                        MOTOR_REAR_SPEED_STATUS *left_status,
                                        MOTOR_REAR_SPEED_STATUS *right_status)
{
    MOTOR_REAR_SPEED_INPUT left_input;
    MOTOR_REAR_SPEED_INPUT right_input;

    left_input.target_rad_s = left_target;
    left_input.measured_rad_s = left_measured;
    left_input.feedforward_pwm = 0.0f;
    left_input.output_sign = g_vehicle_config.left_motor_sign;
    left_input.pwm_limit = TEST_MODE_REAR_PWM_LIMIT;
    left_input.enabled = left_enabled;

    right_input.target_rad_s = right_target;
    right_input.measured_rad_s = right_measured;
    right_input.feedforward_pwm = 0.0f;
    right_input.output_sign = g_vehicle_config.right_motor_sign;
    right_input.pwm_limit = TEST_MODE_REAR_PWM_LIMIT;
    right_input.enabled = right_enabled;

    Motor_rear_speed_control(&left_input, &right_input, 0U);
    Motor_get_rear_speed_status(left_status, right_status);
}

static void test_clear_status(void)
{
    /* 清显示值不改 PID 参数；转向测量仍保留当前编码器角度。 */
    g_status.target_a = 0.0f;
    g_status.measured_a = 0.0f;
    g_status.error_a = 0.0f;
    g_status.pwm_a = 0.0f;
    g_status.target_b = 0.0f;
    g_status.measured_b = 0.0f;
    g_status.error_b = 0.0f;
    g_status.pwm_b = 0.0f;
    g_status.steering_target_deg = g_steering_center_deg;
    g_status.steering_measured_deg = Servo_get_angle();
    g_status.steering_pwm = 0.0f;
}

static void test_select_channel(TestModeChannel_e channel)
{
    /*
     * 通道切换先无条件关断全部电机，再清控制器，最后只使能指定通道。
     * 即使遥控器拨码从一个通道直接切到另一个，也不会短暂同时输出。
     */
    Motor_stop_all();
    g_outputs_enabled = 0U;
    test_reset_controllers();
    g_channel = channel;
    test_clear_status();

    if(g_apply_gain_pending != 0U) { test_apply_gains(); }
    /* 非转向测试时也先把目标设为机械中心，但未使能转向则不会驱动。 */
    Servo_set_angle(g_steering_center_deg);
    if(channel == TEST_MODE_CHANNEL_STEERING)
    {
        /* 仅使能前轮转向电机。 */
        Motor_enable_channels(0U, 0U, 1U);
        g_outputs_enabled = 1U;
    }
    else if(channel == TEST_MODE_CHANNEL_LEFT_REAR)
    {
        /* 仅使能左后轮，右后轮和转向保持关闭。 */
        Motor_enable_channels(1U, 0U, 0U);
        g_outputs_enabled = 1U;
    }
    else if(channel == TEST_MODE_CHANNEL_RIGHT_REAR)
    {
        /* 仅使能右后轮。 */
        Motor_enable_channels(0U, 1U, 0U);
        g_outputs_enabled = 1U;
    }
    else if(channel == TEST_MODE_CHANNEL_DRIVE)
    {
        /* 遥控运动测试需要左右后轮和转向三路共同工作。 */
        Motor_enable_channels(1U, 1U, 1U);
        g_outputs_enabled = 1U;
    }
}

static void test_adjust_gain(TestModeChannel_e channel, int8_t direction)
{
    ACKERMANN_PID_GAIN *gain = test_gain(channel);
    float step;

    /* 没有选择单电机 PID 通道时，增减键不产生任何效果。 */
    if(gain == 0) { return; }
    /* 转向位置环和后轮速度环使用各自适合的调节步长。 */
    if(g_selected_term == TEST_MODE_PID_KP)
    {
        step = (channel == TEST_MODE_CHANNEL_STEERING)
             ? TEST_STEER_KP_STEP : TEST_REAR_KP_STEP;
        gain->kp = test_clampf(gain->kp + (float)direction * step,
                              0.0f, TEST_PID_KP_MAX);
    }
    else if(g_selected_term == TEST_MODE_PID_KI)
    {
        step = (channel == TEST_MODE_CHANNEL_STEERING)
             ? TEST_STEER_KI_STEP : TEST_REAR_KI_STEP;
        gain->ki = test_clampf(gain->ki + (float)direction * step,
                              0.0f, TEST_PID_KI_MAX);
    }
    else
    {
        step = (channel == TEST_MODE_CHANNEL_STEERING)
             ? TEST_STEER_KD_STEP : TEST_REAR_KD_STEP;
        gain->kd = test_clampf(gain->kd + (float)direction * step,
                              0.0f, TEST_PID_KD_MAX);
    }
    /* 只修改参数副本；5 ms 控制点会统一应用并清空旧积分。 */
    g_apply_gain_pending = 1U;
}

static void test_run_drive(const LoraRemoteState_t *remote,
                           const ACKERMANN_CONTROL_TELEMETRY *telemetry,
                           float steer_input,
                           float throttle_input)
{
    float center_speed = throttle_input * TEST_MODE_DRIVE_MAX_SPEED_MPS;
    float steering_rad = steer_input * TEST_MODE_MAX_STEERING_DEG
                       * ACKERMANN_DEG_TO_RAD;
    float left_mps;
    float right_mps;
    float left_target;
    float right_target;
    MOTOR_REAR_SPEED_STATUS left_motor_status;
    MOTOR_REAR_SPEED_STATUS right_motor_status;
    float steering_target = g_steering_center_deg
                          + steer_input * TEST_MODE_MAX_STEERING_DEG;

    /* remote 已在调用者中完成归一化，保留参数便于以后扩展按键限速。 */
    (void)remote;
    /* 根据阿克曼几何计算弯道内外侧后轮应有的线速度差。 */
    Ackermann_electronic_differential(center_speed, steering_rad,
                                      &g_vehicle_config, &left_mps, &right_mps);
    /* v/r 转换为编码器速度环目标 rad/s，并进行独立限幅。 */
    left_target = test_clampf(left_mps / g_vehicle_config.wheel_radius_m,
                              -TEST_MODE_MAX_WHEEL_RAD_S,
                              TEST_MODE_MAX_WHEEL_RAD_S);
    right_target = test_clampf(right_mps / g_vehicle_config.wheel_radius_m,
                               -TEST_MODE_MAX_WHEEL_RAD_S,
                               TEST_MODE_MAX_WHEEL_RAD_S);
    /* 限制目标变化速度，避免测试时突然满杆造成机械冲击。 */
    g_left_ramped_target = test_approach(
        g_left_ramped_target, left_target,
        TEST_WHEEL_ACCEL_RAD_S2 * TEST_CONTROL_DT);
    g_right_ramped_target = test_approach(
        g_right_ramped_target, right_target,
        TEST_WHEEL_ACCEL_RAD_S2 * TEST_CONTROL_DT);
    /* 左右轮各自使用自己的参数、积分状态和编码器反馈。 */
    test_run_rear_speed_control(
        g_left_ramped_target,
        g_right_ramped_target,
        telemetry->left_measured_rad_s,
        telemetry->right_measured_rad_s,
        (test_absf(g_left_ramped_target) > TEST_STOP_RAD_S) ? 1U : 0U,
        (test_absf(g_right_ramped_target) > TEST_STOP_RAD_S) ? 1U : 0U,
        &left_motor_status,
        &right_motor_status);
    /* 转向电机使用位置编码器闭环，不使用电流采样。 */
    Servo_set_angle(steering_target);
    Servo_position_control(&servo_duty);

    /* 整车测试页面中 A=左后轮，B=右后轮，转向单独显示。 */
    g_status.target_a = left_motor_status.target_rad_s;
    g_status.measured_a = left_motor_status.measured_rad_s;
    g_status.error_a = left_motor_status.error_rad_s;
    g_status.pwm_a = left_motor_status.pwm;
    g_status.target_b = right_motor_status.target_rad_s;
    g_status.measured_b = right_motor_status.measured_rad_s;
    g_status.error_b = right_motor_status.error_rad_s;
    g_status.pwm_b = right_motor_status.pwm;
    g_status.steering_target_deg = steering_target;
    g_status.steering_measured_deg = Servo_get_angle();
    g_status.steering_pwm = servo_position_pid.state.output;
}

void TestMode_Init(void)
{
    uint8_t i;
    MOTOR_PID_GAIN left_motor_gain;
    MOTOR_PID_GAIN right_motor_gain;

    /* 加载轴距 0.8 m、轮距 1.0 m、轮半径 0.2 m 及默认 PID。 */
    Ackermann_get_default_config(&g_vehicle_config);
    g_steering_gain = g_vehicle_config.steering_position_pid;
    g_left_gain = g_vehicle_config.left_speed_pid;
    g_right_gain = g_vehicle_config.right_speed_pid;
    /* 如果共享区已有调参结果，则覆盖默认的左右后轮参数。 */
    Motor_get_rear_speed_pid_gains(&left_motor_gain, &right_motor_gain);
    test_copy_gain_from_motor(&g_left_gain, &left_motor_gain);
    test_copy_gain_from_motor(&g_right_gain, &right_motor_gain);
    /* 测试模式始终把后轮输出限制在安全 PWM 范围内。 */
    g_left_gain.output_limit = TEST_MODE_REAR_PWM_LIMIT;
    g_right_gain.output_limit = TEST_MODE_REAR_PWM_LIMIT;
    g_left_gain.integral_limit = test_clampf(g_left_gain.integral_limit,
                                             0.0f,
                                             TEST_MODE_REAR_PWM_LIMIT);
    g_right_gain.integral_limit = test_clampf(g_right_gain.integral_limit,
                                              0.0f,
                                              TEST_MODE_REAR_PWM_LIMIT);
    g_steering_center_deg = Ackermann_get_steering_center();
    for(i = 0U; i < 4U; ++i) { g_previous_key[i] = 0U; }
    g_active = 0U;
    g_outputs_enabled = 0U;
    g_apply_gain_pending = 1U;
    g_reset_pending = 0U;
    g_channel = TEST_MODE_CHANNEL_NONE;
    g_selected_term = TEST_MODE_PID_KP;
    test_reset_controllers();
    test_clear_status();
}

void TestMode_Enter(void)
{
    /* 禁止自动驾驶控制器与测试模式同时操作三个电机。 */
    Ackermann_control_enable(0U);
    TestMode_EmergencyStop();
    g_active = 1U;
}

void TestMode_Task(void)
{
    LoraRemoteState_t remote;
    TestModeChannel_e channel;
    uint8_t i;

    if(g_active == 0U) { return; }
    LoraRemote_GetState(&remote);
    if(remote.link_ok == 0U)
    {
        /* 失联时清按键历史，重连后当前按下状态可重新产生一次边沿。 */
        for(i = 0U; i < 4U; ++i) { g_previous_key[i] = 0U; }
        return;
    }
    channel = test_decode_switches(remote.switch_key);

    /* K0 上升沿：Kp -> Ki -> Kd -> Kp，仅单电机 PID 通道有效。 */
    if((remote.key[LORA_REMOTE_KEY_SELECT_TERM_INDEX] != 0U)
    && (g_previous_key[LORA_REMOTE_KEY_SELECT_TERM_INDEX] == 0U)
    && (channel >= TEST_MODE_CHANNEL_STEERING)
    && (channel <= TEST_MODE_CHANNEL_RIGHT_REAR))
    {
        g_selected_term = (TestModePidTerm_e)(
            ((uint8_t)g_selected_term + 1U) % TEST_MODE_PID_TERM_COUNT);
    }
    /* K1 上升沿：只清 PID 运行状态，不把 Kp/Ki/Kd 数值清零。 */
    if((remote.key[LORA_REMOTE_KEY_CLEAR_PID_INDEX] != 0U)
    && (g_previous_key[LORA_REMOTE_KEY_CLEAR_PID_INDEX] == 0U))
    {
        g_reset_pending = 1U;
    }
    /* K2/K3 上升沿：按所选步长减小/增大当前 PID 参数。 */
    if((remote.key[LORA_REMOTE_KEY_DECREASE_INDEX] != 0U)
    && (g_previous_key[LORA_REMOTE_KEY_DECREASE_INDEX] == 0U))
    {
        test_adjust_gain(channel, -1);
    }
    if((remote.key[LORA_REMOTE_KEY_INCREASE_INDEX] != 0U)
    && (g_previous_key[LORA_REMOTE_KEY_INCREASE_INDEX] == 0U))
    {
        test_adjust_gain(channel, 1);
    }
    /* 最后保存本帧按键状态，供下一轮检测 0->1 变化。 */
    for(i = 0U; i < 4U; ++i) { g_previous_key[i] = remote.key[i]; }
}

void TestMode_5msCallback(void)
{
    LoraRemoteState_t remote;
    ACKERMANN_CONTROL_TELEMETRY telemetry;
    MOTOR_REAR_SPEED_STATUS left_motor_status;
    MOTOR_REAR_SPEED_STATUS right_motor_status;
    TestModeChannel_e requested;
    float steer_input;
    float throttle_input;
    float target;

    if(g_active == 0U) { return; }
    LoraRemote_GetState(&remote);
    g_status.lora_link_ok = remote.link_ok;
    g_status.lora_age_ms = remote.age_ms;
    g_status.selected_term = g_selected_term;

    /* 失联保护优先于通道解码和 PID，旧摇杆数据绝不继续驱动电机。 */
    if(remote.link_ok == 0U)
    {
        TestMode_EmergencyStop();
        return;
    }
    /* 遥控器拨码决定测试对象；它不参与主板工作模式选择。 */
    requested = test_decode_switches(remote.switch_key);
    /* 换通道时 test_select_channel 会先关全部输出再选择性使能。 */
    if(requested != g_channel) { test_select_channel(requested); }
    g_status.channel = requested;
    /* 无选择或多开冲突均保持停机，只更新屏幕状态。 */
    if((requested == TEST_MODE_CHANNEL_NONE)
    || (requested == TEST_MODE_CHANNEL_SWITCH_CONFLICT))
    {
        return;
    }
    /* 参数/复位请求只在控制周期边界处理，避免半周期状态不一致。 */
    if(g_apply_gain_pending != 0U) { test_apply_gains(); }
    if(g_reset_pending != 0U) { test_reset_controllers(); }

    /* 一次读取本周期的两个后轮编码器速度快照。 */
    Ackermann_get_telemetry(&telemetry);
    /* 摇杆死区和满量程处理统一由 LoRa 适配层完成；左、上为正。 */
    steer_input = LoraRemote_NormalizeAxis(
        remote.joystick[LORA_REMOTE_STEERING_AXIS_INDEX])
        * LORA_REMOTE_STEERING_SIGN;
    throttle_input = LoraRemote_NormalizeAxis(
        remote.joystick[LORA_REMOTE_THROTTLE_AXIS_INDEX])
        * LORA_REMOTE_THROTTLE_SIGN;

    if(requested == TEST_MODE_CHANNEL_STEERING)
    {
        /* SW0：左摇杆 X 给出中心 +/-20 度目标，仅运行转向位置环。 */
        target = g_steering_center_deg
               + steer_input * TEST_MODE_MAX_STEERING_DEG;
        /* 显式写零，形成除硬件通道禁用之外的第二重保护。 */
        test_run_rear_speed_control(0.0f, 0.0f,
                                    telemetry.left_measured_rad_s,
                                    telemetry.right_measured_rad_s,
                                    0U, 0U,
                                    &left_motor_status,
                                    &right_motor_status);
        Servo_set_angle(target);
        Servo_position_control(&servo_duty);
        g_status.target_a = target;
        g_status.measured_a = Servo_get_angle();
        g_status.error_a = test_angle_error(target, g_status.measured_a);
        g_status.pwm_a = servo_position_pid.state.output;
        g_status.steering_target_deg = target;
        g_status.steering_measured_deg = g_status.measured_a;
        g_status.steering_pwm = g_status.pwm_a;
    }
    else if(requested == TEST_MODE_CHANNEL_LEFT_REAR)
    {
        /* SW1：左摇杆 Y 给出左后轮 +/-10 rad/s 目标。 */
        target = throttle_input * TEST_MODE_MAX_WHEEL_RAD_S;
        g_left_ramped_target = test_approach(
            g_left_ramped_target, target,
            TEST_WHEEL_ACCEL_RAD_S2 * TEST_CONTROL_DT);
        test_run_rear_speed_control(
            g_left_ramped_target, 0.0f,
            telemetry.left_measured_rad_s,
            telemetry.right_measured_rad_s,
            (test_absf(g_left_ramped_target) > TEST_STOP_RAD_S) ? 1U : 0U,
            0U,
            &left_motor_status,
            &right_motor_status);
        /* 未测试的右后轮始终明确写 0。 */
        g_status.target_a = left_motor_status.target_rad_s;
        g_status.measured_a = left_motor_status.measured_rad_s;
        g_status.error_a = left_motor_status.error_rad_s;
        g_status.pwm_a = left_motor_status.pwm;
    }
    else if(requested == TEST_MODE_CHANNEL_RIGHT_REAR)
    {
        /* SW2：左摇杆 Y 给出右后轮 +/-10 rad/s 目标。 */
        target = throttle_input * TEST_MODE_MAX_WHEEL_RAD_S;
        g_right_ramped_target = test_approach(
            g_right_ramped_target, target,
            TEST_WHEEL_ACCEL_RAD_S2 * TEST_CONTROL_DT);
        test_run_rear_speed_control(
            0.0f, g_right_ramped_target,
            telemetry.left_measured_rad_s,
            telemetry.right_measured_rad_s,
            0U,
            (test_absf(g_right_ramped_target) > TEST_STOP_RAD_S) ? 1U : 0U,
            &left_motor_status,
            &right_motor_status);
        /* 未测试的左后轮始终明确写 0。 */
        g_status.target_a = right_motor_status.target_rad_s;
        g_status.measured_a = right_motor_status.measured_rad_s;
        g_status.error_a = right_motor_status.error_rad_s;
        g_status.pwm_a = right_motor_status.pwm;
    }
    else
    {
        /* SW3：三电机联合运动测试，包含阿克曼电子差速。 */
        test_run_drive(&remote, &telemetry, steer_input, throttle_input);
    }
}

void TestMode_Exit(void)
{
    /* 先禁止后续周期执行，再进行一次完整停机。 */
    g_active = 0U;
    TestMode_EmergencyStop();
}

void TestMode_EmergencyStop(void)
{
    /* 幂等安全停机：无论从失联、模式切换还是人工退出进入都可重复调用。 */
    Motor_stop_all();
    g_outputs_enabled = 0U;
    g_channel = TEST_MODE_CHANNEL_NONE;
    test_reset_controllers();
    test_clear_status();
    g_status.channel = TEST_MODE_CHANNEL_NONE;
    g_status.lora_link_ok = 0U;
}

static void test_display_gain(const ACKERMANN_PID_GAIN *gain)
{
    /* '>' 标记当前 K0 选择的参数；K2/K3 只修改这一项。 */
    ips200_show_string(0U, 132U,
        (g_selected_term == TEST_MODE_PID_KP) ? ">Kp" : " Kp");
    ips200_show_float(40U, 132U, gain->kp, 7U, 2U);
    ips200_show_string(0U, 150U,
        (g_selected_term == TEST_MODE_PID_KI) ? ">Ki" : " Ki");
    ips200_show_float(40U, 150U, gain->ki, 7U, 2U);
    ips200_show_string(0U, 168U,
        (g_selected_term == TEST_MODE_PID_KD) ? ">Kd" : " Kd");
    ips200_show_float(40U, 168U, gain->kd, 7U, 2U);
    ips200_show_string(0U, 194U, "K0 term K2- K3+");
    ips200_show_string(0U, 212U, "K1 clear PID state");
}

void TestMode_Display(void)
{
    TestModeStatus_t status;
    ACKERMANN_PID_GAIN *gain;

    /* 屏幕刷新在主循环中运行，读取一次快照后完成整页绘制。 */
    TestMode_GetStatus(&status);
    ips200_clear();
    ips200_show_string(0U, 0U, "TEST / LORA");
    ips200_show_string(130U, 0U,
        (status.lora_link_ok != 0U) ? "LINK OK" : "LINK LOST");
    ips200_show_string(0U, 18U, "Channel:");
    ips200_show_string(72U, 18U, g_channel_names[status.channel]);

    if(status.lora_link_ok == 0U)
    {
        /* 失联时控制回调已经关闭输出，页面只负责提示原因。 */
        ips200_show_string(0U, 52U, "FAILSAFE: MOTORS OFF");
    }
    else if(status.channel == TEST_MODE_CHANNEL_SWITCH_CONFLICT)
    {
        /* 多个遥控器测试拨码同时打开属于不安全输入。 */
        ips200_show_string(0U, 52U, "ONLY ONE SWITCH ON");
        ips200_show_string(0U, 76U, "MOTORS DISABLED");
    }
    else if(status.channel == TEST_MODE_CHANNEL_NONE)
    {
        ips200_show_string(0U, 52U, "Select one TX switch");
    }
    else if(status.channel == TEST_MODE_CHANNEL_DRIVE)
    {
        /* 联合测试同时展示左右后轮和转向三组闭环数据。 */
        ips200_show_string(0U, 42U, "L Tar/Mea/PWM");
        ips200_show_float(0U, 60U, status.target_a, 6U, 2U);
        ips200_show_float(72U, 60U, status.measured_a, 6U, 2U);
        ips200_show_int(150U, 60U, (int32)status.pwm_a, 6U);
        ips200_show_string(0U, 80U, "R Tar/Mea/PWM");
        ips200_show_float(0U, 98U, status.target_b, 6U, 2U);
        ips200_show_float(72U, 98U, status.measured_b, 6U, 2U);
        ips200_show_int(150U, 98U, (int32)status.pwm_b, 6U);
        ips200_show_string(0U, 120U, "Steer Tar/Mea/PWM");
        ips200_show_float(0U, 138U, status.steering_target_deg, 6U, 1U);
        ips200_show_float(72U, 138U, status.steering_measured_deg, 6U, 1U);
        ips200_show_int(150U, 138U, (int32)status.steering_pwm, 6U);
    }
    else
    {
        /* 单电机页面展示目标、反馈、误差、PWM 及当前 PID。 */
        ips200_show_string(0U, 42U, "Target:");
        ips200_show_float(72U, 42U, status.target_a, 8U, 2U);
        ips200_show_string(0U, 62U, "Measure:");
        ips200_show_float(72U, 62U, status.measured_a, 8U, 2U);
        ips200_show_string(0U, 82U, "Error:");
        ips200_show_float(72U, 82U, status.error_a, 8U, 2U);
        ips200_show_string(0U, 102U, "PWM:");
        ips200_show_int(72U, 102U, (int32)status.pwm_a, 8U);
        gain = test_gain(status.channel);
        if(gain != 0) { test_display_gain(gain); }
    }
    ips200_show_string(0U, 244U, "SW0 steer SW1 left");
    ips200_show_string(0U, 262U, "SW2 right SW3 drive");
    ips200_show_string(0U, 282U, "LX steer LY speed");
}

TestModeChannel_e TestMode_GetChannel(void)
{
    return g_channel;
}

void TestMode_GetStatus(TestModeStatus_t *status)
{
    /* 逐字段复制便于后续按平台需要加入临界区，而不暴露内部全局变量。 */
    if(status == 0) { return; }
    status->channel = g_status.channel;
    status->selected_term = g_status.selected_term;
    status->lora_link_ok = g_status.lora_link_ok;
    status->lora_age_ms = g_status.lora_age_ms;
    status->target_a = g_status.target_a;
    status->measured_a = g_status.measured_a;
    status->error_a = g_status.error_a;
    status->pwm_a = g_status.pwm_a;
    status->target_b = g_status.target_b;
    status->measured_b = g_status.measured_b;
    status->error_b = g_status.error_b;
    status->pwm_b = g_status.pwm_b;
    status->steering_target_deg = g_status.steering_target_deg;
    status->steering_measured_deg = g_status.steering_measured_deg;
    status->steering_pwm = g_status.steering_pwm;
}

uint8_t TestMode_GetPidGains(TestModeChannel_e channel,
                             ACKERMANN_PID_GAIN *gain)
{
    /* 仅转向、左后轮、右后轮三个通道具有可读写 PID 参数。 */
    ACKERMANN_PID_GAIN *source = test_gain(channel);
    if((source == 0) || (gain == 0)) { return 1U; }
    *gain = *source;
    return 0U;
}

uint8_t TestMode_SetPidGains(TestModeChannel_e channel,
                             const ACKERMANN_PID_GAIN *gain)
{
    ACKERMANN_PID_GAIN *destination = test_gain(channel);
    /*
     * 外部写参数必须满足非负增益、正输出上限和测试 PWM 限值。
     * 校验失败返回 1，原参数保持不变；成功返回 0 并延迟到控制点应用。
     */
    if((destination == 0) || (gain == 0)
    || (gain->kp < 0.0f) || (gain->ki < 0.0f) || (gain->kd < 0.0f)
    || (gain->integral_limit < 0.0f)
    || (gain->output_limit <= 0.0f)
    || (gain->output_limit > TEST_MODE_REAR_PWM_LIMIT)
    || (gain->deadband < 0.0f))
    {
        return 1U;
    }
    *destination = *gain;
    g_apply_gain_pending = 1U;
    return 0U;
}
