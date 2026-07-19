#include "zf_common_headfile.h"

/*
 * LoRa 遥控模式的数据流：
 *   右摇杆 Y 原始值 +/-1900 -> 带死区归一化 [-1,1]
 *                       -> S5=0：开环 PWM [-8000,8000]
 *                       -> S5=1：电子差速目标 + 左右后轮速度 PID
 *   左摇杆 X 原始值 +/-1900 -> 带死区归一化 [-1,1]
 *                       -> 最大转角 +/-20 deg -> 转向位置 PID
 *
 * S5 控制方式切换时先清零后轮 PWM 和速度 PID；闭环模式下编码器无效则停车。
 */

static ACKERMANN_CONTROL_CONFIG g_config;
static float g_steering_center_deg;
static volatile uint8_t g_active;
static volatile uint8_t g_outputs_enabled;
static volatile uint8_t g_rear_closed_loop;
static volatile RemoteModeStatus_t g_status;

static float remote_mode_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void remote_mode_select_rear_control(uint8_t closed_loop)
{
    closed_loop = (closed_loop != 0U) ? 1U : 0U;
    if(closed_loop == g_rear_closed_loop)
    {
        return;
    }

    /* Remove open-loop PWM and closed-loop integral before changing owner. */
    Motor_set_pwm(0.0f, 0.0f);
    Motor_rear_speed_pid_reset();
    g_rear_closed_loop = closed_loop;
}

static void remote_mode_run_ackermann_rear(
    const ACKERMANN_CONTROL_TELEMETRY *telemetry,
    float throttle_input)
{
    MOTOR_REAR_SPEED_INPUT left_input;
    MOTOR_REAR_SPEED_INPUT right_input;
    MOTOR_REAR_SPEED_STATUS left_status;
    MOTOR_REAR_SPEED_STATUS right_status;
    float center_speed_mps;
    float steering_angle_rad;
    float left_speed_mps;
    float right_speed_mps;
    float left_target_mps;
    float right_target_mps;
    float largest_target_mps;
    float target_scale;
    uint8_t rear_enabled;

    if(telemetry->encoder_valid == 0U)
    {
        Motor_set_pwm(0.0f, 0.0f);
        Motor_rear_speed_pid_reset();
        g_status.left_target_mps = 0.0f;
        g_status.left_measured_mps = telemetry->left_measured_mps;
        g_status.left_pwm = 0.0f;
        g_status.right_target_mps = 0.0f;
        g_status.right_measured_mps = telemetry->right_measured_mps;
        g_status.right_pwm = 0.0f;
        return;
    }

    center_speed_mps = throttle_input * g_config.max_vehicle_speed_mps;
    /* Differential speed follows the measured steering encoder angle only.
     * The joystick target is intentionally not used in this calculation. */
    steering_angle_rad = Ackermann_encoder_angle_to_road_steering_rad(
        Servo_get_angle(),
        &g_config);
    Ackermann_electronic_differential(center_speed_mps,
                                      steering_angle_rad,
                                      &g_config,
                                      &left_speed_mps,
                                      &right_speed_mps);
    left_target_mps = left_speed_mps;
    right_target_mps = right_speed_mps;
    largest_target_mps = remote_mode_absf(left_target_mps);
    if(remote_mode_absf(right_target_mps) > largest_target_mps)
    {
        largest_target_mps = remote_mode_absf(right_target_mps);
    }
    if(largest_target_mps > g_config.max_wheel_speed_mps)
    {
        target_scale = g_config.max_wheel_speed_mps / largest_target_mps;
        left_target_mps *= target_scale;
        right_target_mps *= target_scale;
    }
    rear_enabled = (throttle_input != 0.0f) ? 1U : 0U;

    left_input.target_mps = left_target_mps;
    left_input.measured_mps = telemetry->left_measured_mps;
    left_input.feedforward_pwm = 0.0f;
    left_input.output_sign = g_config.left_motor_sign;
    left_input.pwm_limit = REMOTE_MODE_PWM_LIMIT;
    left_input.enabled = rear_enabled;

    right_input.target_mps = right_target_mps;
    right_input.measured_mps = telemetry->right_measured_mps;
    right_input.feedforward_pwm = 0.0f;
    right_input.output_sign = g_config.right_motor_sign;
    right_input.pwm_limit = REMOTE_MODE_PWM_LIMIT;
    right_input.enabled = rear_enabled;

    Motor_rear_speed_control(&left_input, &right_input, 0U);
    Motor_get_rear_speed_status(&left_status, &right_status);
    g_status.left_target_mps = left_status.target_mps;
    g_status.left_measured_mps = left_status.measured_mps;
    g_status.left_pwm = left_status.pwm;
    g_status.right_target_mps = right_status.target_mps;
    g_status.right_measured_mps = right_status.measured_mps;
    g_status.right_pwm = right_status.pwm;
}

/** 模式切换或失联时清空所有闭环控制器的历史状态。 */
static void remote_mode_reset(void)
{
    /* 遥控后轮虽然是开环，但仍清空速度 PID，防止切换模式后带入旧积分。 */
    Motor_rear_speed_pid_reset();
    Servo_pid_reset();
}

/** 清空屏幕状态快照；本函数不会使能或驱动电机。 */
static void remote_mode_clear_status(void)
{
    g_status.rear_closed_loop = g_rear_closed_loop;
    g_status.throttle_input = 0.0f;
    g_status.steering_input = 0.0f;
    g_status.left_target_mps = 0.0f;
    g_status.left_measured_mps = 0.0f;
    g_status.left_pwm = 0.0f;
    g_status.right_target_mps = 0.0f;
    g_status.right_measured_mps = 0.0f;
    g_status.right_pwm = 0.0f;
    g_status.steering_target_deg = g_steering_center_deg;
    g_status.steering_measured_deg = Servo_get_angle();
    g_status.steering_pwm = 0.0f;
}

void RemoteMode_Init(void)
{
    /* 读取实车几何、限速、电机方向和转向参数。 */
    Ackermann_get_default_config(&g_config);
    g_steering_center_deg = Ackermann_get_steering_center();
    g_config.steering_center_encoder_deg = g_steering_center_deg;
    g_active = 0U;
    g_outputs_enabled = 0U;
    g_rear_closed_loop = 0U;
    remote_mode_reset();
    remote_mode_clear_status();
}

void RemoteMode_Enter(void)
{
    /* 禁止自动循迹控制器和遥控模式同时写三个电机。 */
    Ackermann_control_enable(0U);
    RemoteMode_EmergencyStop();
    g_active = 1U;
}

void RemoteMode_5msCallback(void)
{
    LoraRemoteState_t remote;
    ACKERMANN_CONTROL_TELEMETRY telemetry;
    float steer_input;
    float throttle_input;
    float rear_pwm;
    float left_pwm;
    float right_pwm;
    float steering_target;
    uint8_t requested_closed_loop;

    if(g_active == 0U) { return; }

    LoraRemote_GetState(&remote);
    g_status.link_ok = remote.link_ok;
    g_status.link_age_ms = remote.age_ms;

    /* 失联优先级最高，旧摇杆数据绝不能继续驱动电机。 */
    if(remote.link_ok == 0U)
    {
        RemoteMode_EmergencyStop();
        return;
    }

    if(g_outputs_enabled == 0U)
    {
        /* 先清零 PWM/PID，再使能三路驱动，避免出现上个模式的残留输出。 */
        Motor_stop_all();
        remote_mode_reset();
        Servo_set_angle(g_steering_center_deg);
        Motor_enable_channels(1U, 1U, 1U);
        g_outputs_enabled = 1U;
    }

    /* 左摇杆左右控制转向，右摇杆上下控制速度；左推、上推均为正。 */
    steer_input = LoraRemote_NormalizeAxis(
        remote.joystick[LORA_REMOTE_STEERING_AXIS_INDEX])
        * LORA_REMOTE_STEERING_SIGN;
    throttle_input = LoraRemote_NormalizeAxis(
        remote.joystick[LORA_REMOTE_THROTTLE_AXIS_INDEX])
        * LORA_REMOTE_THROTTLE_SIGN;

    Ackermann_get_telemetry(&telemetry);
    requested_closed_loop = remote.switch_key[LORA_REMOTE_SWITCH_S5_INDEX];
    remote_mode_select_rear_control(requested_closed_loop);
    g_status.rear_closed_loop = g_rear_closed_loop;

    /*
     * S5=0 后轮开环映射：
     *   throttle =  0.0 -> PWM =     0
     *   throttle = +1.0 -> PWM = +8000
     *   throttle = -1.0 -> PWM = -8000
     * 两个后轮使用相同幅值，各自乘电机安装方向符号。
     */
    if(g_rear_closed_loop == 0U)
    {
        rear_pwm = throttle_input * REMOTE_MODE_PWM_LIMIT;
        left_pwm = rear_pwm * g_config.left_motor_sign;
        right_pwm = rear_pwm * g_config.right_motor_sign;
        Motor_set_pwm(left_pwm, right_pwm);
        g_status.left_target_mps = 0.0f;
        g_status.left_measured_mps = telemetry.left_measured_mps;
        g_status.left_pwm = left_pwm;
        g_status.right_target_mps = 0.0f;
        g_status.right_measured_mps = telemetry.right_measured_mps;
        g_status.right_pwm = right_pwm;
    }
    else
    {
        remote_mode_run_ackermann_rear(&telemetry, throttle_input);
    }

    /* 转向仍使用位置编码器闭环，摇杆满量程对应允许的最大转角。 */
    /* Preserve the original remote-stick direction: positive stick input
     * increases the steering encoder target. */
    steering_target = Ackermann_road_steering_to_encoder_angle_deg(
        steer_input * REMOTE_MODE_MAX_STEERING_DEG * ACKERMANN_DEG_TO_RAD,
        &g_config);
    Servo_set_angle(steering_target);
    Servo_position_control(&servo_duty);

    /* 公共状态同时供菜单和调试器读取。 */
    g_status.throttle_input = throttle_input;
    g_status.steering_input = steer_input;
    g_status.steering_target_deg = steering_target;
    g_status.steering_measured_deg = Servo_get_angle();
    g_status.steering_pwm = servo_position_pid.state.output;
}

void RemoteMode_Exit(void)
{
    g_active = 0U;
    RemoteMode_EmergencyStop();
}

void RemoteMode_EmergencyStop(void)
{
    /* 幂等停机：重复调用仍保持三路关闭、PWM 为 0、PID 状态清零。 */
    Motor_stop_all();
    g_outputs_enabled = 0U;
    g_rear_closed_loop = 0U;
    remote_mode_reset();
    remote_mode_clear_status();
    g_status.link_ok = 0U;
}

void RemoteMode_Display(void)
{
    RemoteModeStatus_t status;

    RemoteMode_GetStatus(&status);
    ips200_clear();
    ips200_show_string(0U, 0U,
        (status.rear_closed_loop != 0U)
        ? "REMOTE ACK DIFF" : "REMOTE OPEN LOOP");
    ips200_show_string(130U, 0U,
        (status.link_ok != 0U) ? "LINK OK" : "LINK LOST");
    if(status.link_ok == 0U)
    {
        ips200_show_string(0U, 36U, "FAILSAFE: MOTORS OFF");
        ips200_show_string(0U, 60U, "Waiting for LoRa...");
        return;
    }

    ips200_show_string(0U, 36U, "Throttle / Steering");
    ips200_show_float(0U, 56U, status.throttle_input, 6U, 3U);
    ips200_show_float(100U, 56U, status.steering_input, 6U, 3U);
    ips200_show_string(0U, 82U, "L m/s / PWM");
    ips200_show_float(0U, 102U, status.left_measured_mps, 7U, 2U);
    ips200_show_int(120U, 102U, (int32)status.left_pwm, 7U);
    ips200_show_string(0U, 128U, "R m/s / PWM");
    ips200_show_float(0U, 148U, status.right_measured_mps, 7U, 2U);
    ips200_show_int(120U, 148U, (int32)status.right_pwm, 7U);
    ips200_show_string(0U, 174U, "Steer Tar/Mea/PWM");
    ips200_show_float(0U, 194U, status.steering_target_deg, 6U, 1U);
    ips200_show_float(72U, 194U, status.steering_measured_deg, 6U, 1U);
    ips200_show_int(150U, 194U, (int32)status.steering_pwm, 6U);
}

void RemoteMode_GetStatus(RemoteModeStatus_t *status)
{
    if(status == 0) { return; }
    status->link_ok = g_status.link_ok;
    status->rear_closed_loop = g_status.rear_closed_loop;
    status->link_age_ms = g_status.link_age_ms;
    status->throttle_input = g_status.throttle_input;
    status->steering_input = g_status.steering_input;
    status->left_target_mps = g_status.left_target_mps;
    status->left_measured_mps = g_status.left_measured_mps;
    status->left_pwm = g_status.left_pwm;
    status->right_target_mps = g_status.right_target_mps;
    status->right_measured_mps = g_status.right_measured_mps;
    status->right_pwm = g_status.right_pwm;
    status->steering_target_deg = g_status.steering_target_deg;
    status->steering_measured_deg = g_status.steering_measured_deg;
    status->steering_pwm = g_status.steering_pwm;
}
