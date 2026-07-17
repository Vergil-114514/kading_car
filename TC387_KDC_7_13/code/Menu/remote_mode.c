#include "zf_common_headfile.h"

/*
 * LoRa 遥控模式的数据流：
 *   油门原始值 +/-1900 -> 带死区归一化 [-1,1]
 *                       -> 开环 PWM [-8000,8000] -> 左右后轮
 *   转向原始值 +/-1900 -> 带死区归一化 [-1,1]
 *                       -> 最大转角 +/-20 deg -> 转向位置 PID
 *
 * 遥控模式的后轮不执行速度 PID，也不使用编码器速度修正 PWM。编码器速度只
 * 保留给屏幕显示。测试模式和自动 Ackermann 模式仍使用 motor.c 的速度 PID。
 */

static ACKERMANN_CONTROL_CONFIG g_config;
static float g_steering_center_deg;
static volatile uint8_t g_active;
static volatile uint8_t g_outputs_enabled;
static volatile RemoteModeStatus_t g_status;

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
    g_status.throttle_input = 0.0f;
    g_status.steering_input = 0.0f;
    g_status.left_target_rad_s = 0.0f;
    g_status.left_measured_rad_s = 0.0f;
    g_status.left_pwm = 0.0f;
    g_status.right_target_rad_s = 0.0f;
    g_status.right_measured_rad_s = 0.0f;
    g_status.right_pwm = 0.0f;
    g_status.steering_target_deg = g_steering_center_deg;
    g_status.steering_measured_deg = Servo_get_angle();
    g_status.steering_pwm = 0.0f;
}

void RemoteMode_Init(void)
{
    /* 读取左右电机方向和转向参数；遥控模式不使用其中的车速/PID配置。 */
    Ackermann_get_default_config(&g_config);
    g_steering_center_deg = Ackermann_get_steering_center();
    g_active = 0U;
    g_outputs_enabled = 0U;
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

    /* LoRa 层将实测 +/-1900 去除死区后归一化；左推、上推均为正。 */
    steer_input = LoraRemote_NormalizeAxis(
        remote.joystick[LORA_REMOTE_STEERING_AXIS_INDEX])
        * LORA_REMOTE_STEERING_SIGN;
    throttle_input = LoraRemote_NormalizeAxis(
        remote.joystick[LORA_REMOTE_THROTTLE_AXIS_INDEX])
        * LORA_REMOTE_THROTTLE_SIGN;

    /*
     * 后轮开环映射：
     *   throttle =  0.0 -> PWM =     0
     *   throttle = +1.0 -> PWM = +8000
     *   throttle = -1.0 -> PWM = -8000
     * 两个后轮使用相同幅值，各自乘电机安装方向符号。
     */
    rear_pwm = throttle_input * REMOTE_MODE_PWM_LIMIT;
    left_pwm = rear_pwm * g_config.left_motor_sign;
    right_pwm = rear_pwm * g_config.right_motor_sign;
    Motor_set_pwm(left_pwm, right_pwm);

    /* 转向仍使用位置编码器闭环，摇杆满量程对应允许的最大转角。 */
    steering_target = g_steering_center_deg
                    + steer_input * REMOTE_MODE_MAX_STEERING_DEG;
    Servo_set_angle(steering_target);
    Servo_position_control(&servo_duty);

    /* 后轮编码器不参与控制，只读取 Ackermann 遥测快照用于屏幕显示。 */
    Ackermann_get_telemetry(&telemetry);
    g_status.throttle_input = throttle_input;
    g_status.steering_input = steer_input;
    g_status.left_target_rad_s = 0.0f;
    g_status.left_measured_rad_s = telemetry.left_measured_rad_s;
    g_status.left_pwm = left_pwm;
    g_status.right_target_rad_s = 0.0f;
    g_status.right_measured_rad_s = telemetry.right_measured_rad_s;
    g_status.right_pwm = right_pwm;
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
    remote_mode_reset();
    remote_mode_clear_status();
    g_status.link_ok = 0U;
}

void RemoteMode_Display(void)
{
    RemoteModeStatus_t status;

    RemoteMode_GetStatus(&status);
    ips200_clear();
    ips200_show_string(0U, 0U, "REMOTE OPEN LOOP");
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
    ips200_show_string(0U, 82U, "L Speed / PWM");
    ips200_show_float(0U, 102U, status.left_measured_rad_s, 7U, 2U);
    ips200_show_int(120U, 102U, (int32)status.left_pwm, 7U);
    ips200_show_string(0U, 128U, "R Speed / PWM");
    ips200_show_float(0U, 148U, status.right_measured_rad_s, 7U, 2U);
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
    status->link_age_ms = g_status.link_age_ms;
    status->throttle_input = g_status.throttle_input;
    status->steering_input = g_status.steering_input;
    status->left_target_rad_s = g_status.left_target_rad_s;
    status->left_measured_rad_s = g_status.left_measured_rad_s;
    status->left_pwm = g_status.left_pwm;
    status->right_target_rad_s = g_status.right_target_rad_s;
    status->right_measured_rad_s = g_status.right_measured_rad_s;
    status->right_pwm = g_status.right_pwm;
    status->steering_target_deg = g_status.steering_target_deg;
    status->steering_measured_deg = g_status.steering_measured_deg;
    status->steering_pwm = g_status.steering_pwm;
}
