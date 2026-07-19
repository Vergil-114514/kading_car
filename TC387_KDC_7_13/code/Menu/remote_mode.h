#ifndef _REMOTE_MODE_H_
#define _REMOTE_MODE_H_

#include <stdint.h>

/**
 * @file remote_mode.h
 * @brief LoRa 遥控运动模式接口。
 *
 * 左摇杆 X 控制转向，右摇杆 Y 控制后轮。S5=0 时后轮为开环 PWM，S5=1 时
 * 使用 Ackermann 电子差速和左右后轮速度 PID；转向电机始终使用位置 PID。
 * LoRa 超时后立即关闭三个电机驱动。
 */

/** 提供给菜单和调试器读取的遥控模式运行状态。 */
typedef struct
{
    uint8_t rear_closed_loop;        /**< 0=S5 open-loop PWM, 1=Ackermann differential PID. */
    uint8_t link_ok;                 /**< 1=LoRa 有效；0=失联保护。 */
    uint16_t link_age_ms;            /**< 最近一帧数据的时间，单位 ms。 */
    float throttle_input;            /**< 油门归一化值 [-1,1]，上推为正。 */
    float steering_input;            /**< 转向归一化值 [-1,1]，左推为正。 */

    /* 开环遥控时目标速度为 0；S5 闭环时显示电子差速目标。 */
    float left_target_mps;
    float left_measured_mps;         /**< 左后轮线速度，单位 m/s。 */
    float left_pwm;                  /**< 左后轮实际 PWM。 */
    float right_target_mps;
    float right_measured_mps;        /**< 右后轮线速度，单位 m/s。 */
    float right_pwm;                 /**< 右后轮实际 PWM。 */

    float steering_target_deg;       /**< 转向位置 PID 目标角度。 */
    float steering_measured_deg;     /**< 转向编码器测量角度。 */
    float steering_pwm;              /**< 转向位置 PID 输出 PWM。 */
} RemoteModeStatus_t;

/** 最大前轮转角，以及后轮开环/闭环 PWM 最大绝对值。 */
#define REMOTE_MODE_MAX_STEERING_DEG       (20.0f)
#define REMOTE_MODE_PWM_LIMIT              (8000.0f)

/** 初始化遥控状态并保持所有电机关闭。 */
void RemoteMode_Init(void);
/** 进入遥控模式，关闭自动 Ackermann 控制并清空旧输出。 */
void RemoteMode_Enter(void);
/** 每 5 ms 执行一次遥控解析、S5 后轮控制选择和转向位置闭环。 */
void RemoteMode_5msCallback(void);
/** 退出遥控模式并执行安全停机。 */
void RemoteMode_Exit(void);
/** 可重复调用的失联/模式级紧急停机。 */
void RemoteMode_EmergencyStop(void);
/** 绘制遥控输入、编码器和三路 PWM。 */
void RemoteMode_Display(void);
/** 复制当前遥控模式状态。 */
void RemoteMode_GetStatus(RemoteModeStatus_t *status);

#endif
