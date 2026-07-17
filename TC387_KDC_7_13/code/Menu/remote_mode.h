#ifndef _REMOTE_MODE_H_
#define _REMOTE_MODE_H_

#include <stdint.h>

/**
 * @file remote_mode.h
 * @brief LoRa 遥控运动模式接口。
 *
 * 遥控模式忽略遥控器四个测试拨码。左右后轮采用开环 PWM：油门归一化值
 * [-1,1] 直接映射到 [-8000,8000]；转向电机继续使用位置编码器 PID。
 * LoRa 超时后立即关闭三个电机驱动。
 */

/** 提供给菜单和调试器读取的遥控模式运行状态。 */
typedef struct
{
    uint8_t link_ok;                 /**< 1=LoRa 有效；0=失联保护。 */
    uint16_t link_age_ms;            /**< 最近一帧数据的时间，单位 ms。 */
    float throttle_input;            /**< 油门归一化值 [-1,1]，上推为正。 */
    float steering_input;            /**< 转向归一化值 [-1,1]，左推为正。 */

    /* 这两个目标速度字段为兼容原菜单接口保留；开环遥控时固定为 0。 */
    float left_target_rad_s;
    float left_measured_rad_s;       /**< 左后轮编码器计算速度，仅用于显示。 */
    float left_pwm;                  /**< 左后轮实际开环 PWM。 */
    float right_target_rad_s;
    float right_measured_rad_s;      /**< 右后轮编码器计算速度，仅用于显示。 */
    float right_pwm;                 /**< 右后轮实际开环 PWM。 */

    float steering_target_deg;       /**< 转向位置 PID 目标角度。 */
    float steering_measured_deg;     /**< 转向编码器测量角度。 */
    float steering_pwm;              /**< 转向位置 PID 输出 PWM。 */
} RemoteModeStatus_t;

/** 最大前轮转角，以及后轮开环 PWM 最大绝对值。 */
#define REMOTE_MODE_MAX_STEERING_DEG       (20.0f)
#define REMOTE_MODE_PWM_LIMIT              (8000.0f)

/** 初始化遥控状态并保持所有电机关闭。 */
void RemoteMode_Init(void);
/** 进入遥控模式，关闭自动 Ackermann 控制并清空旧输出。 */
void RemoteMode_Enter(void);
/** 每 5 ms 执行一次遥控解析、后轮开环 PWM 和转向位置闭环。 */
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
