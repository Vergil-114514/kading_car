#ifndef _TEST_MODE_H_
#define _TEST_MODE_H_

#include <stdint.h>
#include "../ackermann_control.h"

/**
 * @file test_mode.h
 * @brief 三电机独立测试、整车遥控测试和 PID 在线调参接口。
 *
 * 本模式只在主板 S6 选择 TEST 时运行。遥控器四个拨码选择测试对象，
 * 四个普通按键选择/修改 PID；任意时刻仅允许一个测试拨码开启。
 */

/** 遥控器测试拨码解码后的测试通道。 */
typedef enum
{
    TEST_MODE_CHANNEL_NONE = 0,       /**< 没有拨码开启，所有电机关闭。 */
    TEST_MODE_CHANNEL_STEERING,       /**< 只测试前轮转向位置环。 */
    TEST_MODE_CHANNEL_LEFT_REAR,      /**< 只测试左后轮速度环。 */
    TEST_MODE_CHANNEL_RIGHT_REAR,     /**< 只测试右后轮速度环。 */
    TEST_MODE_CHANNEL_DRIVE,          /**< 三电机配合的遥控运动测试。 */
    TEST_MODE_CHANNEL_SWITCH_CONFLICT /**< 多个拨码同时开启，禁止输出。 */
} TestModeChannel_e;

/** 当前由按键选择、准备增减的 PID 参数项。 */
typedef enum
{
    TEST_MODE_PID_KP = 0,
    TEST_MODE_PID_KI,
    TEST_MODE_PID_KD,
    TEST_MODE_PID_TERM_COUNT
} TestModePidTerm_e;

/** 测试页面和外部调试工具使用的只读状态。 */
typedef struct
{
    TestModeChannel_e channel;        /**< 当前测试对象。 */
    TestModePidTerm_e selected_term;  /**< 当前选择的 Kp/Ki/Kd。 */
    uint8_t lora_link_ok;             /**< 1=链路有效。 */
    uint16_t lora_age_ms;             /**< 最近遥控帧年龄，单位 ms。 */
    float target_a;                   /**< 单电机目标，或整车测试左轮目标。 */
    float measured_a;                 /**< 与 target_a 对应的反馈值。 */
    float error_a;                    /**< target_a - measured_a。 */
    float pwm_a;                      /**< A 通道控制输出。 */
    float target_b;                   /**< 整车测试右轮目标。 */
    float measured_b;                 /**< 整车测试右轮反馈。 */
    float error_b;                    /**< target_b - measured_b。 */
    float pwm_b;                      /**< B 通道控制输出。 */
    float steering_target_deg;        /**< 整车测试时的转向目标角。 */
    float steering_measured_deg;      /**< 转向编码器反馈角。 */
    float steering_pwm;               /**< 转向位置环输出。 */
} TestModeStatus_t;

/* 测试时的机械/速度/PWM 安全限值。 */
#define TEST_MODE_MAX_STEERING_DEG        (20.0f)
#define TEST_MODE_MAX_WHEEL_RAD_S         (10.0f)
#define TEST_MODE_DRIVE_MAX_SPEED_MPS     (1.5f)
#define TEST_MODE_REAR_PWM_LIMIT          (2000.0f)

/** 初始化车辆参数、PID 副本和测试状态，不使能电机。 */
void TestMode_Init(void);
/** 切入测试模式并保持全部输出关闭，等待遥控器选择通道。 */
void TestMode_Enter(void);
/** 主循环任务：检测按键上升沿并修改 PID 参数。 */
void TestMode_Task(void);
/** 5 ms 周期闭环：解码拨码、使能指定电机并执行控制。 */
void TestMode_5msCallback(void);
/** 退出测试模式并停机。 */
void TestMode_Exit(void);
/** 关闭三路输出、清空 PID 状态并取消当前测试通道。 */
void TestMode_EmergencyStop(void);
/** 绘制当前通道、链路、PID 和电机反馈信息。 */
void TestMode_Display(void);

/** 返回当前测试通道。 */
TestModeChannel_e TestMode_GetChannel(void);
/** 复制当前测试状态，供显示或调试工具读取。 */
void TestMode_GetStatus(TestModeStatus_t *status);
/** 读取指定电机通道的 PID 参数；成功返回 0。 */
uint8_t TestMode_GetPidGains(TestModeChannel_e channel,
                             ACKERMANN_PID_GAIN *gain);
/** 校验并设置指定通道 PID；参数会在下一个安全时机应用。 */
uint8_t TestMode_SetPidGains(TestModeChannel_e channel,
                             const ACKERMANN_PID_GAIN *gain);

#endif
