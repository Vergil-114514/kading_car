#ifndef _LORA_REMOTE_H_
#define _LORA_REMOTE_H_

#include <stdint.h>

/**
 * @file lora_remote.h
 * @brief LoRa 遥控器数据适配层。
 *
 * 本模块位于 3A22 串口驱动和菜单控制逻辑之间，负责：
 * 1. 将底层接收帧发布为可稳定读取的遥控器状态；
 * 2. 统计最近一帧数据的年龄并给出 link_ok；
 * 3. 将摇杆原始值转换为 [-1.0, 1.0] 的无量纲控制量。
 *
 * 调用约定：
 * - LoraRemote_Task() 放在主循环，用于消费完整 LoRa 帧；
 * - LoraRemote_5msCallback() 每 5 ms 调用，用于链路看门狗计时；
 * - 控制模块通过 LoraRemote_GetState() 读取快照，不直接访问底层驱动。
 */

/* 遥控器实测摇杆范围为 +/-1900；绝对值不超过 50 时按零输入处理。 */
#define LORA_REMOTE_JOYSTICK_FULL_SCALE       (1900.0f)
#define LORA_REMOTE_JOYSTICK_DEAD_ZONE        (50.0f)
/* 用户约定：向左和向上均为正值，因此两个方向系数均为 +1。 */
#define LORA_REMOTE_STEERING_SIGN             (1.0f)
#define LORA_REMOTE_THROTTLE_SIGN             (1.0f)
/* 连续 300 ms 没有发布新帧时，所有使用者必须进入失联保护。 */
#define LORA_REMOTE_TIMEOUT_MS                (300U)

/* 摇杆数组映射：左摇杆 X 控制转向，左摇杆 Y 控制速度。 */
#define LORA_REMOTE_STEERING_AXIS_INDEX       (0U)
#define LORA_REMOTE_THROTTLE_AXIS_INDEX       (1U)

/* 测试模式下，遥控器四个拨码开关与测试通道一一对应。 */
#define LORA_REMOTE_SWITCH_STEERING_INDEX     (0U)
#define LORA_REMOTE_SWITCH_LEFT_REAR_INDEX    (1U)
#define LORA_REMOTE_SWITCH_RIGHT_REAR_INDEX   (2U)
#define LORA_REMOTE_SWITCH_DRIVE_TEST_INDEX   (3U)

/* 测试模式下，遥控器四个普通按键用于选择和修改 PID 参数。 */
#define LORA_REMOTE_KEY_SELECT_TERM_INDEX     (0U)
#define LORA_REMOTE_KEY_CLEAR_PID_INDEX       (1U)
#define LORA_REMOTE_KEY_DECREASE_INDEX        (2U)
#define LORA_REMOTE_KEY_INCREASE_INDEX        (3U)

/** 对上层公开的一致性遥控器快照。 */
typedef struct
{
    int16_t joystick[4];       /**< 四路摇杆原始值，范围约为 [-1900, 1900]。 */
    uint8_t key[4];            /**< 普通按键状态：1=按下，0=松开。 */
    uint8_t switch_key[4];     /**< 遥控器拨码状态：1=开启，0=关闭。 */
    uint32_t sequence;         /**< 适配层发布序号，每发布一帧递增一次。 */
    uint16_t age_ms;           /**< 距离最近一次新帧发布的时间，单位 ms。 */
    uint8_t link_ok;           /**< 1=链路有效，0=超时或尚未收到有效帧。 */
} LoraRemoteState_t;

/** 初始化适配层状态并启动底层 3A22 串口接收。 */
void LoraRemote_Init(void);
/** 主循环任务：读取一帧底层快照并原子式发布给控制模块。 */
void LoraRemote_Task(void);
/** 5 ms 周期回调：更新底层及上层链路看门狗。 */
void LoraRemote_5msCallback(void);
/** 将当前遥控器状态复制到调用者提供的结构体。 */
void LoraRemote_GetState(LoraRemoteState_t *state);
/** 带死区地将摇杆原始值归一化到 [-1.0, 1.0]。 */
float LoraRemote_NormalizeAxis(int16_t raw_value);

#endif
