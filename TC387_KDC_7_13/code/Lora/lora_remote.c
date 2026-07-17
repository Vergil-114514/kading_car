#include "zf_common_headfile.h"

/*
 * LoRa 遥控器适配层实现。
 *
 * 底层 3A22 驱动在串口中断中组帧，本文件在主循环中取走完整帧，再通过
 * 双缓冲发布给 5 ms 控制回调。写缓冲完全更新后才切换 g_active_index，
 * 可以避免控制回调读到“摇杆来自新帧、按键仍来自旧帧”的混合数据。
 */

/** 双缓冲中的内部帧；只保存控制层真正需要的数据。 */
typedef struct
{
    int16_t joystick[4];
    uint8_t key[4];
    uint8_t switch_key[4];
    uint32_t sequence;
} LoraRemoteFrame_t;

/*
 * g_frame[2]：一个供读、一个供写；g_active_index 指向当前可读缓冲。
 * volatile 用于明确这些量可能在主循环和 5 ms 回调之间异步变化。
 */
static volatile LoraRemoteFrame_t g_frame[2];
static volatile uint8_t g_active_index = 0U;
/* 发布序号只在完整新帧发布后递增，也是上层看门狗的新帧判据。 */
static uint32_t g_publish_sequence = 0U;
static uint32_t g_last_watchdog_sequence = 0U;
/* 上电时直接置为超时状态，收到第一帧之前绝不允许电机启动。 */
static volatile uint16_t g_age_ms = LORA_REMOTE_TIMEOUT_MS + 5U;
static volatile uint8_t g_link_ok = 0U;

/** 单精度绝对值，避免为简单操作引入额外数学库依赖。 */
static float remote_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float remote_clampf(float value, float min_value, float max_value)
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

void LoraRemote_Init(void)
{
    uint8_t i;

    /* 两个缓冲都清零，保证底层第一帧到来前上层读到安全中位状态。 */
    for(i = 0U; i < 4U; ++i)
    {
        g_frame[0].joystick[i] = 0;
        g_frame[0].key[i] = 0U;
        g_frame[0].switch_key[i] = 0U;
        g_frame[1].joystick[i] = 0;
        g_frame[1].key[i] = 0U;
        g_frame[1].switch_key[i] = 0U;
    }
    g_frame[0].sequence = 0U;
    g_frame[1].sequence = 0U;
    g_active_index = 0U;
    g_publish_sequence = 0U;
    g_last_watchdog_sequence = 0U;
    g_age_ms = LORA_REMOTE_TIMEOUT_MS + 5U;
    g_link_ok = 0U;
    /* 最后启动串口，避免中断先于本模块状态初始化。 */
    lora3a22_init();
}

void LoraRemote_Task(void)
{
    lora3a22_uart_transfer_dat_struct source;
    uint32 source_sequence;
    uint8_t write_index;
    uint8_t i;

    /* get_snapshot 只返回校验正确且尚未消费的完整帧。 */
    if(lora3a22_get_snapshot(&source, &source_sequence) == 0U)
    {
        return;
    }
    /* 底层序号仅用于底层诊断；适配层使用自己的连续发布序号。 */
    (void)source_sequence;

    /* 始终写非活动缓冲，写完全部字段后再一次性发布。 */
    write_index = (uint8_t)(g_active_index ^ 1U);
    for(i = 0U; i < 4U; ++i)
    {
        g_frame[write_index].joystick[i] = source.joystick[i];
        g_frame[write_index].key[i] = source.key[i];
        g_frame[write_index].switch_key[i] = source.switch_key[i];
    }
    ++g_publish_sequence;
    g_frame[write_index].sequence = g_publish_sequence;
    /* 此赋值是发布点：从这里开始控制回调才会看到新帧。 */
    g_active_index = write_index;
}

void LoraRemote_5msCallback(void)
{
    uint8_t index;
    uint32_t sequence;

    /* 先更新底层链路状态，再用适配层发布序号做第二层看门狗。 */
    lora3a22_5ms_callback();
    index = g_active_index;
    sequence = g_frame[index].sequence;
    if(sequence != g_last_watchdog_sequence)
    {
        /* 检测到主循环刚发布的新帧，链路年龄清零。 */
        g_last_watchdog_sequence = sequence;
        g_age_ms = 0U;
    }
    else if(g_age_ms <= (uint16_t)(65535U - 5U))
    {
        g_age_ms = (uint16_t)(g_age_ms + 5U);
    }
    /* 超过 300 ms 后，测试模式和遥控模式都会立即执行停机。 */
    g_link_ok = (g_age_ms <= LORA_REMOTE_TIMEOUT_MS) ? 1U : 0U;
}

void LoraRemote_GetState(LoraRemoteState_t *state)
{
    uint8_t index;
    uint8_t i;

    if(state == 0)
    {
        return;
    }
    /* 先锁定本次读取使用的缓冲索引，避免复制过程中切换来源。 */
    index = g_active_index;
    for(i = 0U; i < 4U; ++i)
    {
        state->joystick[i] = g_frame[index].joystick[i];
        state->key[i] = g_frame[index].key[i];
        state->switch_key[i] = g_frame[index].switch_key[i];
    }
    state->sequence = g_frame[index].sequence;
    state->age_ms = g_age_ms;
    state->link_ok = g_link_ok;
}

float LoraRemote_NormalizeAxis(int16_t raw_value)
{
    float value = (float)raw_value;
    float magnitude = remote_absf(value);
    float normalized;

    /* 中位附近的小幅抖动直接置零，防止静止时电机缓慢爬行。 */
    if(magnitude <= LORA_REMOTE_JOYSTICK_DEAD_ZONE)
    {
        return 0.0f;
    }
    /* 超过实测满量程的异常值先限幅，确保输出绝不超过 1。 */
    magnitude = remote_clampf(magnitude,
                              LORA_REMOTE_JOYSTICK_DEAD_ZONE,
                              LORA_REMOTE_JOYSTICK_FULL_SCALE);
    /* 去掉死区后重新线性映射，使死区边缘连续地从 0 开始。 */
    normalized = (magnitude - LORA_REMOTE_JOYSTICK_DEAD_ZONE)
               / (LORA_REMOTE_JOYSTICK_FULL_SCALE
                - LORA_REMOTE_JOYSTICK_DEAD_ZONE);
    return (value < 0.0f) ? -normalized : normalized;
}
