#include "zf_common_headfile.h"

/* 主板 S6：P33.4 为低位，P33.5 为高位，两路均为低电平有效。 */
#define BOARD_MODE_BIT0_PIN              (P33_4)
#define BOARD_MODE_BIT1_PIN              (P33_5)

uint8 carmode = 0;

CarMode_e BoardModeSwitch_Init(void)
{
    uint8_t bit0;
    uint8_t bit1;

    /* 配置上拉输入，拨码断开时稳定读取为高电平。 */
    gpio_init(BOARD_MODE_BIT0_PIN, GPI, 1U, GPI_PULL_UP);
    gpio_init(BOARD_MODE_BIT1_PIN, GPI, 1U, GPI_PULL_UP);

    /* 唯一一次硬件采样：拨码闭合接地，因此 GPIO=0 对应逻辑位 1。 */
    bit0 = (gpio_get_level(BOARD_MODE_BIT0_PIN) == 0U) ? 1U : 0U;
    bit1 = (gpio_get_level(BOARD_MODE_BIT1_PIN) == 0U) ? 1U : 0U;

    carmode=(bit0 | (uint8_t)(bit1 << 1U));
    printf("%d\r\n",carmode);
    return (CarMode_e)carmode;
}
