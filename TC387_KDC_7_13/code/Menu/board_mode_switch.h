#ifndef _BOARD_MODE_SWITCH_H_
#define _BOARD_MODE_SWITCH_H_

#include <stdint.h>

/**
 * @file board_mode_switch.h
 * @brief 主板 S6 两位拨码开关的上电模式选择接口。
 *
 * P33.4 为模式位 bit0，P33.5 为模式位 bit1，硬件上拉且低电平有效。
 * 两路 GPIO 只在 BoardModeSwitch_Init() 中读取一次，并将模式直接返回给 Menu。
 * 运行中不再访问拨码，必须复位主控后才会读取新的拨码位置。
 *
 * 主板 S6 只负责整车工作模式，与 LoRa 遥控器上的四个测试拨码完全独立。
 */

/** 两位拨码组成的 0~3 对应四种工作模式。 */
typedef enum
{
    CAR_MODE_TRACK = 0,       /**< 00：循迹模式，当前预留外部接口。 */
    CAR_MODE_SAVE_POINT = 1,  /**< 01：存点模式，当前预留外部接口。 */
    CAR_MODE_TEST = 2,        /**< 10：电机/PID 测试模式。 */
    CAR_MODE_REMOTE = 3,      /**< 11：LoRa 遥控运动模式。 */
    CAR_MODE_COUNT            /**< 模式数量，仅用于数组边界检查。 */
} CarMode_e;

extern uint8 carmode;

/**
 * 初始化 P33.4/P33.5，并立即读取一次当前拨码位置。
 * @return 本次上电选择的工作模式；函数返回后不再读取拨码。
 */
CarMode_e BoardModeSwitch_Init(void);

#endif
